#include "RigAnimation.h"
#include <igl/angular_distance.h>
#include <igl/get_seconds.h>
#include <igl/ONE.h>
#include <igl/ZERO.h>

RigAnimation::RigAnimation():
  m_bone_animations(),
  m_pose_at_start_playing(),
  m_keyframe_step(1),
  m_t_start(0),
  m_is_playing_to_from_identity(false)
{
}

bool RigAnimation::similar(
  const Eigen::Quaterniond & A,
  const Eigen::Quaterniond & B)
{
  using namespace igl;
  const double MIN_DIFF = 0.0;
  const double diff = angular_distance(A,B);
  return diff <= MIN_DIFF;
}

bool RigAnimation::is_listening() const
{
  using namespace std;
  for(int b = 0;b<(int)m_bone_animations.size();b++)
  {
    const auto & bone_anim = m_bone_animations[b];
    if(bone_anim.is_listening)
    {
      return true;
    }
  }
  return false;
}

bool RigAnimation::is_recording() const
{
  for(const auto & bone_anim : m_bone_animations)
  {
    if(bone_anim.is_recording)
    {
      return true;
    }
  }
  return false;
}

bool RigAnimation::is_playing() const
{
  for(const auto & bone_anim : m_bone_animations)
  {
    if(bone_anim.is_playing)
    {
      return true;
    }
  }
  return false;
}

bool RigAnimation::record_pose(const RotationList & pose)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  assert(pose.size() == m_bone_animations.size());
  const double t_abs = get_seconds(); 
  bool still_recording = false;
  for(int b = 0;b<(int)m_bone_animations.size();b++)
  {
    auto & bone_anim = m_bone_animations[b];
    if(bone_anim.is_recording)
    {
      still_recording = true;
      const double t = t_abs - bone_anim.t_start;
      auto & keyframes = bone_anim.keyframes;
      const double MIN_T = 0.3;
      // Skip if identical or if we haven't taken a sample in a while
      const auto & q = pose[b];
      if(keyframes.size() == 0)
      {
        const BezierKeyframe<Quaterniond> key(q,q,q);
        bone_anim.keyframes.push_back(make_pair(t,key));
      }else
      {
        const auto & prev = keyframes.back().second.value;
        const double t_diff = (t-keyframes.back().first);
        if(!similar(prev,q) || t_diff > MIN_T)
        {
          // Set Bezier for previous
          bone_anim.keyframes.back().second.next = prev.slerp(1./3.,q);
          const BezierKeyframe<Quaterniond> key( prev.slerp(2./3.,q), q, q);
          bone_anim.keyframes.push_back(make_pair(t,key));
        }
      }
    }
  }
  return still_recording;
}

void RigAnimation::clear(const int n)
{
  m_bone_animations.clear();
  m_bone_animations.resize(n,BoneAnimation());
  // Default is to not be listening!
  set_all_is_listening(false);
}

void RigAnimation::clear()
{
  return clear(m_bone_animations.size());
}

void RigAnimation::set_all_is_recording(const bool v)
{
  for(auto & bone_anim : m_bone_animations)
  {
    bone_anim.is_recording = v;
  }
}

void RigAnimation::set_all_is_listening(const bool v)
{
  for(auto & bone_anim : m_bone_animations)
  {
    bone_anim.is_listening = v;
  }
}

void RigAnimation::start_recording(const RotationList & pose)
{
  using namespace igl;
  assert(pose.size() == m_bone_animations.size());
  m_t_start = get_seconds();
  for(auto & bone_anim : m_bone_animations)
  {
    bone_anim.is_playing = !bone_anim.is_listening;
    bone_anim.is_recording = bone_anim.is_listening;
    if(bone_anim.is_recording)
    {
      bone_anim.keyframes.clear();
    }
    // Either playing or recording, both need rewind
    bone_anim.t_start = m_t_start;
    bone_anim.last_interval = 0;
  }
  //m_pose_at_start_playing = pose;
}

bool RigAnimation::update(const RotationList & pose)
{
  assert(pose.size() == m_bone_animations.size());
  using namespace std;
  const bool still_recording = record_pose(pose);

  if(still_recording)
  {
    return true;
  }

  return false;
}

bool RigAnimation::play(const double t_abs, RotationList & pose)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  bool still_playing = false;
  for(int b = 0;b<(int)m_bone_animations.size();b++)
  {
    // Can't be const if we want to keep track of `last_interval`
    auto & bone_anim  = m_bone_animations[b];
    if(bone_anim.is_playing)
    {
      assert(!bone_anim.is_recording);
      const auto & keyframes = bone_anim.keyframes;
      if(keyframes.size() == 0)
      {
        continue;
      }
      const auto & slerp = [](
        const double t, 
        const Quaterniond & A, 
        const Quaterniond & B) -> Quaterniond
      {
        return A.slerp(t,B);
      };
      still_playing |= 
        BezierKeyframe<Quaterniond>::find(
          bone_anim,
          t_abs,
          pose[b],
          bone_anim.last_interval,
          slerp);
    }
  }

  if(!still_playing)
  {
    // Otherwise restore pose at start_playing()
    //pose = m_pose_at_start_playing;
    if(m_is_playing_to_from_identity)
    {
      m_is_playing_to_from_identity = false;
      clear();
    }
  }

  // Only stop if all are done.
  for(auto & bone_anim : m_bone_animations)
  {
    if(!still_playing && bone_anim.is_playing)
    {
      bone_anim.is_playing = false;
    }
  }

  return still_playing;
}

bool RigAnimation::play(RotationList & pose)
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;
  assert(pose.size() == m_bone_animations.size());
  const double t_abs = get_seconds(); 
  return play(t_abs,pose);
}

void RigAnimation::add_to_reanttweakbar(igl::anttweakbar::ReTwBar & rebar)
{
  using namespace igl;
  using namespace igl::anttweakbar;
  rebar.TwAddVarRW("rig_animation_keyframe_step",TW_TYPE_DOUBLE,
    &m_keyframe_step,"label='keyframe dur.' group='Animation'");
  const auto & noop = [](const void *,void *){};
  const auto & get_keyframes_sizeCB = [](void * v, void * clientData)
  {
    const RigAnimation & ra = *static_cast<const RigAnimation*>(clientData);
    int & np = *(int*)(v);
    np = ra.keyframes_size();
  };
  rebar.TwAddVarCB("rig_animation_keyframes_size",TW_TYPE_INT32,
    noop,get_keyframes_sizeCB,this,
    "label='#keyframes' group='Animation' readonly=true");
  const auto & get_is_listeningCB = [](void * v, void * clientData)
  {
    *(bool*)(v) = static_cast<const RigAnimation*>(clientData)->is_listening();
  };
  const auto & get_is_playingCB= [](void * v, void * clientData)
  {
    *(bool*)(v) = static_cast<const RigAnimation*>(clientData)->is_playing();
  };
  rebar.TwAddVarCB("rig_animation_is_listening",TW_TYPE_BOOLCPP,noop,
    get_is_listeningCB,this,
    "label='listening' group='Animation' readonly=true");
  rebar.TwAddVarCB("rig_animation_is_playing",TW_TYPE_BOOLCPP,noop,
    get_is_playingCB,this, "label='playing' group='Animation' readonly=true");
  const auto & pop_poseCB = [](void *clientData)
  {
    static_cast<RigAnimation*>(clientData)->pop_pose();
  };
  rebar.TwAddButton("rig_animation_pop_pose", pop_poseCB, this,
    "label='pop keyframe' group='Animation' key=Q");
  const auto & clearCB = [](void *clientData)
  {
    static_cast<RigAnimation*>(clientData)->clear();
  };
  rebar.TwAddButton("rig_animation_clear", clearCB, this,
    "label='clear' group='Animation' key=q");

  rebar.TwSetParam( "Animation", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
}

void RigAnimation::start_playing(const RotationList & pose)
{
  using namespace igl;
  using namespace std;
  assert(pose.size() == m_bone_animations.size());
  m_t_start = get_seconds();
  for(auto & bone_anim : m_bone_animations)
  {
    bone_anim.is_playing = true;
    bone_anim.is_recording = false;
    bone_anim.t_start = m_t_start;
    bone_anim.last_interval = 0;
  }
  //m_pose_at_start_playing = pose;
}

void RigAnimation::push_pose(const RotationList & pose)
{
  using namespace std;
  using namespace Eigen;
  assert(pose.size() == m_bone_animations.size());
  // Collect next time value (across theta)
  double t = 0;
  for(int b = 0;b<(int)m_bone_animations.size();b++)
  {
    auto & keyframes = m_bone_animations[b].keyframes;
    if(!keyframes.empty())
    {
      t = max(t,keyframes.back().first + m_keyframe_step);
    }
  }
  // Push keyframes on all theta at same time
  for(int b = 0;b < (int)m_bone_animations.size();b++)
  {
    auto & bone_anim = m_bone_animations[b];
    auto & keyframes = bone_anim.keyframes;
    const auto & v = pose[b];
    keyframes.push_back(make_pair(t,BezierKeyframe<Quaterniond>(v,v,v)));
  }
}

bool RigAnimation::pop_pose()
{
  // Pop one from each theta. Doesn't really make sense if keyframes didn't
  // come from pushing...
  bool popped = false;
  for(auto & bone_anim : m_bone_animations)
  {
    auto & keyframes = bone_anim.keyframes;
    if(!keyframes.empty())
    {
      keyframes.pop_back();
      popped = true;
    }
  }
  return popped;
}

void RigAnimation::start_to_from_identity(const RotationList & pose)
{
  using namespace Eigen;
  push_pose(pose);
  RotationList I(pose.size(),Quaterniond::Identity());
  push_pose(I);
  push_pose(pose);
  m_is_playing_to_from_identity = true;
  start_playing(pose);
}

int RigAnimation::keyframes_size() const
{
  int np = 0;
  for(const auto & bone_anim : bone_animations())
  {
    np = std::max(np,(int)bone_anim.keyframes.size());
  }
  return np;
}

void RigAnimation::set_is_listening( const VectorXb & S)
{
  using namespace std;
  assert((int)m_bone_animations.size() == S.rows());
  for(int b = 0;b<(int)m_bone_animations.size();b++)
  {
    auto & bone_anim = m_bone_animations[b];
    bone_anim.is_listening = S(b);
  }
}

double RigAnimation::t_start() const
{
  return m_t_start;
}
