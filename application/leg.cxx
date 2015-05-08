#include "set_is_fullscreen.h"
#include "EigenConvenience.h"
#include "AugViewport.h"
#include "PuppetReader.h"
#include "PuppetReaderCallbacks.h"
#include "TBT.h"
#include "draw_genie.h"
#include "add_puppetreader_anttweakbar.h"
#include "draw_guide_circle.h"
#include "Log.h"
#include "BezierKeyframe.h"
#include "Animation.h"
#include "Uniped.h"
#include "rigid2d.h"
#include "Callbacks.h"
#include "filter_prefix.h"

#include <igl/report_gl_error.h>
#include <igl/readDMAT.h>
#include <igl/writeDMAT.h>
#include <igl/draw_mesh.h>
#include <igl/material_colors.h>
#include <igl/REDRUM.h>
#include <igl/normalize_row_lengths.h>
#include <igl/C_STR.h>
#include <igl/STR.h>
#include <igl/PI.h>
#include <igl/EPS.h>
#include <igl/get_seconds.h>
#include <igl/ReAntTweakBar.h>
#include <igl/ZERO.h>
#include <igl/snap_points.h>
#include <igl/unproject_to_zero_plane.h>
#include <igl/project.h>
#include <igl/matlab_format.h>
#include <igl/dated_copy.h>
#include <igl/path_to_executable.h>
#include <igl/randperm.h>
#include <igl/cumsum.h>
#include <igl/list_to_matrix.h>
#include <igl/is_file.h>
#include <igl/randperm.h>
#include <igl/png/render_to_png.h>
#include <igl/png/render_to_png_async.h>
#define BINARY_BACKUP_DIR "./binary-backups/"

#include <YImage.hpp>

#ifdef WIN32
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif
#ifndef GLUT_WHEEL_UP
#define GLUT_WHEEL_UP    3
#define GLUT_WHEEL_DOWN  4
#define GLUT_WHEEL_RIGHT 5
#define GLUT_WHEEL_LEFT  6
#define GLUT_ACTIVE_COMMAND 1
#endif

#include <boost/program_options.hpp>

#include <string>
#include <stack>
#include <iostream>
#include <algorithm>
#include <iomanip> 

std::string anim_prefix = "";

bool render_to_png_on_next = false;

const Eigen::Vector3d 
  AESTHETIC_MAX_THETA(2.2,0.2,1.9),
  AESTHETIC_MIN_THETA(-1.5,-1.8,-0.3),
  PUPPET_MAX_THETA(1.5,1.5,1.5),
  PUPPET_MIN_THETA(-1.5,-1.5,-1.5),
  MAX_THETA(PUPPET_MAX_THETA.array().min(AESTHETIC_MAX_THETA.array())),
  MIN_THETA(PUPPET_MIN_THETA.array().max(AESTHETIC_MIN_THETA.array()));
Uniped uniped, simon_says, shadow;
double keyframe_step = 2.;
std::vector<double> samples;
double last_sample_time = 0;
bool flash_on_next = false;

enum GenerateAnimationType
{
  GENERATE_ANIMATION_TYPE_SEQUENTIAL = 0,
  GENERATE_ANIMATION_TYPE_SIMULTANEOUS = 1,
  GENERATE_ANIMATION_TYPE_INPUT = 2,
  NUM_GENERATE_ANIMATION_TYPES = 3
} generate_anim_type = GENERATE_ANIMATION_TYPE_SEQUENTIAL;
int num_simultaneous = 3;
bool is_dragging = false;
bool is_near_rest = false;
double NEAR_EPS = 0.035;
bool is_drawing_progress_bar = false;
int selected_theta = -1;
Eigen::Vector2d m_down,m_drag,p_center;
double down_theta;
double fps =0, mspf =0;
std::vector<std::vector<Uniped::ThetaAnimationType > > target_anims;
Eigen::VectorXi target_anim_order;

// yo dawg, stupid math.h defines log
Log<std::string> lawg;
std::string log_filename = "leg.log";

const Eigen::Vector3d REST_THETA(0.3,-0.4,1.5);

struct State
{
  Eigen::Vector2d pan;
  double zoom;
  State(): pan(850,78),zoom(0.4)
  {}
} s;

std::stack<State> undo_stack;
std::stack<State> redo_stack;

bool is_panning = false;
bool is_shadow_visible = false;
igl::Viewport main_vp(0,0,0,0);
AugViewport genie_vp(0,0,150,150);

igl::ReTwBar rebar;
bool bar_is_visible = true;

PuppetReader * pr;
std::string rebar_in_name = "leg.rbr";
const std::string rebar_out_name = "leg.rbr";

bool is_testing = false;
bool is_fullscreen = false;
int target = 0;
int g_num_targets = 5;
enum ModeType
{
  MODE_TYPE_BEFORE_SIMON = 0, // Displaying a message, waiting for user to hit a key
  MODE_TYPE_SIMON = 1, // Watching simon animation
  MODE_TYPE_BEFORE_RECORDING = 2, // Displaying a message, waiting for user to hit a key
  MODE_TYPE_RECORDING = 3, // Recording animation
  NUM_MODE_TYPES = 4
} mode = MODE_TYPE_BEFORE_SIMON;

inline int num_targets()
{
  return 
    ((generate_anim_type == GENERATE_ANIMATION_TYPE_INPUT)?
       target_anim_order.size():g_num_targets);
};

void test_log(const std::string & msg);
void TW_CALL generate_animCB(void * client_data);
void TW_CALL set_is_animation_playing(const void * v, void * client_data);
void TW_CALL set_is_animation_recording(const void * v, void * client_data);

Eigen::MatrixXd animation_matrix(
  const Uniped::ThetaAnimationType * anims)
{
  using namespace Eigen;
  MatrixXd A(
    anims[0].keyframes.size() + 
    anims[1].keyframes.size() + 
    anims[2].keyframes.size(),2+3);
  {
    int k = 0;
    for(int th = 0;th<3;th++)
    {
      for(auto & keyframe : anims[th].keyframes)
      {
        A(k,0) = th;
        A(k,1) = keyframe.first;
        A(k,2) = keyframe.second.prev;
        A(k,3) = keyframe.second.value;
        A(k,4) = keyframe.second.next;
        k++;
      }
    }
  }
  return A;
}

double append_samples()
{
  using namespace igl;
  const double t = get_seconds();
  samples.push_back(t);
  samples.push_back(uniped.theta(0));
  samples.push_back(uniped.theta(1));
  samples.push_back(uniped.theta(2));
  samples.push_back(simon_says.theta(0));
  samples.push_back(simon_says.theta(1));
  samples.push_back(simon_says.theta(2));
  return t;
}

void test_log_animation(const Uniped & u,const std::string name)
{
  using namespace Eigen;
  using namespace std;
  MatrixXd A = animation_matrix(u.theta_animation);
  RowVectorXd C(A.size());
  copy(A.data(),A.data()+A.size(),C.data());
  test_log(STR(name<<", "<<C.format(IOFormat(FullPrecision,DontAlignCols))));
}

std::string control_type_as_string(
  const Uniped::ControlType & control_type)
{
  switch(control_type)
  {
    case Uniped::CONTROL_TYPE_PUPPET:
      return "custom device";
    case Uniped::CONTROL_TYPE_MOUSE:
      return "mouse";
    case Uniped::CONTROL_TYPE_KEYBOARD:
      return "keyboard";
    default:
      assert(false);
      return "unknown";
  }
}

void TW_CALL set_control_type(const void * v, void * /*client_data*/)
{
  const Uniped::ControlType & c = *static_cast<const Uniped::ControlType *>(v);
  test_log(STR("control_type,"<<
    control_type_as_string(c)));
  uniped.control_type = c;
  switch(uniped.control_type)
  {
    case Uniped::CONTROL_TYPE_KEYBOARD:
      selected_theta = -1;
    case Uniped::CONTROL_TYPE_MOUSE:
      uniped.color.leftCols(3).setConstant(1);
      break;
    case Uniped::CONTROL_TYPE_PUPPET:
      selected_theta = -1;
      if(is_testing)
      {
        uniped.color.leftCols(3).setConstant(1);
      }
      break;
    default:
      break;
  }
}
void TW_CALL get_control_type(void * v, void * /*client_data*/)
{
  Uniped::ControlType & c = *static_cast<Uniped::ControlType *>(v);
  c = uniped.control_type;
}

void set_mode(const ModeType new_mode)
{
  using namespace Eigen;
  using namespace std;
#ifndef NDEBUG
  ModeType old_mode = mode;
#endif
  mode = new_mode;
  switch(mode)
  {
    case MODE_TYPE_BEFORE_SIMON:
    {
      bool T = false;
      set_is_animation_recording(&T,&uniped);
      // Immediately fall through
      set_mode(MODE_TYPE_SIMON);
      break;
    }
    case MODE_TYPE_SIMON:
    {
      assert(old_mode == MODE_TYPE_BEFORE_SIMON);
      generate_animCB(&simon_says);
      const Uniped::ControlType new_ct = 
        (uniped.control_type == Uniped::CONTROL_TYPE_PUPPET ? 
           Uniped::CONTROL_TYPE_MOUSE : Uniped::CONTROL_TYPE_PUPPET);
      set_control_type(&new_ct,NULL);
      test_log_animation(simon_says,"target_anim");
      //bool T = true;
      //set_is_animation_playing(&T,&simon_says);
      //fall through instead
      set_mode(MODE_TYPE_BEFORE_RECORDING);
      break;
    }
    case MODE_TYPE_BEFORE_RECORDING:
      flash_on_next = true;
      samples.clear();
      break;
    case MODE_TYPE_RECORDING:
    {
      assert(is_near_rest);
      assert(old_mode == MODE_TYPE_BEFORE_RECORDING);
      bool T = true;
      set_is_animation_recording(&T,&uniped);
      break;
    }
    default:
      assert(false);
      break;
  }
}

void test_log(const std::string & msg)
{
  if(is_testing)
  {
    lawg<<msg;
  }
}

bool init_puppet()
{
  using namespace Eigen;
  genie_vp.camera.look_at(
    Vector3d(0,-0.5,2.2),
    Vector3d(0,-0.5,0),
    Vector3d(0,1,0));
  pr = new PuppetReader("ignored",115200);
  return pr->is_attached();
}

void push_undo()
{
  undo_stack.push(s);
  // Clear
  redo_stack = std::stack<State>();
}

void reshape(int width, int height)
{
  using namespace std;
  main_vp.width = width;
  main_vp.height = height;
  genie_vp.x = main_vp.x+width-genie_vp.width;
  genie_vp.y = main_vp.y+height-genie_vp.height;
  // Send the new window size to AntTweakBar
  TwWindowSize(width, height);
}

bool save_animation(
  const std::string & anim_filename,
  const Uniped::ThetaAnimationType * anims)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;

  MatrixXd A = animation_matrix(anims);

  if(!writeDMAT(anim_filename,A,false))
  {
    cerr<<REDRUM("Saving to "<<anim_filename<<" failed.")<<endl;
    return false;
  }
  cerr<<GREENGIN("Saved to "<<anim_filename<<" successfully.")<<endl;
  return true;
}

void TW_CALL push_keyframe(void * client_data)
{
  using namespace std;
  Uniped & u = *static_cast< Uniped*>(client_data);
  // Collect next time value (across theta)
  double t = 0;
  for(int th = 0;th<3;th++)
  {
    auto & keyframes = u.theta_animation[th].keyframes;
    if(!keyframes.empty())
    {
      t = max(t,keyframes.back().first + keyframe_step);
    }
  }
  // Push keyframes on all theta at same time
  for(int th = 0;th<3;th++)
  {
    auto & keyframes = u.theta_animation[th].keyframes;
    const double & v = u.theta(th);
    keyframes.push_back(make_pair(t,BezierKeyframe<double>(v,v,v)));
  }
}

void TW_CALL pop_keyframe(void * client_data)
{
  Uniped & u = *static_cast< Uniped*>(client_data);
  // Pop one from each theta. Doesn't really make sense if keyframes didn't
  // come from pushing...
  for(int th = 0;th<3;th++)
  {
    auto & keyframes = u.theta_animation[th].keyframes;
    if(!keyframes.empty())
    {
      keyframes.pop_back();
    }
  }
}

void TW_CALL get_max_keyframes_size(void * v, void * client_data)
{
  using namespace std;
  int & s = *(int*)(v);
  s = -1;
  const Uniped & u = *static_cast<const Uniped*>(client_data);
  for(int th = 0;th<3;th++)
  {
    auto & keyframes = u.theta_animation[th].keyframes;
    s = max(s,(int)keyframes.size());
  }
}

static void push_ease_theta(const Eigen::Vector3d & theta, Uniped & u)
{
  using namespace std;
  for(int th = 0;th<3;th++)
  {
    auto & keyframes = u.theta_animation[th].keyframes;
    double t = 0;
    if(!keyframes.empty())
    {
      t = keyframes.back().first + keyframe_step;
    }
    const double & v = theta(th);
    keyframes.push_back(make_pair(t,BezierKeyframe<double>(v,v,v)));
  }
}

Eigen::Vector3d random_theta(const double dist_away_from_rest)
{
  using namespace Eigen;
  const double & d = dist_away_from_rest;
  // u ∈ [0,1]
  const Vector3d u = Vector3d::Random().array()*0.5+0.5;
  Vector3d w;
  for(int th = 0;th<3;th++)
  {
    const double n = MIN_THETA(th);
    const double x = MAX_THETA(th);
    const double r = REST_THETA(th);
    // v ∈ [n,x-2*d]
    const double v = u(th) * (x - n - 2.*d) + n;
    // w ∈ [n,r-d] ∪ [r+d,x]
    if(v < (r-d))
    {
      w(th) = v;
    }else
    {
      w(th) = v + 2.*d;
    }
    assert(
      (w(th) >= n && w(th) <= (r-d)) ||
      (w(th) >= (r+d) && w(th) <= x));
  }
  return w;
}

void random_simultaneous_animation(Uniped & u, const int num_sim)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  for(auto & anim : u.theta_animation)
  {
    anim.keyframes.clear();
  }
  // degrees from REST_THETA
  Vector3d S = random_theta(10./180.*PI);
  VectorXi I = randperm<VectorXi>(3);
  Vector3d R = S;
  for(int th = 0;th<(3-num_sim);th++)
  {
    R(I(th)) = REST_THETA(I(th));
  }
  push_ease_theta(REST_THETA,u);
  push_ease_theta(REST_THETA,u);
  push_ease_theta(R,u);
  push_ease_theta(R,u);
  push_ease_theta(REST_THETA,u);
}


void random_sequential_animation(Uniped & u)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  for(auto & anim : u.theta_animation)
  {
    anim.keyframes.clear();
  }
  Vector3d S = random_theta(5./180.*PI);
  MatrixXd T(3,3);
  T.rowwise() = REST_THETA.transpose();
  VectorXi order = randperm<VectorXi>(3);
  for(int th = 0; th<3;th++)
  {
    for(int c = order(th);c<3;c++)
    {
      T(c,th) = S(th);
    }
  }
  push_ease_theta(REST_THETA,u);
  push_ease_theta(REST_THETA,u);
  Eigen::Vector3d prev = REST_THETA;
  auto bounce = [](
    const Vector3d & theta,
    Vector3d & prev,
    Uniped & u)
  {
    push_ease_theta(theta,u);
    push_ease_theta(prev,u);
    push_ease_theta(theta,u);
    push_ease_theta(theta,u);
    prev = theta;
  };
  for(int f = 0;f<T.rows();f++)
  {
    // Push twice to pause
    bounce(T.row(f),prev,u);
  }
  for(int f = T.rows()-2;f>=0;f--)
  {
    bounce(T.row(f),prev,u);
  }
  bounce(REST_THETA,prev,u);
}

void next_target_anim(Uniped & u)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  static int i = 0;
  i = (i+1)%target_anim_order.size();
  const int a = target_anim_order(i); 
  assert(a<(int)target_anims.size());
  const auto & target_anim = target_anims[a];
  if(is_testing)
  {
    test_log(STR("target_id,"<<a));
  }
  assert(target_anim.size() == 3);
  for(int th = 0;th<3;th++)
  {
    u.theta_animation[th] = target_anim[th];
  }
}

void TW_CALL generate_animCB(void * client_data)
{
  Uniped & u = *static_cast< Uniped*>(client_data);
  switch(generate_anim_type)
  {
    case GENERATE_ANIMATION_TYPE_SIMULTANEOUS:
      random_simultaneous_animation(u,num_simultaneous);
      break;
    case GENERATE_ANIMATION_TYPE_SEQUENTIAL:
      random_sequential_animation(u);
      break;
    case GENERATE_ANIMATION_TYPE_INPUT:
      if(target_anim_order.size()>0)
      {
        next_target_anim(u);
      }
      break;
    default:
      assert(false && "Unknown GenerateAnimationType");
      break;
  }
}

void TW_CALL save_animationCB(void * client_data)
{
  Uniped & u = *static_cast< Uniped*>(client_data);
  save_animation("leg-anim.dmat",u.theta_animation);
}

void TW_CALL reset_theta(void * /*client_data*/)
{
  uniped.theta = REST_THETA;
}

double get_animation_duration(const Uniped & u)
{
  using namespace std;
  double t = 0;
  for(auto & anim : u.theta_animation)
  {
    if(!anim.keyframes.empty())
    {
      t = max(anim.keyframes.back().first,t);
    }
  }
  return t;
}

void TW_CALL get_is_animation_playing(void * v, void * client_data)
{
  Uniped & u = *static_cast<Uniped *>(client_data);
  bool & b = *(bool *)v;
  b = false;
  for(auto & anim : u.theta_animation)
  {
    b |= anim.is_playing;
  }
}

void TW_CALL set_is_animation_playing(const void * v, void * client_data)
{
  using namespace igl;
  using namespace std;
  Uniped & u = *static_cast<Uniped *>(client_data);
  const bool & b = *(const bool *)v;
  const double t = get_seconds();
  for(auto & anim : u.theta_animation)
  {
    if(anim.is_recording)
    {
      return;
    }
  }
  //test_log("play");
  for(auto & anim : u.theta_animation)
  {
    anim.is_playing =  b;
    // if playing then reset start time
    if(anim.is_playing)
    {
      anim.t_start = t;
      anim.last_interval = 0;
    }
  }
  if(&u != &simon_says)
  {
    set_is_animation_playing(v,&simon_says);
  }
}

void TW_CALL get_is_animation_recording(void * v, void * client_data)
{
  Uniped & u = *static_cast<Uniped *>(client_data);
  bool & b = *(bool *)v;
  b = false;
  for(auto & anim : u.theta_animation)
  {
    b |= anim.is_recording;
  }
}

void TW_CALL set_is_animation_recording(const void * v, void * client_data)
{
  using namespace igl;
  using namespace std;
  Uniped & u = *static_cast<Uniped *>(client_data);
  const bool & b = *(const bool *)v;
  const double t = get_seconds();
  string msg = "";
  for(int th = 0;th<3;th++)
  {
    auto & anim  = u.theta_animation[th];
    // toggle recording or playing
    if(!b)
    {
      anim.is_recording = false;
      anim.is_playing = false;
      continue;
    }
    if(anim.is_listening)
    {
      anim.is_recording = true;
      anim.is_playing = false;
      msg += STR(th<<",");
    }else
    {
      anim.is_recording = false;
      anim.is_playing = true;
    }
    // if recording then reset start time
    if(anim.is_recording)
    {
      anim.keyframes.clear();
    }
    // either anim.is_recording or anim.is_playing is set
    anim.t_start = t;
    anim.last_interval = 0;
  }
  if(!msg.empty())
  {
    //test_log(STR("record,"<<msg));
  }
  if(&u != &simon_says)
  {
    set_is_animation_playing(v,&simon_says);
  }
}

bool init_anttweakbar()
{
  using namespace igl;
  using namespace std;
  if( !TwInit(TW_OPENGL, NULL) )
  {
    // A fatal error occured
    cerr<<"AntTweakBar initialization failed: "<<TwGetLastError()<<endl;
    return false;
  }
  // Create a tweak bar
  rebar.TwNewBar("bar");
  TwDefine("bar label='leg' size='200 550' text=light alpha='200' color='68 68 68'");
  rebar.TwAddVarRW("zoom",TW_TYPE_DOUBLE,&s.zoom," keyIncr='+' keyDecr='-' step=0.1 min=0.1 max=2");
  rebar.TwAddVarRW("pan_x",TW_TYPE_DOUBLE,s.pan.data()+0," step=1 min=-500 max=500");
  rebar.TwAddVarRW("pan_y",TW_TYPE_DOUBLE,s.pan.data()+1," step=1 min=-500 max=500");
  rebar.TwAddVarRW("auto_x",TW_TYPE_DOUBLE,simon_says.pos.data()," step=1 min=-2000 max=2000");
  rebar.TwAddVarRW("near_eps",TW_TYPE_DOUBLE,&NEAR_EPS,"step=0.0001 min=0");
  rebar.TwAddVarRW("num_targets",TW_TYPE_INT32,&g_num_targets,"visible=false");
  for(int th = 0;th<3;th++)
  {
    rebar.TwAddVarRW(
      C_STR("uniped_theta_"<<th),
      TW_TYPE_DOUBLE,
      uniped.theta.data()+th,
      C_STR("keyIncr='"<<th+7<<"' keyDecr='"<<th+4<<
        "' step=0.1 min="<<PUPPET_MIN_THETA(th)<<
        " max="<<PUPPET_MAX_THETA(th)));
  }
  rebar.TwAddButton("reset_theta",reset_theta,NULL,"key=0");
  //rebar.TwAddVarRW("uniped_offsets_0",TW_TYPE_DOUBLE,uniped.offsets.data()+1,"step=0.01",false);
  //rebar.TwAddVarRW("uniped_offsets_1",TW_TYPE_DOUBLE,uniped.offsets.data()+2,"step=0.01",false);
  //rebar.TwAddVarRW("uniped_offsets_2",TW_TYPE_DOUBLE,uniped.offsets.data()+3,"step=0.01",false);
  TwType ControlTypeTW = 
    ReTwDefineEnumFromString("ControlType", "device,keyboard,mouse");
  rebar.TwAddVarCB("control_type",ControlTypeTW,set_control_type,get_control_type,NULL,"key=c");
  rebar.TwAddVarRW("bar_is_visible",TW_TYPE_BOOLCPP,&bar_is_visible,"key=B",false);
  add_puppetreader_anttweakbar(pr,rebar);
  rebar.TwSetParam( "PuppetReader", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
  rebar.TwAddVarCB("simon play",TW_TYPE_BOOLCPP,set_is_animation_playing,get_is_animation_playing,
    &simon_says,"group='Animation' key=s",false);
  rebar.TwAddVarCB("play",TW_TYPE_BOOLCPP,set_is_animation_playing,get_is_animation_playing,
    &uniped,"group='Animation' key=p ",false);
  rebar.TwAddVarCB("record",TW_TYPE_BOOLCPP,set_is_animation_recording,get_is_animation_recording,
    &uniped,"group='Animation' key=r",false);
  rebar.TwAddButton("push_keyframe",push_keyframe,&uniped,"group='Animation' key='A'");
  rebar.TwAddButton("pop_keyframe",pop_keyframe,&uniped,"group='Animation' key='Q'");
  rebar.TwAddVarRW("keyframe_step",TW_TYPE_DOUBLE,&keyframe_step,"group='Animation' min=0 max=10 step=0.1");
  rebar.TwAddVarCB("keyframe_stack_size",TW_TYPE_INT32,
    no_op,get_max_keyframes_size,&uniped,
    "group='Animation' readonly=true",false);
  for(int th = 0;th<3;th++)
  {
    rebar.TwAddVarRW(
      C_STR("is_listening_"<<th),
      TW_TYPE_BOOLCPP,
      &uniped.theta_animation[th].is_listening,
      C_STR("label='is_listening["<<th<<"]' group='Animation' "));
  }
  rebar.TwAddButton("save_animation",save_animationCB,&uniped,
    "label=save key=S group=Animation help='save animation to file'");
  rebar.TwAddVarRO("fps",TW_TYPE_DOUBLE, &fps,
    " group='Info' label='Frames per second'"
    " help='Displays current number of frames drawn per second.'");
  rebar.TwAddVarRO("mspf",TW_TYPE_DOUBLE, &mspf,
    " group='Info' label='ms per frame'");
  TwType GenerateAnimationTypeTW = 
    ReTwDefineEnumFromString("GenerateAnimationType", "sequential,simultaneous,input");
  rebar.TwAddVarRW("generate_anim_type",
    GenerateAnimationTypeTW,&generate_anim_type,"group='Animation' ");
  rebar.TwAddVarRW("num_simultaneous",TW_TYPE_INT32,&num_simultaneous,
    "group='Animation' min=1 max=3");
  rebar.TwAddButton("generate_anim",generate_animCB,&simon_says,
    "group='Animation' key=G");
  rebar.TwAddVarCB("is_fullscreen",TW_TYPE_BOOLCPP,
    set_is_fullscreen,get_is_fullscreen,&is_fullscreen,"key=f");
  rebar.load(rebar_in_name.c_str());
  return true;
}

void push_ortho_camera()
{
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(s.pan(0),s.pan(1),0);
  glScaled(s.zoom,s.zoom,s.zoom);
}

void pop_ortho_camera()
{
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void push_ortho_scene(const igl::Viewport & vp)
{
  using namespace igl;
  using namespace std;
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  //const double near = 
  //  s.zoom * 
  //    (min(simon_says.V.col(2).minCoeff(),uniped.V.col(2).minCoeff())-1);
  //const double far = 
  //  s.zoom * 
  //    (max(simon_says.V.col(2).maxCoeff(),uniped.V.col(2).maxCoeff())+1);
  glOrtho(0,vp.width,0,vp.height,-50,50);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
}
void pop_ortho_scene()
{
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

// Set up double-sided lights
void lights()
{
  using namespace std;
  using namespace Eigen;
  glEnable(GL_LIGHTING);
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
  glEnable(GL_LIGHT0);
  float WHITE[4] = {1,1,1,1.};
  //float GREY[4] = {0.4,0.4,0.4,1.};
  float BLACK[4] = {0.,0.,0.,1.};
  //float NEAR_BLACK[4] =  {0.1,0.1,0.1,1.};
  glLightfv(GL_LIGHT0,GL_AMBIENT,BLACK);
  glLightfv(GL_LIGHT0,GL_DIFFUSE,WHITE);
  glLightfv(GL_LIGHT0,GL_SPECULAR,BLACK);
  const Eigen::Vector4f light_pos(-0.1,-0.1,0.9,0);
  glLightfv(GL_LIGHT0,GL_POSITION,light_pos.data());
}

const std::vector<const TBT *> get_const_tbt_chain(const PuppetReader * pr)
{
  using namespace std;
  if(const Node * r = pr->get_const_root())
  {
    vector<const TBT *> chain(3,NULL);
    const TBT * c0= dynamic_cast<const TBT *>(r->get_child(0));
    if(!c0)
    {
      return vector<const TBT *>();
    }
    chain[0] = c0;
    const TBT * c1 = dynamic_cast<const TBT *>(c0->get_child(0));
    if(!c1)
    {
      return vector<const TBT *>();
    }
    chain[1] = c1;
    const TBT * c2 = dynamic_cast<const TBT *>(c1->get_child(0));
    if(!c2)
    {
      return vector<const TBT *>();
    }
    chain[2] = c2;
    return chain;
  }
  return vector<const TBT *>();
}

void sync(PuppetReader * pr, Uniped & u)
{
  using namespace std;
  using namespace Eigen;
  if(pr->is_attached() && 
    (!is_testing || (
      (mode == MODE_TYPE_BEFORE_RECORDING ||
      mode == MODE_TYPE_RECORDING))))
  {
    pr->sync();
    if(Node * r = pr->get_root())
    {
      r->r_off = Quat(0,0,0.707106781,0.707106781);
    }
    const std::vector<const TBT * > chain = get_const_tbt_chain(pr);
    if(chain.size() == 3)
    {
      // Allow puppet to steal attention once
      static bool once = false;
      if(!once && !is_testing)
      {
        cout<<"stealing once..."<<endl;
        u.control_type = Uniped::CONTROL_TYPE_PUPPET;
        once = true;
      }
      if(u.control_type == Uniped::CONTROL_TYPE_PUPPET)
      {
        for(int t = 0; t<3;t++)
        {
          u.theta[t] =  chain[t]->angle[1]/180.*igl::PI;
        }
        if(!is_testing)
        {
          const int n = u.V.rows();
          for(int v = 0;v<n;v++)
          {
            switch(u.G(v))
            {
              case UNIPED_UPPER_LEG:
                u.color(v,0) = chain[0]->color[0];
                u.color(v,1) = chain[0]->color[1];
                u.color(v,2) = chain[0]->color[2];
                break;
              case UNIPED_LOWER_LEG:
                u.color(v,0) = chain[1]->color[0];
                u.color(v,1) = chain[1]->color[1];
                u.color(v,2) = chain[1]->color[2];
                break;
              case UNIPED_FOOT:
                u.color(v,0) = chain[2]->color[0];
                u.color(v,1) = chain[2]->color[1];
                u.color(v,2) = chain[2]->color[2];
                break;
              default:
                u.color.row(v).setConstant(1);
                break;
            }
          }
        }
      }
      return;
    }
    // fall through
  }
  
}

double theta_dist(
  const Eigen::Vector3d & A,
  const Eigen::Vector3d & B)
{
  using namespace igl;
  using namespace Eigen;
  Eigen::Vector3d cA,cB;
  cumsum(A,1,cA);
  cumsum(B,1,cB);
  return (cA-cB).norm();
}

void draw_instructions()
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  // True screen space
  glViewport(0,0,main_vp.width,main_vp.height);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0,main_vp.width,0,main_vp.height);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glDisable(GL_LIGHTING);
  glColor4f(0.2,0.2,0.2,1);
  // bottom left corner
  const float scale = 0.25f;
  glPushMatrix();
  glScalef(scale,scale,scale);
  glTranslatef(0,(119.05-33.33)/2,0);
  glLineWidth(2);
  auto draw_string = [](const string & str)
  {
    for_each(
      str.begin(),
      str.end(),
      bind1st(ptr_fun(&glutStrokeCharacter),GLUT_STROKE_ROMAN));
  };
  auto draw_return_to_rest = [&]()
  {
    const double f = 
      sqrt(theta_dist(uniped.theta,REST_THETA) /
          theta_dist(REST_THETA+Vector3d(PI/2,PI/2,PI/2),REST_THETA));
    glLineWidth(2.+f);
    glColor4d(f,0.2,0.2,2*f+0.2);
    draw_string("Return to rest pose before continuing.");
  };
  if(is_testing)
  {
    switch(mode)
    {
      case MODE_TYPE_BEFORE_SIMON:
        draw_string(STR("[space]: watch target animation"));
        break;
      case MODE_TYPE_SIMON:
        draw_string(STR(
          "... you will be asked to repeat this animation ..."));
        break;
      case MODE_TYPE_BEFORE_RECORDING:
        if(is_near_rest)
        {
          draw_string(STR("[space]: repeat animation with the "<<
            control_type_as_string(uniped.control_type)));
        }else
        {
          draw_return_to_rest();
        }
        break;
      case MODE_TYPE_RECORDING:
        draw_string(STR("... match target animation using the "<<
          control_type_as_string(uniped.control_type)<<"..."));
        break;
      default:
        assert(false);
        break;
    }
    glPopMatrix();
    glPushMatrix();
    glColor4f(0.8,0.2,0.6,1);
    glTranslatef(0,main_vp.height,0);
    glScalef(scale,scale,scale);
    glTranslatef(0,-(119.05-33.33)*1.5,0);
    string progress = STR((target+1)<<"/"<<num_targets()<<" "<<
      control_type_as_string(uniped.control_type));
    draw_string(progress);
  }else
  {
    bool u_is_recording=false; 
    get_is_animation_recording(&u_is_recording,&uniped);
    bool u_is_playing=false; 
    get_is_animation_playing(&u_is_playing,&uniped);
    bool s_is_playing=false; 
    get_is_animation_playing(&s_is_playing,&simon_says);

    if(u_is_recording)
    {
      draw_string("[r]: stop recording.");
    }else if(u_is_playing)
    {
      draw_string("[p]: stop playing.");
    }else if(s_is_playing)
    {
      draw_string("[s]: stop playing.");
    }else if(!u_is_recording && !u_is_playing && !s_is_playing)
    {
      if(is_near_rest)
      {
        draw_string(
          "[r]: start recording, "
          "[p]: playback recording & target, "
          "[s]: playback just target.");
      }else
      {
        draw_return_to_rest();
      }
    }
  }
  glPopMatrix();
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

//const Eigen::Vector4f back(0.8,0.8,0.8,0);
//const Eigen::Vector4f back(0.0,0.0,0.0,0);
const Eigen::Vector4f back(1.0,1.0,1.0,0);
void display()
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;

  glClearColor(back(0),back(1),back(2),back(3));
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 

  if(is_testing)
  {
    bool u_is_recording=false; 
    get_is_animation_recording(&u_is_recording,&uniped);
    bool s_is_playing=false; 
    get_is_animation_playing(&s_is_playing,&simon_says);
    switch(mode)
    {
      case MODE_TYPE_SIMON:
        if(!s_is_playing)
        {
          set_mode(MODE_TYPE_BEFORE_RECORDING);
        }
        break;
      case MODE_TYPE_RECORDING:
        if(!s_is_playing)
        {
          assert(u_is_recording);
          target++;
          //test_log_animation(uniped,"recording");
          RowVectorXd mS;
          list_to_matrix(samples,mS);
          test_log(STR("samples,"<<
            mS.format(IOFormat(FullPrecision,DontAlignCols))));
          if(target>=num_targets())
          {
            test_log("finish");
            if(lawg.save(log_filename))
            {
              cout<<GREENGIN("Saved log to "<<log_filename<<".")<<endl;
            }else
            {
              cout<<REDRUM("Saving log to "<<log_filename<<" failed.")<<endl;
            }
            exit(0);
          }
          set_mode(MODE_TYPE_BEFORE_SIMON);
        }
        break;
      default:
        assert(!u_is_recording);
        assert(!s_is_playing);
        break;
    }
  }

  sync(pr,uniped);

  uniped.update();
  is_near_rest = theta_dist(uniped.theta,REST_THETA) < NEAR_EPS;
  simon_says.update();
  if((uniped.theta-uniped.prev_theta).norm() > 5e-3)
  {
    //test_log(STR("theta,"<<
    //  uniped.theta(0)<<","<<
    //  uniped.theta(1)<<","<<
    //  uniped.theta(2)<<","));
    uniped.prev_theta = uniped.theta;
  }
  if((simon_says.theta-simon_says.prev_theta).norm() > 5e-3)
  {
    //test_log(STR("target,"<<
    //  simon_says.theta(0)<<","<<
    //  simon_says.theta(1)<<","<<
    //  simon_says.theta(2)<<","));
    simon_says.prev_theta = simon_says.theta;
  }
  if(is_testing && mode == MODE_TYPE_RECORDING)
  {
    // sample every 33 ms --> 30 fps
    if((get_seconds()-last_sample_time)>0.033)
    {
      last_sample_time = append_samples();
    }
  }


  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);

  glViewport(main_vp.x, main_vp.y, main_vp.width, main_vp.height);
  push_ortho_scene(main_vp);
  push_ortho_camera();

  if(!is_testing || (mode != MODE_TYPE_BEFORE_SIMON && mode != MODE_TYPE_SIMON))
  {
    uniped.draw();
  }
  if(!is_testing)
  {
    bool u_is_recording=false; 
    get_is_animation_recording(&u_is_recording,&uniped);
    bool u_is_playing=false; 
    get_is_animation_playing(&u_is_playing,&uniped);
    bool s_is_playing=false; 
    get_is_animation_playing(&s_is_playing,&simon_says);
    if(is_shadow_visible  && !u_is_playing && !u_is_recording && !is_near_rest)
    {
      shadow.pos = uniped.pos;
      shadow.draw();
    }
  }
  simon_says.draw();
  if(is_drawing_progress_bar)
  {
    uniped.draw_progress_bar();
    simon_says.draw_progress_bar();
  }

  pop_ortho_camera();
  if(is_dragging && selected_theta >= 0)
  {
    draw_guide_circle(
      p_center,m_down,m_drag,
      50, 
      PUPPET_MIN_THETA(selected_theta),
      PUPPET_MAX_THETA(selected_theta));
  }
  pop_ortho_scene();
  
  lights();
  {
    Vector4f bg_color(1,0,0,1);
    if(get_const_tbt_chain(pr).size() == 3)
    {
      bg_color = Vector4f(0.6,0.6,0.6,1);
    }
    if(uniped.control_type == Uniped::CONTROL_TYPE_PUPPET)
    {
      draw_genie(genie_vp,pr->get_const_root(),bg_color);
    }
  }

  draw_instructions();

  report_gl_error();

  if(render_to_png_on_next)
  {
    static int render_count = 0;
    stringstream padnum; 
    padnum << getenv("HOME")<<"/Desktop/"<< "puppet-" << setw(4) << setfill('0') << render_count++ << ".png";
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    render_to_png_async(padnum.str(),viewport[2],viewport[3],true,false);
    glClearColor(1,1,1,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glFlush();
    usleep(1000*1);
    render_to_png_on_next = false;
  }

  if(bar_is_visible)
  {
    TwDraw();
  }

  if(flash_on_next)
  {
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glFlush();
    usleep(1000*1);
    flash_on_next = false;
  }

  glutSwapBuffers();
  if(pr->is_attached() || is_testing)
  {
    glutPostRedisplay();
  }
}

void mouse_wheel(int wheel, int direction, int mouse_x, int mouse_y)
{
  using namespace std;
  using namespace Eigen;
  // Ignore if testing
  if(is_testing)
  {
    return;
  }
  // mimic photoshop's pan and zoom controls
  int mod = glutGetModifiers();
  if(mod & GLUT_ACTIVE_ALT)
  {
    if(wheel == 0)
    {
      double old_zoom = s.zoom;
      // absolute scale difference when changing zooms (+1)
      const double z_diff = 0.01;
      Vector2d m(mouse_x,main_vp.height - mouse_y);
      Vector2d d = m - s.pan;
      if(!is_dragging)
      {
        s.zoom *= (1.0+double(direction)*z_diff);
        const double min_zoom = 0.01;
        const double max_zoom = 10.0;
        s.zoom = min(max_zoom,max(min_zoom,s.zoom));
      }
      s.pan += m - (s.pan + s.zoom/old_zoom*d);
    }//else horizontal scroll
  }else if(!is_dragging)
  {
    if(wheel==0 && bar_is_visible && TwMouseMotion(mouse_x, mouse_y))
    {
      static double mouse_scroll_y = 0;
      const double delta_y = 0.125*direction;
      mouse_scroll_y += delta_y;
      TwMouseWheel(mouse_scroll_y);
    }else
    {
      if(wheel==0)
      {
        s.pan(1) += (-5*direction)*s.zoom;
      }else
      { 
        s.pan(0) += (-5*direction)*s.zoom;
      }
      s.pan(0) = std::min(std::max(s.pan(0),-200.),(double)main_vp.width-200);
      s.pan(1) = std::min(std::max(s.pan(1),-200.),(double)main_vp.height-200);
    }
  }
  glutPostRedisplay();
}

void mouse(int glutButton, int glutState, int mouse_x, int mouse_y)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  bool tw_using = TwEventMouseButtonGLUT(glutButton,glutState,mouse_x,mouse_y);
  is_dragging = false;
  switch(glutButton)
  {
    case GLUT_RIGHT_BUTTON:
    case GLUT_LEFT_BUTTON:
    {
      switch(glutState)
      {
        case 1:
          selected_theta = -1;
          switch(uniped.control_type)
          {
            case Uniped::CONTROL_TYPE_MOUSE:
              uniped.color.leftCols(3).setConstant(1);
              break;
            default:
              break;
          }
          // up
          break;
        case 0:
          if(!tw_using && uniped.control_type == Uniped::CONTROL_TYPE_MOUSE)
          {
            // Needs to be row vector for snap_points to work correctly
            VectorXd _;
            VectorXi I;
            glViewport(main_vp.x, main_vp.y, main_vp.width, main_vp.height);
            push_ortho_scene(main_vp);
            push_ortho_camera();
            m_down = m_drag = Vector2d(mouse_x,main_vp.height-mouse_y);
            Vector3d p_down;
            unproject_to_zero_plane(m_down,p_down);
            snap_points(p_down.transpose().eval(),uniped.U,I,_);
            int g = -1;
            switch(uniped.G(I(0)))
            {
              case UNIPED_BODY:
              case UNIPED_UPPER_LEG:
                selected_theta = 0;
                g = 1;
                break;
              case UNIPED_KNEE:
              case UNIPED_LOWER_LEG:
                selected_theta = 1;
                g = 3;
                break;
              case UNIPED_ANKLE:
              case UNIPED_FOOT:
                selected_theta = 2;
                g = 5;
                break;
            }
            is_dragging = true;
            if(selected_theta >= 0 && is_dragging)
            {
              {
                Vector3d center(0,0,0);
                center.head(2) = uniped.C.row(uniped.BE(selected_theta+1,0)).transpose();
                // Rigidly transform by anscestors
                Vector3d theta = uniped.theta;
                theta += uniped.offsets.tail(3);
                Affine3d t = rigid2d(uniped.C.row(uniped.BE(0,0)),uniped.offsets(0));
                for(int th = 0; th<selected_theta;th++)
                {
                  t = t * rigid2d(uniped.C.row(uniped.BE(th+1,0)),theta(th));
                }
                center = t*center;
                Vector3d p_center3;
                project(center,p_center3);
                p_center = p_center3.head(2);
              }
              down_theta = uniped.theta(selected_theta);
              for(int v = 0;v<uniped.color.rows();v++)
              {
                if(uniped.G(v)%2==1 && uniped.G(v) >= g)
                {
                  uniped.color.row(v).head(3) = Vector3d(0.7,0.8,1);
                }else
                {
                  uniped.color.row(v).head(3).setConstant(1);
                }
              }
            }
            pop_ortho_camera();
            pop_ortho_scene();
          }
        break;
      }
      break;
      // Scroll down
      case GLUT_WHEEL_DOWN:
      {
        mouse_wheel(0,-1,mouse_x,mouse_y);
        break;
      }
      // Scroll up
      case GLUT_WHEEL_UP:
      {
        mouse_wheel(0,1,mouse_x,mouse_y);
        break;
      }
      // Scroll left
      case GLUT_WHEEL_LEFT:
      {
        mouse_wheel(1,-1,mouse_x,mouse_y);
        break;
      }
      // Scroll right
      case GLUT_WHEEL_RIGHT:
      {
        mouse_wheel(1,1,mouse_x,mouse_y);
        break;
      }
    }
  }
  glutPostRedisplay();
}

void mouse_move(int mouse_x, int mouse_y)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  if(bar_is_visible)
  {
    bool tw_using = TwMouseMotion(mouse_x,mouse_y);
    if(tw_using)
    {
      return;
    }
  }
  glutPostRedisplay();
}

void mouse_drag(int mouse_x, int mouse_y)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  if(is_dragging)
  {
    if(selected_theta >= 0)
    {
      m_drag = Vector2d(mouse_x,main_vp.height-mouse_y);
      const Vector2d B = Vector2d(m_drag(0)-p_center(0),m_drag(1)-p_center(1));
      const Vector2d A = Vector2d(m_down(0)-p_center(0),m_down(1)-p_center(1));
      const double dtheta = atan2(A(0)*B(1)-A(1)*B(0),A(0)*B(0)+A(1)*B(1));
      uniped.theta(selected_theta) = 
        max(min(down_theta + dtheta,
          PUPPET_MAX_THETA(selected_theta)),
          PUPPET_MIN_THETA(selected_theta));
    }
  }else
  {
    if(bar_is_visible)
    {
      /*bool tw_using = */TwMouseMotion(mouse_x,mouse_y);
    }
  }
  glutPostRedisplay();
}

void init_uniped(Uniped & uniped)
{
  using namespace Eigen;
  using namespace igl;
  using namespace std;
  const string
    uniped_path = "data/uniped/unipedli";
  YImage yimg;
#ifndef NDEBUG
  const bool success = 
#endif
    readDMAT(uniped_path+".V.dmat",uniped.V) && 
    readDMAT(uniped_path+".UV.dmat",uniped.UV) && 
    readDMAT(uniped_path+".F.dmat",uniped.F) && 
    readDMAT(uniped_path+".G.dmat",uniped.G) && 
    readDMAT(uniped_path+".C.dmat",uniped.C) && 
    readDMAT(uniped_path+".BE.dmat",uniped.BE) && 
    readDMAT(uniped_path+".offsets.dmat",uniped.offsets) && 
    //yimg.load((uniped_path + ".png").c_str());
    yimg.load((uniped_path + ".png").c_str());
  assert(success);

  if(uniped.tex != 0)
  {
    glDeleteTextures(1,&uniped.tex);
  }
  glGenTextures(1, &uniped.tex);
  glBindTexture(GL_TEXTURE_2D, uniped.tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexImage2D(
    GL_TEXTURE_2D, 0, GL_RGBA, 
    yimg.width(), yimg.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, yimg.data());
  glBindTexture(GL_TEXTURE_2D, 0);
  uniped.U = uniped.V;
  uniped.V.col(2) /= uniped.V.col(2).maxCoeff();
  uniped.color = MatrixXd::Ones(uniped.V.rows(),4);
}

void undo()
{
  using namespace std;
  if(!undo_stack.empty())
  {
    redo_stack.push(s);
    s = undo_stack.top();
    undo_stack.pop();
  }
}

void redo()
{
  using namespace std;
  if(!redo_stack.empty())
  {
    undo_stack.push(s);
    s = redo_stack.top();
    redo_stack.pop();
  }
}

void key(unsigned char key, int mouse_x, int mouse_y)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  int mod = glutGetModifiers();
  switch(key)
  {
    case ' ':
    {
      if(is_testing &&
        (mode == MODE_TYPE_BEFORE_SIMON || 
        (mode == MODE_TYPE_BEFORE_RECORDING && is_near_rest)))
      {
        set_mode((ModeType)(((int)mode + 1)%NUM_MODE_TYPES));
      }else
      {
        render_to_png_on_next = true;
      }
      break;
    }
    // ESC
    case char(27):
      rebar.save(rebar_out_name.c_str());
    //// ^C
    //case char(3):
    {
      string path = path_to_executable();
      if(path.find(BINARY_BACKUP_DIR) == string::npos)
      {
        dated_copy(path_to_executable(),BINARY_BACKUP_DIR);
      }
      exit(0);
    }
    case 'z':
    case 'Z':
      if(mod & GLUT_ACTIVE_COMMAND)
      {
        if(mod & GLUT_ACTIVE_SHIFT)
        {
          redo();
        }else
        {
          undo();
        }
      }else
      {
        push_undo();
        s.pan = Vector2d(0,0);
        s.zoom = 1;
      }
      break;
    default:
      // Even if bar is hidden, process keys
      if(!TwEventKeyboardGLUT(key,mouse_x,mouse_y))
      {
        cout<<"Unknown key command: "<<key<<" "<<int(key)<<endl;
      }
  }
  glutPostRedisplay();
}

bool load_animation(
  const std::string & anim_filename,
  Uniped::ThetaAnimationType * anims)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  MatrixXd A;
  if(!readDMAT(anim_filename,A))
  {
    cerr<<REDRUM("Reading "<<anim_filename<<" failed.")<<endl;
    return false;
  }
  cerr<<GREENGIN("Read "<<anim_filename<<" successfully.")<<endl;
  assert(A.cols() == 5);
  anims[0].keyframes.reserve(A.rows()/3);
  anims[1].keyframes.reserve(A.rows()/3);
  anims[2].keyframes.reserve(A.rows()/3);
  for(int a = 0;a<A.rows();a++)
  {
    const int th = A(a,0);
    assert(th >= 0 && th < 3 && "ID outside [0,2]");
#ifndef NDEBUG
    // http://stackoverflow.com/a/1521682/148668O
    double intpart;
    assert(modf(A(a,0), &intpart) == 0.0 && "ID should be int");
#endif
    anims[th].keyframes.push_back(
      make_pair(A(a,1),BezierKeyframe<double>(A(a,2),A(a,3),A(a,4))));
  }
  return true;
}

bool load_animations(
  const std::string & anim_prefix,
  std::vector< std::vector<Uniped::ThetaAnimationType> > & anims)
{
  using namespace std;
  using namespace igl;
  anims.clear();
  string prefix = filter_prefix(anim_prefix);
  int a = 0;
  while(true)
  {
    string filename = STR(prefix<<"-"<<a<<".dmat");
    if(!is_file(filename.c_str()))
    {
      break;
    }
    anims.push_back(std::vector<Uniped::ThetaAnimationType>(3));
    load_animation(filename,&anims.back()[0]);
    a++;
  }
  return anims.size() > 0;
}
   

bool parse_arg(int & argc, char * argv[])
{
  using namespace std;
  namespace po = boost::program_options;
  po::options_description desc("Options");
  try
  {
    desc.add_options()
      ("help,h","produce this help message")
      ("log,l",
        po::value<string>(&log_filename)->implicit_value(""),
        "path to log file")
      ("testing,t",
        po::value<string>(&rebar_in_name)->implicit_value(""),
        "run in testing mode, followed by path to .rbr preferences file")
      ("anim,a",
        po::value<string>(&anim_prefix)->implicit_value(""),
        "path to animation file(s)");
    po::options_description all("Allowed options");
    all.add(desc);
    
    po::variables_map vm;
    po::parsed_options parsed =
      po::command_line_parser(argc, argv)
        .options(all)
        .allow_unregistered()
        .run();
    po::store(parsed,vm);
    po::notify(vm);
    vector<string> unknown = po::collect_unrecognized(parsed.options, po::include_positional);
    for(auto & u : unknown)
    {
      cout<<"Ignoring unknown option: "<<u<<endl;
    }
    if(vm.count("help"))
    {
      cout << "Usage:"<<endl<<"  ./leg [options]"<<endl<<endl;
      cout << desc << endl;
      return false;
    }
    if(vm.count("logfile"))
    {
      if(vm["log"].as<string>().empty())
      {
        cerr<<"-l,--log requires argument"<<endl;
        exit(1);
      }
    }
    if(vm.count("anim"))
    {
      cout<<"vm.count(\"testing\")"<<endl;
      if(vm["anim"].as<string>().empty())
      {
        cerr<<"-a,--anim requires argument"<<endl;
        exit(1);
      }
    }
    if(vm.count("testing"))
    {
      is_testing = true;
      if(vm["testing"].as<string>().empty())
      {
        cerr<<"-t,--test requires argument"<<endl;
        exit(1);
      }
    }
  } catch( const exception& e)
  {
    cerr << e.what() << endl << endl;
    cout << desc << endl;
    return false;
  }
  return true;
}

int main(int argc, char * argv[])
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  if(!parse_arg(argc,argv))
  {
    return 1;
  }

  if(is_testing)
  {
    bar_is_visible = false;
  }

  // print key commands
  cout<<"[Click] and [drag]  Rotate model using trackball."<<endl;
  cout<<"[Command+Z]         Undo."<<endl;
  cout<<"[Shift+Command+Z]   Redo."<<endl;
  cout<<"[^C,ESC]            Exit."<<endl;
  cout<<"[j,k]               Lower and raise upper leg at hip."<<endl;
  cout<<"[k,o]               Lower and raise lower leg at knee."<<endl;
  cout<<"[l,p]               Lower and raise foot at ankle."<<endl;

  // Init glut
  glutInit(&argc,argv);

  // Init antweakbar
#ifdef __APPLE__
  glutInitDisplayString( "rgba depth double samples>=8");
#else
  glutInitDisplayString( "rgba depth double ");   // samples>=8 somehow not supported on Kenshi's machines...?
#endif
  //glutInitWindowSize(glutGet(GLUT_SCREEN_WIDTH),glutGet(GLUT_SCREEN_HEIGHT));
  if(is_testing)
  {
    glutInitWindowSize(glutGet(GLUT_SCREEN_WIDTH),glutGet(GLUT_SCREEN_HEIGHT));
  }else
  {
    glutInitWindowSize(1440,720);
  }
  glutCreateWindow(C_STR("leg- ("<<rebar_in_name<<")"));
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(key);
  glutMouseFunc(mouse);
  glutMotionFunc(mouse_drag);
  glutPassiveMotionFunc(mouse_move);

  // After window is created
  srand((unsigned)time(NULL));
  init_uniped(uniped);
  shadow = uniped;
  shadow.color *= 0.5;
  shadow.U.col(2).setConstant(-0.1);
  // Need to call once.
  shadow.theta = REST_THETA;
  shadow.update();

  simon_says = uniped;
  simon_says.control_type = Uniped::CONTROL_TYPE_AUTO;
  // Darken
  simon_says.color.rowwise() = RowVector4d(1,0.5,0.5,0.5);
  // Push over
  simon_says.pos(0) = -1000;
  // Push back
  simon_says.V.col(2).array() *= 10;
  simon_says.V.col(2).array() -= 0.0001;
    //(1+(simon_says.V.col(2).maxCoeff() - simon_says.V.col(2).minCoeff()));
  simon_says.theta = REST_THETA;
  auto & anims = simon_says.theta_animation;
  if(anim_prefix == "")
  {
    random_sequential_animation(simon_says);
  }else
  {
    if(is_testing)
    {
      load_animations(anim_prefix,target_anims);
      assert(target_anims.size() > 0);
      const int half = target_anims.size()/2;
      target_anim_order.resize(target_anims.size()*2);
      // Randomize order of target anims
      VectorXi R = randperm<VectorXi>(target_anims.size());
      for(int a = 0;a<(int)target_anims.size();a++)
      {
        target_anim_order(a*2+0) = R(a);
        // offset puppet by half
        target_anim_order(a*2+1) = R((a+half)%target_anims.size());
      }
      cout<<matlab_format(target_anim_order,"target_anim_order")<<endl;
    }else
    {
      load_animation(anim_prefix,anims);
    }
  }
  //if(!is_testing)
  //{
  //  bool T = true;
  //  set_is_animation_playing(&T,&simon_says);
  //}


  init_puppet();
  init_anttweakbar();
  if(is_testing)
  {
    // Reset log
    lawg = Log<std::string>();

    test_log(STR("control_type,"<<
      control_type_as_string(uniped.control_type)));
    set_mode(MODE_TYPE_BEFORE_SIMON);
  }

  if(!is_testing)
  {
    static std::function<void(int)> timer_bounce;
    auto timer = [] (int ms) {
      timer_bounce(ms);
    };
    timer_bounce = [&] (int ms) {
      glutTimerFunc(ms, timer, ms);
      glutPostRedisplay();
    };
    const int ms = is_testing?30:50;
    glutTimerFunc(ms, timer, ms);
  }

  glutMainLoop();

  return 0;
}
