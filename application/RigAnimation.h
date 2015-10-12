#ifndef RIGANIMATION_H
#define RIGANIMATION_H
#include "BezierAnimation.h"
#include <igl/anttweakbar/ReAntTweakBar.h>
#include <Eigen/Geometry>
#include <vector>

class RigAnimation
{
public:
  typedef Eigen::VectorXi VectorXb;
private:
  // Returns whether A and B are similar
  static bool similar(
      const Eigen::Quaterniond & A,
      const Eigen::Quaterniond & B);
  typedef BezierAnimation<Eigen::Quaterniond> BoneAnimation;
  typedef std::vector<
    Eigen::Quaterniond,
    Eigen::aligned_allocator<Eigen::Quaterniond> > RotationList;
  // m_bone_animations  list of animations for each bone
  // m_pose_at_start_playing  Remembered pose at start of recording
  // m_keyframe_step  Length of a new keyframe from push/pop in seconds
  // m_is_playing_to_from_identity  Whether currently playing from
  //   start_to_from_identity and should pop 
  std::vector<BoneAnimation> m_bone_animations;
  RotationList m_pose_at_start_playing;
  double m_keyframe_step;
  bool m_is_playing_to_from_identity;
  double m_t_start;
public:
  RigAnimation();
  const std::vector<BoneAnimation>& bone_animations() const
    {return m_bone_animations;}
  // Returns true if any m_bone_animations are playing
  bool is_listening() const;
  bool is_recording() const;
  bool is_playing() const;
  // Record the given pose to all bone animations who are recording.
  //
  // Inputs:
  //   pose  list of relative angles corresponding to each bone animation
  // Returns whether anything is recording
  bool record_pose(const RotationList & pose);
  // Clear the current animation
  //
  // Inputs:
  //   n  number of bones
  void clear(const int n);
  // n = m_bone_animations.size()
  void clear();
  // Set is_listening to v for all bone animations
  //
  // Inputs:
  //   v  whether all should be listening
  void set_all_is_recording(const bool v);
  void set_all_is_listening(const bool v);
  // Set all those listening to recording
  //
  // Inputs:
  //   pose  current pose (so that it can be restored at finish)
  void start_recording(const RotationList & pose);
  // Input/Output:
  //   pose  pose to play *into* and record *from*
  bool update(const RotationList & pose);
  // Inputs:
  //   t_abs  absolute time (after call to start playing)
  //   pose  pose to play *into*
  // Returns whether anything is playing.
  bool play(const double t_abs, RotationList & pose);
  // Using current time 
  //
  // Inputs:
  //   pose  pose to play *into*
  // Returns whether anything is playing.
  bool play(RotationList & pose);
  // Add menu items to ReAntTweakBar
  //
  // Inputs:
  //   rebar  ReAntTweakBar instance
  void add_to_reanttweakbar(igl::anttweakbar::ReTwBar & rebar);
  // Start playing current animation from beginning.
  //
  // Inputs:
  //   pose  current pose (so that it can be restored at finish)
  void start_playing(const RotationList & pose);
  // Push entire pose onto keyframes (one to each)
  //
  // Inputs:
  //   pose  current pose to be pushed
  void push_pose(const RotationList & pose);
  // Pop entire pose from keyframes (one from each)
  //
  // Returns true only if something was really popped
  bool pop_pose();
  void start_to_from_identity(const RotationList & pose);
  int keyframes_size() const;
  // Set whether bones are listening
  //
  // Inputs:
  //   S  #BE list of whether to be listening
  void set_is_listening( const VectorXb & S);
  double t_start() const;
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
