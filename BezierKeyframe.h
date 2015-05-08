#ifndef BEZIERKEYFRAME_H
#define BEZIERKEYFRAME_H
#include <vector>
#include "Animation.h"
#include "bezier.h"

// Store values and tangent knots for a keyframe.
//
// Examples:
//     // Push linear keyframes
//     double prev = v;
//     if(!linear.keyframes.empty())
//     {
//       prev = linear.keyframes.back().second.value;
//       linear.keyframes.back().second.next = prev + (v-prev)/3.;
//     }
//     linear.keyframes.push_back(make_pair(t,BezierKeyframe<double>(prev+(v-prev)*2./3.,v,v)));
//     // Push eased keyframes
//     ease.keyframes.push_back(make_pair(t,BezierKeyframe<double>(v,v,v)));
//
template <typename T>
class BezierKeyframe
{
  public:
  typedef T (*LerpType)(const double, const T &,const T&);
  // Helper function. Find a keyframe in an animation and output value.
  // Expects that anim.t_start and anim.last_interval are set accordingly.
  // 
  // Inputs:
  //   anim  animation
  //   t_abs  time to search
  // Outputs:
  //   val  value at time t_abs
  //   left  index of keyframe on the left (useful to pass reference to
  //     anim.last_interval here)
  // Returns true iff t_abs < last time in animation. 
  static bool find(
    const Animation<BezierKeyframe<T> > & anim,
    const double t_abs,
    T & val,
    int & left,
    LerpType lerp);
  // Default lerp
  static bool find(
    const Animation<BezierKeyframe<T> > & anim,
    const double t_abs,
    T & val,
    int & left);
  T prev;
  T value;
  T next;
  BezierKeyframe(
    const T & _prev=T(),
    const T & _value=T(),
    const T & _next=T()):
    prev(_prev),
    value(_value),
    next(_next)
    {
    }
};

// Implementation
#include <iostream>
template <typename T>
bool BezierKeyframe<T>::find(
  const Animation<BezierKeyframe<T> > & anim,
  const double t_abs,
  T & val,
  int & left)
{
  const auto & lerp = [](const double t, const T & A, const T & B) -> T
    {
      return A + t*(B-A);
    };
  return find(anim,t_abs,val,left,lerp);
}
template <typename T>
bool BezierKeyframe<T>::find(
  const Animation<BezierKeyframe<T> > & anim,
  const double t_abs,
  T & val,
  int & left,
  LerpType lerp)
{
  using namespace std;
  int right;
  double f;
  bool ret = anim.find(t_abs,left,right,f);
  vector<T> C(4);
  C[0] = anim.keyframes[left].second.value;
  C[1] = anim.keyframes[left].second.next;
  C[2] = anim.keyframes[right].second.prev;
  C[3] = anim.keyframes[right].second.value;
  val =  bezier<T>(f,C.begin(),C.end(),lerp);
  return ret;
}
#endif
