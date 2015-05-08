#ifndef ANIMATION_H
#define ANIMATION_H
#include <vector>
#include <iostream>
#include <cassert>

template <typename T>
struct Animation
{
  std::vector< std::pair<double,T > > keyframes;
  bool is_playing, is_recording, is_listening;
  double t_start,t_pause;
  int last_interval;
  Animation():
    keyframes(),
    is_playing(false), is_recording(false), is_listening(true),
    t_start(-1),t_pause(-1),last_interval(-1)
  {}
  // Should implement:
  //   virtual const & T at(const int i);
  // Find bounding keyframes for a given time. Expects that t_start and
  // last_interval have been set accordingly.
  //
  // Inputs:
  //   t_abs  absolute time (will be taken relative to t_start/t_pause)
  // Outputs:
  //   left  index of keyframe to the left of t_abs
  //   right  index of keyframe to the right of t_abs
  //   f   (t_rel-t_left)/(t_right-t_left)
  // Returns true iff t_rel is less than last keyframe's time.
  bool find(
   const double t_abs,
   int & left,
   int & right,
   double & f) const
  {
    using namespace std;
      const double t = t_abs - t_start;
      left = right = last_interval;
      for(int i = last_interval;i<(int)keyframes.size();i++)
      {
        if(keyframes[i].first > t)
        {
          break;
        }
        left = i;
        right = i+1;
      }
      bool ret;
      if(right == (int)keyframes.size())
      {
        assert(t > keyframes.back().first);
        right = left;
        f = 0;
        ret = false;
      }else if(right == 0)
      {
        ret = true;
        f=1;
      }else
      {
        ret = true;
        const double t_left = keyframes[left].first;
        const double t_right = keyframes[right].first;
        f = (t-t_left)/(t_right-t_left);
      }
      assert(left >= 0 && left < (int)keyframes.size());
      assert(right >= 0 && right < (int)keyframes.size());
      return ret;
  }
};

#endif
