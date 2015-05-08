#ifndef AUGVIEWPORT_H
#define AUGVIEWPORT_H
#include "EigenConvenience.h"
#include <igl/Viewport.h>
#include <igl/Camera.h>
#include <iostream>
class AugViewport : public igl::Viewport 
{
  public:
    igl::Camera camera;
    bool rotation_is_animating;
    double animation_start_time;
    Eigen::Quaterniond animation_from_quat;
    Eigen::Quaterniond animation_to_quat;
    AugViewport(
      const int _x=0, 
      const int _y=0, 
      const int _width=0,
      const int _height=0);
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
