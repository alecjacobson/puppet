#ifndef RIGID2D_H
#define RIGID2D_H
#include "EigenConvenience.h"
// Rigid motion given center of rotation and angle.
//
// Inputs:
//    center  2d position of center
//    angle  in radians
// Returns 3d affine transformation encoding 2d rigid motion
Eigen::Affine3d rigid2d(
  const Eigen::Vector2d & center,
  const double angle);
#endif
