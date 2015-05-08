#ifndef DRAW_GUIDE_CIRCLE_H
#define DRAW_GUIDE_CIRCLE_H
#include "EigenConvenience.h"
// Draw a "guide circle" like AntTweakBar's rotoslider given an center, mouse
// position at down and current.
//
// Inputs:
//   center  2d position of circle center
//   from  2d positoin of mouse at down.
//   to  2d position of mouse
//   radius  radius of circle in screen units
//   min_angle  minimum angle
//   max_angle  maximum angle
void draw_guide_circle(
  const Eigen::Vector2d & center,
  const Eigen::Vector2d & from,
  const Eigen::Vector2d & to,
  const double radius,
  const double min_angle,
  const double max_angle);
#endif
