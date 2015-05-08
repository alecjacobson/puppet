#ifndef SPHERE_RAY_INTERSECT
#define SPHERE_RAY_INTERSECT
#include "EigenConvenience.h"

// Sphere ray intersection
//
// Inputs:
//   c  center of sphere
//   r  radius of sphere
//   o  origin of ray
//   d  (unnormalized) direction of ray
// Outputs:
//   t0  parameterization of first hit (set only if exists) so that hit
//    position = o + t0*d
//   t1  parameterization of second hit (set only if exists)
// Returns number of hits
int sphere_ray_intersect(
  const Eigen::Vector3d & c,
  const double r,
  const Eigen::Vector3d & o,
  const Eigen::Vector3d & d,
  double & t0,
  double & t1);
#endif
