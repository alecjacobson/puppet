#ifndef BEZIER_H
#define BEZIER_H
// Cubic bezier
#include <vector>
#include <Eigen/Geometry>
#include <functional>

double bezier(
  const double t,
  const double & A,
  const double & B,
  const double & C,
  const double & D);

template <typename T>
T bezier(
  const double t,
  const typename std::vector<T>::const_iterator & qbegin,
  const typename std::vector<T>::const_iterator & qend,
  T (*lerp)(const double, const T &,const T&));

  

#endif
