#ifndef FIT_ROTATION_H
#define FIT_ROTATION_H
#include "EigenConvenience.h"
// Fit a rotation, I mean really a rotation, to a covariance matrix.
//
// Inputs:
//   C  dim by dim covariance matrix
// Outputs:
//   R  dim by dim rotation matrix (determinant == 1)
//
template <typename DerivedC, typename DerivedR>
void fit_rotation(
  const Eigen::PlainObjectBase<DerivedC> & C,
  Eigen::PlainObjectBase<DerivedR> & R);
#endif
