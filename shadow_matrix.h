#ifndef SHADOW_MATRIX_H
#define SHADOW_MATRIX_H
#include "EigenConvenience.h"
// Compute a shadow projection matrix
//
// Inputs:
//   plane  4-vector plane equation of "floor"
//   light_pos  light position in homogenous coordinates
// Returns 16-element matrix (must be cleaned up)
void shadow_matrix(
  const Eigen::Vector4d & plane,
  const Eigen::Vector4d & light_pos,
  Eigen::Matrix4d & S);
#endif
