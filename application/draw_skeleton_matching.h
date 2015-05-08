#ifndef DRAW_SKELETON_MATCHING_H
#define DRAW_SKELETON_MATCHING_H
#include "EigenConvenience.h"
// Draw matching between skeletons.
//
// Inputs:
//   rC  #rC by 3 list of rig joint positions
//   rBE  #rBE by 2 list of rig bone edge indices
//   dC  #dC by 3 list of device joint positions
//   dBE  #dBE by 2 list of device bone edge indices
//   I  #rBE list of indices into dBE
void draw_skeleton_matching(
  const Eigen::MatrixXd & rC,
  const Eigen::MatrixXi & rBE,
  const Eigen::MatrixXd & dC,
  const Eigen::MatrixXi & dBE,
  const Eigen::VectorXi & I);

#endif

