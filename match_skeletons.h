#ifndef MATCH_SKELETONS_H
#define MATCH_SKELETONS_H
#include "EigenConvenience.h"
// Match a "rig" skeleton to a "device" skeleton
//
// Inputs:
//   rC  #rC by 3 list of rig joint positions
//   rBE  #rBE by 3 list of rig bone edge indices
//   dC  #dC by 3 list of device joint positions
//   dBE  #dBE by 3 list of device bone edge indices
// Output:
//   I  #dBE list of indices into rBE
void match_skeletons(
  const Eigen::MatrixXd & rC,
  const Eigen::MatrixXi & rBE,
  const Eigen::MatrixXd & dC,
  const Eigen::MatrixXi & dBE,
  Eigen::VectorXi & I);

#endif
