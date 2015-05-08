#ifndef BIND_H
#define BIND_H
#include "EigenConvenience.h"
class Node;
class Mesh;
namespace igl{class BBWData;};
// Tell each node that it's being bound. Assumes that `rig` has recently been
// called.
//
// Inputs:
//   root  pointer to root node
//   rC  #rC by 3 list of rig joint positions
//   rBE  #rBE by 3 list of rig bone edge indices
// Output:
//   I  #dBE list of indices into rBE
//
void bind(
  const Eigen::MatrixXd & rC,
  const Eigen::MatrixXi & rBE,
  Node * root,
  Eigen::VectorXi & I);

#endif
