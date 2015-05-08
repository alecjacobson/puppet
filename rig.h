#ifndef RIG_H
#define RIG_H
#include "EigenConvenience.h"
class Node;
class Mesh;
namespace igl{class BBWData;};
// Compute skinning weights and lbs
// matrix.
//
// Inputs:
//   m  mesh to bind to
//   bbw_data  data for parameters and output of bbw weight computation
//   root  pointer to root node
// Outputs:
//   W  #TV by #bones list of weights
//   M  #V by #bones*(3+1) lbs matrix
//   TV  #TV by 3 list of tet mesh vertices
//   TT  #TT by 4 list of tet indices
//   TF  #TF by 3 list of tet mesh faces
//
bool rig(
  const Mesh & m,
  igl::BBWData & bbw_data,
  const Node * root,
  Eigen::MatrixXd & W,
  Eigen::MatrixXd & M,
  Eigen::MatrixXd & TV,
  Eigen::MatrixXi & TT,
  Eigen::MatrixXi & TF);
// wrapper without tet mesh
bool rig(
  const Mesh & m,
  igl::BBWData & bbw_data,
  const Node * root,
  Eigen::MatrixXd & W,
  Eigen::MatrixXd & M);
#endif

