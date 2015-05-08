#ifndef TREE_FIT_H
#define TREE_FIT_H

#include "EigenConvenience.h"
#include <igl/SolverStatus.h>

struct TreeFitParams;
// TREE_FIT Fit a "stretchable tree" with vertices O and parental connectivity
// P to a given set of user specified locations.
//
// U = tree_fit(O,P,b,bc)
// [U,S] = tree_fit(O,P,b,bc,'ParameterName',ParameterValue, ...)
// 
// Inputs:
//   O  #O by dim list of initial vertex positions
//   P  #O list of parent indices (0 means root)
//   b  #b list of soft-constrained indices into O
//   bc  #b by dim list of soft-constraint positions
//   Optional:
//     'MinS' followed by minimum s value (scalar or #O long list) {0}
//     'MaxIter' followed by maximum number of iterations {100}
// Outputs:
//   N  #O by dim list of new positions
//   S  nnr list of stretch factors corresponding to edges from non-roots
//   find(P~=0) to their parents P(P~=0)
//
// Returns status of solve
template <
  typename DerivedO,
  typename DerivedP,
  typename Derivedb,
  typename Derivedbc,
  typename DerivedN,
  typename DerivedS,
  typename DerivedR>
igl::SolverStatus tree_fit(
  const Eigen::PlainObjectBase<DerivedO> & O,
  const Eigen::PlainObjectBase<DerivedP> & P,
  const Eigen::PlainObjectBase<Derivedb> & b,
  const Eigen::PlainObjectBase<Derivedbc> & bc,
  const TreeFitParams & params,
  Eigen::PlainObjectBase<DerivedN> & N,
  Eigen::PlainObjectBase<DerivedS> & S,
  Eigen::PlainObjectBase<DerivedR> & R);

struct TreeFitParams
{
  Eigen::VectorXd Smin;
  int max_iter;
  double min_dZ,w_user,w_damp,w_smooth,w_drag;
  TreeFitParams():
    Smin(Eigen::VectorXd::Constant(1,1,0.05)),
    max_iter(1),
    min_dZ(1e-5),
    w_user(1e0),
    w_damp(1e-1),
    w_smooth(1e-3),
    w_drag(1e-4)
  {};
  bool operator==(const TreeFitParams & rhs) const
  {
    return 
      max_iter == rhs.max_iter && 
      Smin.rows() == rhs.Smin.rows() && 
      Smin == rhs.Smin && 
      min_dZ == rhs.min_dZ && 
      w_user == rhs.w_user && 
      w_damp == rhs.w_damp && 
      w_smooth == rhs.w_smooth && 
      w_drag == rhs.w_drag;
  };
};

#endif
