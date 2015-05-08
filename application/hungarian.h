#ifndef HUNGARIAN_H
#define HUNGARIAN_H
#include <Eigen/Core>
// Solve the minimal rectangular assignment problem using the Hungarian
// method. Assigns each column to a row (if m>n then some rows will not get an
// assignment).
// 
// Inputs:
//   D  m by n matrix of distances
// Outputs:
//   A  m list of assignments -1 means not assigned
//   c  cost 
//
template <typename DerivedD, typename DerivedA, typename c_type> 
void hungarian(
  const Eigen::PlainObjectBase<DerivedD> & D,
  Eigen::PlainObjectBase<DerivedA> & A,
  c_type & c);
#endif
