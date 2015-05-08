#ifndef LIST_AFFINE_TO_STACK_H
#define LIST_AFFINE_TO_STACK_H
#include "EigenConvenience.h"
#include <vector>

// Stack up transformation matrices
//
// Inputs:
//   vA  #T list of 3d affine transformations
// Output:
//   T  #T*(3+1) by 3 stack of transposed transformation matrices
template <typename DerivedT>
void list_affine_to_stack(
  const std::vector<
    Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & vA,
  Eigen::PlainObjectBase<DerivedT> & T);
#endif
