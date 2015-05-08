#ifndef QUATTRANS2AFFINE_H
#define QUATTRANS2AFFINE_H
#include "EigenConvenience.h"
#include <vector>

//  vector<Quaterniond,aligned_allocator<Quaterniond> > vQ;
void quattrans2affine(
  const std::vector<
    Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > & vQ,
  const std::vector<Eigen::Vector3d> & vT,
  std::vector<
    Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & A);

#endif
