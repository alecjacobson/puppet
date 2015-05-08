#include "quattrans2affine.h"

void quattrans2affine(
  const std::vector<
    Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > & vQ,
  const std::vector<Eigen::Vector3d> & vT,
  std::vector<
    Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & A)
{
  assert(vQ.size() == vT.size());
  A.resize(vQ.size());
  for(int i = 0;i<(int)A.size();i++)
  {
    A[i] = Eigen::Affine3d::Identity();
    A[i].translate(vT[i]);
    A[i].rotate(vQ[i]);
  }
}

