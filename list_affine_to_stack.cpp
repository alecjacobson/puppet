#include "list_affine_to_stack.h"
template <typename DerivedT>
void list_affine_to_stack(
  const std::vector<
    Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & vA,
  Eigen::PlainObjectBase<DerivedT> & T)
{
  T.resize(vA.size()*4,3);
  for(int i = 0;i<(int)vA.size();i++)
  {
    T.block(i*4,0,4,3) = vA[i].matrix().transpose().block(0,0,4,3);
  }
}

template void list_affine_to_stack<Eigen::Matrix<double, -1, -1, 0, -1, -1> >(std::vector<Eigen::Transform<double, 3, 2, 0>, Eigen::aligned_allocator<Eigen::Transform<double, 3, 2, 0> > > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);
