#include "throw_at.h"
template <typename T, typename TAlloc>
void throw_at(
  const std::vector<T,TAlloc> & A,
  const Eigen::VectorXi & RP,
  std::vector<T,TAlloc> & B)
{
  // Seems like this is the same as the "reorder" here:
  // http://stackoverflow.com/a/1267878/148668

  // Boring one-off
  if(RP.size() == 0)
  {
    assert(A.size() == 0);
    B.resize(0);
    return;
  }
  B.resize(RP.maxCoeff()+1);
  for(int a = 0;a<(int)A.size();a++)
  {
    B[RP(a)] = A[a];
  }
}

template <typename T, typename TAlloc>
std::vector<T,TAlloc> throw_at(
  const std::vector<T,TAlloc> & A,
  const Eigen::VectorXi & RP)
{
  std::vector<T,TAlloc> B;
  throw_at(A,RP,B);
  return B;
}

// Explicit template instanciation
template std::vector<Eigen::Matrix<double, 3, 1, 0, 3, 1>, std::allocator<Eigen::Matrix<double, 3, 1, 0, 3, 1> > > throw_at<Eigen::Matrix<double, 3, 1, 0, 3, 1>, std::allocator<Eigen::Matrix<double, 3, 1, 0, 3, 1> > >(std::vector<Eigen::Matrix<double, 3, 1, 0, 3, 1>, std::allocator<Eigen::Matrix<double, 3, 1, 0, 3, 1> > > const&, Eigen::Matrix<int, -1, 1, 0, -1, 1> const&);
template void throw_at<Eigen::Matrix<double, 3, 1, 0, 3, 1>, std::allocator<Eigen::Matrix<double, 3, 1, 0, 3, 1> > >(std::vector<Eigen::Matrix<double, 3, 1, 0, 3, 1>, std::allocator<Eigen::Matrix<double, 3, 1, 0, 3, 1> > > const&, Eigen::Matrix<int, -1, 1, 0, -1, 1> const&, std::vector<Eigen::Matrix<double, 3, 1, 0, 3, 1>, std::allocator<Eigen::Matrix<double, 3, 1, 0, 3, 1> > >&);
template std::vector<Eigen::Quaternion<double, 0>, Eigen::aligned_allocator<Eigen::Quaternion<double, 0> > > throw_at<Eigen::Quaternion<double, 0>, Eigen::aligned_allocator<Eigen::Quaternion<double, 0> > >(std::vector<Eigen::Quaternion<double, 0>, Eigen::aligned_allocator<Eigen::Quaternion<double, 0> > > const&, Eigen::Matrix<int, -1, 1, 0, -1, 1> const&);
template void throw_at<Eigen::Quaternion<double, 0>, Eigen::aligned_allocator<Eigen::Quaternion<double, 0> > >(std::vector<Eigen::Quaternion<double, 0>, Eigen::aligned_allocator<Eigen::Quaternion<double, 0> > > const&, Eigen::Matrix<int, -1, 1, 0, -1, 1> const&, std::vector<Eigen::Quaternion<double, 0>, Eigen::aligned_allocator<Eigen::Quaternion<double, 0> > >&);
