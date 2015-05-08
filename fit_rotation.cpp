#include "fit_rotation.h"
#include <igl/polar_svd.h>
template <typename DerivedC, typename DerivedR>
void fit_rotation(
  const Eigen::PlainObjectBase<DerivedC> & C,
  Eigen::PlainObjectBase<DerivedR> & R)
{
  using namespace Eigen;
  MatrixXd dRT,_,svdU,svdV;
  VectorXd svdS;
  igl::polar_svd(C,R,_,svdU,svdS,svdV);
  MatrixXd temp = svdU * svdS.asDiagonal() * svdV.transpose();
  //assert(R.determinant() >= 0);
  // Check for reflection
  if(R.determinant() < 0)
  {
    svdU.col(svdU.cols()-1) *= -1.0;
    R = svdU * svdV.transpose();
  }
  assert(R.determinant() >= 0);
}

// Explicit template specialization
template void fit_rotation<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > &);
