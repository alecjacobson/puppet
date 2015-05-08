#include "match_skeletons.h"
#include "hungarian.h"
#include <igl/project_to_line_segment.h>

void match_skeletons(
  const Eigen::MatrixXd & rC,
  const Eigen::MatrixXi & rBE,
  const Eigen::MatrixXd & dC,
  const Eigen::MatrixXi & dBE,
  Eigen::VectorXi & I)
{
  using namespace igl;
  using namespace Eigen;
  MatrixXd D(dBE.rows(),rBE.rows());
  // Loop over device skeleton
  for(int d = 0;d<dBE.rows();d++)
  {
    // Loop over rig skeleton
    for(int r = 0;r<rBE.rows();r++)
    {
      // Compute Hausdorff distance between ab and cd
      const auto hausdorff = [&](
        const RowVector3d & a,
        const RowVector3d & b,
        const RowVector3d & c,
        const RowVector3d & d) -> double
      {
        Vector4d D;
        Matrix<double,1,1> t, vsqrD;
        project_to_line_segment(a,c,d,t,vsqrD); D(0) = vsqrD(0);
        project_to_line_segment(b,c,d,t,vsqrD); D(1) = vsqrD(0);
        project_to_line_segment(c,a,b,t,vsqrD); D(2) = vsqrD(0);
        project_to_line_segment(d,a,b,t,vsqrD); D(3) = vsqrD(0);
        return sqrt(D.maxCoeff());
      };
      D(d,r) =
        hausdorff(
          rC.row(rBE(r,0)),
          rC.row(rBE(r,1)),
          dC.row(dBE(d,0)),
          dC.row(dBE(d,1)));
    }
  }
  double cost;
  hungarian(D,I,cost);
  assert(I.size() == dBE.rows());
  assert(I.size()==0 || I.maxCoeff() < rBE.rows());
}
