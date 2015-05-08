#include "draw_skeleton_matching.h"
#include "GL_include.h"
#include <igl/barycenter.h>

void draw_skeleton_matching(
  const Eigen::MatrixXd & rC,
  const Eigen::MatrixXi & rBE,
  const Eigen::MatrixXd & dC,
  const Eigen::MatrixXi & dBE,
  const Eigen::VectorXi & I)
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;

  GLboolean ol,dt,ols;
  glGetBooleanv(GL_LIGHTING,&ol);
  glGetBooleanv(GL_DEPTH_TEST,&dt);
  glGetBooleanv(GL_LINE_STIPPLE,&ols);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  const Vector4f SKELETON_MATCHING_LINE_COLOR(0.4,0.8,0.4,1.0);
  glColor4fv(SKELETON_MATCHING_LINE_COLOR.data());
  glLineWidth(5);
  glEnable(GL_LINE_STIPPLE);
  glLineStipple(2, 0xAAAA);

  // Draw a dashed line between edge barycenters
  MatrixXd rBC,dBC;
  barycenter(rC,rBE,rBC);
  barycenter(dC,dBE,dBC);
  glBegin(GL_LINES);
  for(int i = 0;i<I.rows();i++)
  {
    const int ii = I(i);
    if(ii>0)
    {
      glVertex3d(rBC(ii,0),rBC(ii,1),rBC(ii,2));
      glVertex3d(dBC(i,0),dBC(i,1),dBC(i,2));
    }
  }
  glEnd();

  ols?glEnable(GL_LINE_STIPPLE):glDisable(GL_LINE_STIPPLE);
  ol?glEnable(GL_LIGHTING):glDisable(GL_LIGHTING);
  dt?glEnable(GL_DEPTH_TEST):glDisable(GL_DEPTH_TEST);
}
