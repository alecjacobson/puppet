#include "rigid2d.h"

Eigen::Affine3d rigid2d(
  const Eigen::Vector2d & center,
  const double angle)
{
  using namespace Eigen;
  Affine3d t = Affine3d::Identity();
  const Vector3d c3(center(0),center(1),0);
  t.translate(c3);
  t.rotate(AngleAxisd(angle,Vector3d(0,0,1)));
  t.translate(-c3);
  return t;
}

