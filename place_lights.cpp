#include "place_lights.h"
#include "GL_include.h"
#include <cassert>

template <typename DerivedL>
void place_lights(
  const Eigen::PlainObjectBase<DerivedL> & L,
  const int offset,
  const Eigen::Vector4f & ambient,
  const Eigen::Vector4f & diffuse,
  const Eigen::Vector4f & specular,
  const bool invert)
{
  using namespace std;
  using namespace Eigen;
  assert(L.rows() == 4);
  glEnable(GL_LIGHTING);
  const int num = L.cols();
  for(int l = 0;l<num;l++)
  {
    glEnable( GL_LIGHT0+offset+l);
    glLightfv(GL_LIGHT0+offset+l,GL_AMBIENT,ambient.data());
    glLightfv(GL_LIGHT0+offset+l,GL_DIFFUSE,diffuse.data());
    glLightfv(GL_LIGHT0+offset+l,GL_SPECULAR,specular.data());
    Vector4f pos = L.col(l).template cast<float>();
    if(invert)
    {
      pos *= -1.;
    }
    glLightfv(GL_LIGHT0+offset+l,GL_POSITION,pos.data());
  }
}


template <typename DerivedL>
void place_lights(
  const Eigen::PlainObjectBase<DerivedL> & L,
  const int offset,
  const Eigen::Vector4f & color,
  const bool invert)
{
  return place_lights(L,offset,color,color,color,invert);
}

// Explicit instanciation
template void place_lights<Eigen::Matrix<float, 4, 1, 0, 4, 1> >(Eigen::PlainObjectBase<Eigen::Matrix<float, 4, 1, 0, 4, 1> > const&, int, Eigen::Matrix<float, 4, 1, 0, 4, 1> const&, Eigen::Matrix<float, 4, 1, 0, 4, 1> const&, Eigen::Matrix<float, 4, 1, 0, 4, 1> const&, bool);
template void place_lights<Eigen::Matrix<float, 4, -1, 0, 4, -1> >(Eigen::PlainObjectBase<Eigen::Matrix<float, 4, -1, 0, 4, -1> > const&, int, Eigen::Matrix<float, 4, 1, 0, 4, 1> const&, bool);
