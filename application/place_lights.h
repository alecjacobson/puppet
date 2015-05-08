#ifndef PLACE_LIGHTS_H
#define PLACE_LIGHTS_H
#include "EigenConvenience.h"
// Place a number of lights given a list of positions and colors
//
// Inputs:
//   L  4 by #lights **Column vectors** are light positions.
//   offset  offset in OpenGL GL_LIGHT* enums start with
//   color  color of lights
//   invert  whether to invert light direction {false}
template <typename DerivedL>
void place_lights(
  const Eigen::PlainObjectBase<DerivedL> & L,
  const int offset,
  const Eigen::Vector4f & ambient,
  const Eigen::Vector4f & diffuse,
  const Eigen::Vector4f & specular,
  const bool invert = false);

template <typename DerivedL>
void place_lights(
  const Eigen::PlainObjectBase<DerivedL> & L,
  const int offset,
  const Eigen::Vector4f & color,
  const bool invert = false);

#endif
