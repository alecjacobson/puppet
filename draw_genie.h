#ifndef DRAW_GENIE
#include "AugViewport.h"
#include "EigenConvenience.h"
class Node;
// Draw a small version of the puppet in a given viewport
//
// Inputs:
//   vp  Viewport
//   r  Root node
//   bg_color  Background color
//
void draw_genie(
  const AugViewport & vp, 
  const Node * r,
  const Eigen::Vector4f & bg_color);

#endif
