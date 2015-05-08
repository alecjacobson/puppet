#ifndef DRAW_STRING_H
#define DRAW_STRING_H

#include "GLUT_include.h"

#include <string>
// Draw a string at a point in 3D space
//
// Inputs:
//   x  x-position
//   y  y-position
//   z  z-position
//   text  string of text to draw
//   Optional:
//      font  glut font to use
//        http://www.opengl.org/resources/libraries/glut/spec3/node76.html
//        {GLUT_BITMAP_HELVETICA_18}
void draw_string(
    const double x, 
    const double y, 
    const double z, 
    const std::string str,
    void * font = GLUT_BITMAP_HELVETICA_18);
#endif
