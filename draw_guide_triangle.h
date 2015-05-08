#ifndef DRAW_GUIDE_TRIANGLE_H
#define DRAW_GUIDE_TRIANGLE_H
// Draw a "guide triangle" between three 3d points
//
// Inputs:
//   A  3d position of typically raw mouse
//   B  3d position of typically mouse down
//   C  3d position of typically snapped mouse
void draw_guide_triangle(
  const double * A,
  const double * B,
  const double * C);
#endif
