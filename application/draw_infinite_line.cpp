#include "draw_infinite_line.h"
#include "GL_include.h"

void draw_infinite_line(const double * axis)
{
  // Keep track of opengl settings
  int cm;
  glGetIntegerv(GL_COLOR_MATERIAL,&cm);
  int l;
  glGetIntegerv(GL_LIGHTING,&l);
  int s;
  glGetIntegerv(GL_LINE_STIPPLE,&s);
  double lw;
  glGetDoublev(GL_LINE_WIDTH,&lw);

  // Set up colors
  glEnable(GL_COLOR_MATERIAL);
  glDisable(GL_LIGHTING);
  glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);
  float mat_ambient[4] = {0.1,0.1,0.1,1.0};
  float mat_specular[4] = {0.0,0.0,0.0,1.0};
  float mat_shininess = 1;
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,  mat_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,  mat_ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
  glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
  glDisable(GL_LIGHTING);

  glLineWidth(2);
  // Draw line to projection
  glColor3fv(INFINITE_LINE_COLOR);
  glDisable(GL_LINE_STIPPLE);

  double large = 1000.0;
  glBegin(GL_LINES);
  glVertex3d(
   -large*axis[0],
   -large*axis[1],
   -large*axis[2]);
  glVertex3d(
   large*axis[0],
   large*axis[1],
   large*axis[2]);
  glEnd();

  glLineWidth(lw);
  (s ? glEnable(GL_LINE_STIPPLE):glDisable(GL_LINE_STIPPLE));
  (l ? glEnable(GL_LIGHTING):glDisable(GL_LIGHTING));
  (cm ? glEnable(GL_COLOR_MATERIAL):glDisable(GL_COLOR_MATERIAL));

}

