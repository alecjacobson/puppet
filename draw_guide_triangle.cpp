#include "draw_guide_triangle.h"
#include "GL_include.h"

static float DRAG_LINE_GUIDE_COLOR[4] = {120./255.,113./255.,113./255.,255./255.};
static float DRAG_LINE_COLOR[4] = {239./255.,46./255.,46./255.,255./255.};

void draw_guide_triangle(
  const double * A,
  const double * B,
  const double * C)
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

  // Set up drag guide line style
  glLineWidth(2);
  glEnable(GL_LINE_STIPPLE);
  glLineStipple(1,0xAAAA);
  glColor3fv(DRAG_LINE_GUIDE_COLOR);

  // Draw axis guide line
  double scale = 2000;
  glBegin(GL_LINES);
  glVertex3d(
    B[0] + scale*(B[0]-C[0]),
    B[1] + scale*(B[1]-C[1]),
    B[2] + scale*(B[2]-C[2]));
  glVertex3d(
    B[0] - scale*(B[0]-C[0]),
    B[1] - scale*(B[1]-C[1]),
    B[2] - scale*(B[2]-C[2]));
  glEnd();

  // Draw line to mouse
  glColor3fv(DRAG_LINE_GUIDE_COLOR);
  glBegin(GL_LINES);
  glVertex3dv(B);
  glVertex3dv(A);
  glEnd();
  // Draw line to projection
  glColor3fv(DRAG_LINE_GUIDE_COLOR);
  glBegin(GL_LINES);
  glVertex3dv(C);
  glVertex3dv(A);
  glEnd();
  glLineWidth(2);
  // Draw line to projection
  glColor3fv(DRAG_LINE_COLOR);
  glDisable(GL_LINE_STIPPLE);
  glBegin(GL_LINES);
  glVertex3dv(B);
  glVertex3dv(C);
  glEnd();


  glLineWidth(lw);
  (s ? glEnable(GL_LINE_STIPPLE):glDisable(GL_LINE_STIPPLE));
  (l ? glEnable(GL_LIGHTING):glDisable(GL_LIGHTING));
  (cm ? glEnable(GL_COLOR_MATERIAL):glDisable(GL_COLOR_MATERIAL));
}
