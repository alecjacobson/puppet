#include "draw_number.h"
#include "draw_string.h"

#include <sstream>

#ifdef __APPLE__
#  include <OpenGL/gl.h>
#elif defined(_WIN32)
#    define NOMINMAX
#    include <Windows.h>
#    undef NOMINMAX
#else
#  include <GL/gl.h>
#endif

void draw_number(const int i)
{
  using namespace std;
  // Keep track of opengl settings
  int cm;
  glGetIntegerv(GL_COLOR_MATERIAL,&cm);
  int l;
  glGetIntegerv(GL_LIGHTING,&l);
  int s;
  glGetIntegerv(GL_LINE_STIPPLE,&s);
  double lw;
  glGetDoublev(GL_LINE_WIDTH,&lw);
  int dt;
  glGetIntegerv(GL_DEPTH_TEST,&dt);

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
  glDisable(GL_DEPTH_TEST);

  stringstream ss;
  ss<<i;
  draw_string(0,0,0,ss.str());

  // Reset settings
  glLineWidth(lw);
  (s ? glEnable(GL_LINE_STIPPLE):glDisable(GL_LINE_STIPPLE));
  (l ? glEnable(GL_LIGHTING):glDisable(GL_LIGHTING));
  (cm ? glEnable(GL_COLOR_MATERIAL):glDisable(GL_COLOR_MATERIAL));
  (dt ? glEnable(GL_DEPTH_TEST) : glDisable(GL_DEPTH_TEST));
}

