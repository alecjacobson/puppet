#include "GL_include.h"

void dim_lights(const double f)
{
  int max_lights = 0;
  glGetIntegerv(GL_MAX_LIGHTS,&max_lights);
  float ld[4];
  for(int l = 0; l<max_lights; l++)
  {
    // Dim if hovered over
    glGetLightfv(GL_LIGHT0+l,GL_DIFFUSE,ld);
    ld[0] *= f;
    ld[1] *= f;
    ld[2] *= f;
    ld[3] *= f;
    glLightfv(GL_LIGHT0+l,GL_DIFFUSE,ld);
  }
}
