#include "draw_axis.h"

#ifdef __APPLE__
#  include <OpenGL/gl.h>
#elif defined(_WIN32)
#    define NOMINMAX
#    include <Windows.h>
#    undef NOMINMAX
#else
#  include <GL/gl.h>
#endif

void draw_axis(double length, double width)
{
  // old settings
  int old_lighting=0;
  int old_depth_test=0;
  double old_line_width=1;
  glGetIntegerv(GL_LIGHTING,&old_lighting);
  glGetIntegerv(GL_DEPTH_TEST,&old_depth_test);
  glGetDoublev(GL_LINE_WIDTH,&old_line_width);
  glLineWidth(width);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glBegin(GL_LINES);
  for(int i = 0;i<3;i++)
  {
    glColor3f(i==0,i==1,i==2);
    glVertex3f(0,0,0);
    glColor3f(i==0,i==1,i==2);
    glVertex3f((i==0?length:0),(i==1?length:0),(i==2?length:0));
  }
  glEnd();
  // Reset settings
  (old_lighting ? glEnable(GL_LIGHTING) : glDisable(GL_LIGHTING));
  (old_depth_test ? glEnable(GL_DEPTH_TEST) : glDisable(GL_DEPTH_TEST));
  glLineWidth(old_line_width);
}
