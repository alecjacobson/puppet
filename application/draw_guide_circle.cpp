#include "draw_guide_circle.h"
#include "GL_include.h"
#include <igl/PI.h>
#include <algorithm>

void draw_guide_circle(
  const Eigen::Vector2d & center,
  const Eigen::Vector2d & from,
  const Eigen::Vector2d & to,
  const double radius,
  const double /*min_angle*/,
  const double /*max_angle*/)
{
  using namespace Eigen;
  using namespace igl;
  using namespace std;
  const Vector4d RED(1,0,0,1);
  const Vector4d GREY(0.5,0.5,0.5,1);
  const Vector4d DARK_GREY(0.3,0.3,0.3,1);
  // Keep track of opengl settings
  int cm;
  glGetIntegerv(GL_COLOR_MATERIAL,&cm);
  int l;
  glGetIntegerv(GL_LIGHTING,&l);
  int s;
  glGetIntegerv(GL_LINE_STIPPLE,&s);
  double lw;
  glGetDoublev(GL_LINE_WIDTH,&lw);
  int d;
  glGetIntegerv(GL_DEPTH_TEST,&d);

  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  // draw thick solid circle
  glDisable(GL_LINE_STIPPLE);
  glLineWidth(2);
  glColor4dv(GREY.data());
  glBegin(GL_LINE_STRIP);
  for(double th = 0;th<2.*PI;th+=0.01)
  {
    Vector2d p = center + radius*Vector2d(cos(th),sin(th));
    glVertex2dv(p.data());
  }
  glEnd();

  // Draw thick solid red arc project `from` to projected `to`
  glDisable(GL_LINE_STIPPLE);
  glLineWidth(2);
  glColor4dv(RED.data());
  glBegin(GL_LINE_STRIP);
  double from_th = atan2(from(1)-center(1),from(0)-center(0));
  double to_th = atan2(to(1)-center(1),to(0)-center(0));
  if(to_th < from_th)
  {
    swap(from_th,to_th);
  }
  if(to_th - from_th > PI)
  {
    swap(from_th,to_th);
    to_th += 2.*PI;
  }
  for(double th = from_th;th<to_th;th+=0.01)
  {
    Vector2d p = center + radius*Vector2d(cos(th),sin(th));
    glVertex2dv(p.data());
  }
  glEnd();

  // draw thick solid red line from center to `to`
  glDisable(GL_LINE_STIPPLE);
  glLineWidth(2);
  glColor4dv(RED.data());
  glBegin(GL_LINES);
  glVertex2dv(center.data());
  glVertex2dv(to.data());
  glEnd();

  // draw thin dashed gray line from center to `from`
  glLineWidth(1);
  glEnable(GL_LINE_STIPPLE);
  glLineStipple(1,0xAAAA);
  glColor4dv(GREY.data());
  glBegin(GL_LINES);
  glVertex2dv(center.data());
  glVertex2dv(from.data());
  glEnd();

  // draw thick gray solid max and min lines
  //glDisable(GL_LINE_STIPPLE);
  //glLineWidth(2);
  //glColor4dv(DARK_GREY.data());
  //{
  //  Vector2d A,B;
  //  const double r = radius*1.1;
  //  glBegin(GL_LINE_STRIP);
  //  glVertex2dv(center.data());
  //  A = center + r*Vector2d(cos(min_angle),sin(min_angle));
  //  glVertex2dv(A.data());
  //  B = center + r*Vector2d(cos(min_angle+0.1),sin(min_angle+0.1));
  //  glVertex2dv(B.data());
  //  glEnd();
  //  glBegin(GL_LINE_STRIP);
  //  glVertex2dv(center.data());
  //  A = center + r*Vector2d(cos(max_angle),sin(max_angle));
  //  glVertex2dv(A.data());
  //  B = center + r*Vector2d(cos(max_angle-0.1),sin(max_angle-0.1));
  //  glVertex2dv(B.data());
  //  glEnd();
  //}
            
  glLineWidth(lw);
  (s ? glEnable(GL_LINE_STIPPLE):glDisable(GL_LINE_STIPPLE));
  (d ? glEnable(GL_DEPTH_TEST):glDisable(GL_DEPTH_TEST));
  (l ? glEnable(GL_LIGHTING):glDisable(GL_LIGHTING));
  (cm ? glEnable(GL_COLOR_MATERIAL):glDisable(GL_COLOR_MATERIAL));
}
