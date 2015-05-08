#include "draw_genie.h"
#include "push_scene.h"
#include "Node.h"
#include "GL_include.h"
#include <iostream>
void draw_genie(
  const AugViewport & vp, 
  const Node * r,
  const Eigen::Vector4f & bg_color)
{
  using namespace std;
  const double BAR_THICKNESS = 3.0;
  int old_vp[4];
  glGetIntegerv(GL_VIEWPORT,old_vp);
  glViewport(
    vp.x,
    vp.y,
    vp.width,
    vp.height);
  glDepthMask(GL_FALSE);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  gluOrtho2D(0,vp.width,0,vp.height);
  glDisable(GL_LIGHTING);
  for(int p = 0;p<3;p++)
  {
    switch(p)
    {
      case 0:
        glColor4fv(bg_color.data());
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        break;
      case 1:
        glLineWidth(BAR_THICKNESS);
        glColor4f(0.5,0.5,0.5,1);
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
        break;
      case 2:
        glLineWidth(BAR_THICKNESS/3.0);
        glColor4f(0.8,0.8,0.8,1);
        glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
        break;
    }
    glBegin(GL_QUADS);
    glVertex2d(0,0);
    glVertex2d(vp.width,0);
    glVertex2d(vp.width,vp.height);
    glVertex2d(0,vp.height);
    glEnd();
  }
  glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
  glEnable(GL_LIGHTING);
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glDepthMask(GL_TRUE);
  push_scene(vp.camera);
  if(r)
  {
    r->draw();
  }
  pop_scene();
  glViewport(old_vp[0],old_vp[1],old_vp[2],old_vp[3]);
}

