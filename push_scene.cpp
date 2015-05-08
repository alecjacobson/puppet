#include "push_scene.h"
#include "GL_include.h"

void push_scene(const igl::Camera & camera)
{
  using namespace igl;
  using namespace std;
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glMultMatrixd(camera.projection().data());
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  gluLookAt(
    camera.eye()(0), camera.eye()(1), camera.eye()(2),
    camera.at()(0), camera.at()(1), camera.at()(2),
    camera.up()(0), camera.up()(1), camera.up()(2));
}

void pop_scene()
{
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}
