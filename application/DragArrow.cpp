#include "DragArrow.h"
#include "GL_include.h"
#include "draw_string.h"

#include <igl/opengl2/draw_point.h>
#include <igl/opengl2/project.h>
#include <igl/opengl2/unproject.h>

#include <iostream>

DragArrow::DragArrow():
  from(),
  to(),
  activated(false),
  is_down(false),
  snapping(false),
  static_from(false),
  delay_snap(false),
  down_x(0),
  down_y(0),
  down_to(),
  down_from()
{
}

static const float DRAG_ARROW_FROM_POINT_COLOR[4] = {150./255.,180./255.,255./255.,1.};
static const float DRAG_ARROW_TO_POINT_COLOR[4] = {120./255.,130./255.,240./255.,1.};
static const float DRAG_ARROW_LINE_COLOR[4] = {80./255.,90./255.,160./255.,1.};
static const double DRAG_ARROW_FROM_POINT_RADIUS = 10;
static const double DRAG_ARROW_TO_POINT_RADIUS = 7;
void DragArrow::draw()
{
  using namespace igl;
  using namespace igl::opengl2;

  GLboolean ol,dt,ols;
  glGetBooleanv(GL_LIGHTING,&ol);
  glGetBooleanv(GL_DEPTH_TEST,&dt);
  glGetBooleanv(GL_LINE_STIPPLE,&ols);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  const double STR_OFF = 4.5;
  if(activated)
  {
    glColor4fv(DRAG_ARROW_LINE_COLOR);
    glLineWidth(5);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0xAAAA);
    glBegin(GL_LINES);
    glVertex3dv(from.data());
    glVertex3dv(to.data());
    glEnd();
    //glDisable(GL_LINE_STIPPLE);
    glColor4fv(DRAG_ARROW_FROM_POINT_COLOR);
    draw_point(from(0),from(1),from(2),DRAG_ARROW_FROM_POINT_RADIUS,false);
    glColor4fv(DRAG_ARROW_TO_POINT_COLOR);
    draw_point(to(0),to(1),to(2),DRAG_ARROW_TO_POINT_RADIUS,false);
    if(is_down && !delay_snap && static_from)
    {
      Vec3 pos = from;
      Vec3 win;
      project(pos,win);
      win(0)-= STR_OFF;
      win(1)-= STR_OFF;
      unproject(win,pos);
      glColor3f(0.1,0.1,0.1);
      draw_string(pos(0),pos(1),pos(2),"-");
    }
  }else
  {
    glColor4fv(DRAG_ARROW_FROM_POINT_COLOR);
    draw_point(from(0),from(1),from(2),DRAG_ARROW_FROM_POINT_RADIUS,false);
    Vec3 pos = from;
    Vec3 win;
    project(pos,win);
    win(0)-= STR_OFF;
    win(1)-= STR_OFF;
    unproject(win,pos);
    // What's the best part about living in Switzerland?
    // Well, the flag is a big plus.
    glColor3f(0.1,0.1,0.1);
    draw_string(pos(0),pos(1),pos(2),"+");
  }

  ols?glEnable(GL_LINE_STIPPLE):glDisable(GL_LINE_STIPPLE);
  ol?glEnable(GL_LIGHTING):glDisable(GL_LIGHTING);
  dt?glEnable(GL_DEPTH_TEST):glDisable(GL_DEPTH_TEST);
}

bool DragArrow::down(const int x, const int y, const int /*mod*/)
{
  using namespace igl;
  using namespace igl::opengl2;
  using namespace std;
  is_down = false;
  down_x = x;
  down_y = y;
  down_from = from;
  static_from = true;
  Vec3 win_to,win_from;
  project(to,win_to);
  // use win_to's z
  if((win_to - Vec3((double)x,(double)y,win_to(2))).norm() < 
    DRAG_ARROW_TO_POINT_RADIUS)
  {
    is_down = true;
    down_to = to;
    goto finish;
  }
  project(from,win_from);
  if((win_from - Vec3((double)x,(double)y,win_from(2))).norm() < 
    DRAG_ARROW_FROM_POINT_RADIUS)
  {
    is_down = true;
    // pull out from... err... from
    down_to = from;
    goto finish;
  }
finish:
  // If already activated then delay snapping
  if(is_down)
  {
    delay_snap = activated;
  }
  return is_down;
}

static const double SNAP_DIST = 10;

bool DragArrow::drag(const int x, const int y, const int mod)
{
  using namespace igl;
  using namespace igl::opengl2;
  using namespace std;
  if(!is_down)
  {
    return false;
  }
  // Use to's z
  Vec3 win_to;
  project(to,win_to);
  return drag(x,y,win_to[2],mod);
}

bool DragArrow::drag(const int x, const int y, const double z, const int /*mod*/)
{
  using namespace igl;
  using namespace igl::opengl2;
  using namespace std;
  if(!is_down)
  {
    return false;
  }
  Vec3 win_to;
  project(down_to,win_to);
  // This can be computed on down
  win_to(0) += (x-down_x);
  win_to(1) += (y-down_y);
  win_to(2) = z;
  unproject(win_to,to);
  snapping = false;
  Vec3 win_from;
  project(from,win_from);
  Vec3 win_down_from;
  project(down_from,win_down_from);
  if((win_down_from-win_from).norm() < 0.1*SNAP_DIST && static_from)
  {
    // Snapping in screen space
    const double dist = (win_to-win_from).norm();
    const double eff_SNAP_DIST = delay_snap?1.5*SNAP_DIST:SNAP_DIST;
    if(dist < eff_SNAP_DIST)
    {
      snapping = !delay_snap;
    }else
    {
      delay_snap = false;
    }
  }else
  {
    static_from = false;
  }
  activated = !snapping;

  return is_down;
}

bool DragArrow::up(const int /*x*/, const int /*y*/, const int /*mod*/)
{
  is_down = false;
  snapping = false;
  return is_down;
}
