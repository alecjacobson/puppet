#include "FourView.h"
#include <igl/C_STR.h>
#include <igl/ZERO.h>
#include <igl/ONE.h>
#include <igl/get_seconds.h>
#include <igl/ReAnttweakBar.h>
#include <igl/snap_to_fixed_up.h>
#include <igl/canonical_quaternions.h>
#include "GLUT_include.h" 
#include <algorithm>

FourView::FourView():
  m_width(-1), m_height(-1),
  m_down_vp(-1), m_hover_vp(-1),
  m_horiz_on(false), m_vert_on(false),
  m_horiz(0.5), m_vert(0.5),
  m_down_camera()
{
  using namespace igl;
  using namespace std;
  // Initialize viewports
  for(auto & vp : m_viewports)
  {
    vp.camera.push_away(3.);
  }
  m_viewports[0].camera.dolly_zoom(0.-m_viewports[0].camera.m_angle);
  m_viewports[1].camera.dolly_zoom(0.-m_viewports[1].camera.m_angle);
  m_viewports[2].camera.dolly_zoom(0.-m_viewports[2].camera.m_angle);
  m_viewports[3].camera.dolly_zoom(25.-m_viewports[3].camera.m_angle);
  // Above view
  double XZ_PLANE_QUAT_D_FLIP[4];
  copy(XZ_PLANE_QUAT_D,XZ_PLANE_QUAT_D+4,XZ_PLANE_QUAT_D_FLIP);
  XZ_PLANE_QUAT_D_FLIP[0] *= -1.0;
  // Straight on
  copy(
    XZ_PLANE_QUAT_D_FLIP,
    XZ_PLANE_QUAT_D_FLIP+4,
    m_viewports[0].camera.m_rotation_conj.coeffs().data());
  // Left side view
  copy(
    XY_PLANE_QUAT_D,
    XY_PLANE_QUAT_D+4,
    m_viewports[1].camera.m_rotation_conj.coeffs().data());
  copy(
    CANONICAL_VIEW_QUAT_D[14],
    CANONICAL_VIEW_QUAT_D[14]+4,
    m_viewports[2].camera.m_rotation_conj.coeffs().data());
  // Straight on
  copy(
    XY_PLANE_QUAT_D,
    XY_PLANE_QUAT_D+4,
    m_viewports[3].camera.m_rotation_conj.coeffs().data());
}

double FourView::clamped_horiz() const
{
  const double MIN_H = (BAR_THICKNESS/2.0)/(double)m_height;
  return std::min(std::max(m_horiz,MIN_H),1.0-MIN_H);
}

double FourView::clamped_vert() const
{
  const double MIN_V = (BAR_THICKNESS/2.0)/(double)m_width;
  return std::min(std::max(m_vert,MIN_V),1.0-MIN_V);
}

void FourView::draw() const
{
  // remember settings
  int old_vp[4];
  glGetIntegerv(GL_VIEWPORT,old_vp);
  double lw;
  glGetDoublev(GL_LINE_WIDTH,&lw);
  GLboolean ol,dt,ols;
  glGetBooleanv(GL_LIGHTING,&ol);
  glGetBooleanv(GL_LINE_STIPPLE,&ols);
  glGetBooleanv(GL_DEPTH_TEST,&dt);

  // True screen space
  glViewport(0,0,m_width,m_height);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0,m_width,0,m_height);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LINE_STIPPLE);
  glDisable(GL_LIGHTING);

  const double horiz = clamped_horiz();
  const double vert = clamped_vert();
  glLineWidth(BAR_THICKNESS);
  glColor3f(0.5,0.5,0.5);
  glBegin(GL_LINES);
  glVertex2f(0,horiz*m_height);
  glVertex2f(m_width,horiz*m_height);
  glVertex2f(vert*m_width,0);
  glVertex2f(vert*m_width,m_height);
  glEnd();

  glLineWidth(BAR_THICKNESS/3.0);
  glColor3f(0.8,0.8,0.8);
  glBegin(GL_LINES);
  glVertex2f(0,horiz*m_height);
  glVertex2f(m_width,horiz*m_height);
  glVertex2f(vert*m_width,0);
  glVertex2f(vert*m_width,m_height);
  glEnd();

  // Restore settings
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glLineWidth(lw);
  glViewport(old_vp[0],old_vp[1],old_vp[2],old_vp[3]);
  ols?glEnable(GL_LINE_STIPPLE):glDisable(GL_LINE_STIPPLE);
  ol?glEnable(GL_LIGHTING):glDisable(GL_LIGHTING);
  dt?glEnable(GL_DEPTH_TEST):glDisable(GL_DEPTH_TEST);
}

void FourView::reshape(const int w, const int h)
{
  m_width = w;
  m_height = h;
  return reshape();
}

void FourView::reshape()
{
  // Make effective horiz and vert sane
  const double horiz = clamped_horiz();
  const double vert = clamped_vert();
  // resize each viewport
  const int w = m_width;
  const int h = m_height;
  m_viewports[0].reshape(vert*w,horiz*h,(1.-vert)*w,(1.-horiz)*h);
  m_viewports[1].reshape(     0,horiz*h,     vert*w,(1.-horiz)*h);
  m_viewports[2].reshape(     0,      0,     vert*w,     horiz*h);
  m_viewports[3].reshape(vert*w,      0,(1.-vert)*w,     horiz*h);
  // Reset aspects
  for(auto & vp : m_viewports)
  {
    vp.camera.m_aspect = (double)vp.width/(double)vp.height;
  }
}

bool FourView::down(const int x, const int y)
{
  const double horiz = clamped_horiz();
  const double vert = clamped_vert();
  m_down_vp = in_viewport_index(x,y);
  if(m_down_vp >= 0)
  {
    m_down_camera = m_viewports[m_down_vp].camera;
  }
  m_horiz_on = m_vert_on = false;
  if( (fabs((m_height-y) - horiz*m_height) < 2.*SNAP_DIST)
      && (fabs(x - vert*m_width) < 2.*SNAP_DIST))
  {
    //For lack of middle resize cursor
    glutSetCursor(GLUT_CURSOR_TOP_LEFT_CORNER);
    m_horiz_on = m_vert_on = true;
  } else if( fabs((m_height-y) - horiz*m_height) < SNAP_DIST)
  {
    glutSetCursor(GLUT_CURSOR_UP_DOWN);
    m_horiz_on = true;
  } else if( fabs(x - vert*m_width) < SNAP_DIST)
  {
    glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
    m_vert_on = true;
  }
  return m_horiz_on || m_vert_on;
}

bool FourView::drag(const int x, const int y)
{
  if(m_horiz_on)
  {
    glutSetCursor(GLUT_CURSOR_UP_DOWN);
    m_horiz = (double)(m_height-y)/(double)m_height;
    m_horiz = std::min(std::max(m_horiz,0.),1.);
    reshape();
  }
  if(m_vert_on)
  {
    if(m_horiz_on)
    {
      glutSetCursor(GLUT_CURSOR_TOP_LEFT_CORNER);
    }else
    {
      glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
    }
    m_vert = (double)x/(double)m_width;
    m_vert = std::min(std::max(m_vert,0.),1.);
    reshape();
  }
  return m_vert_on || m_horiz_on;
}

bool FourView::hover(const int x, const int y)
{
  m_hover_vp = in_viewport_index(x,y);
  const double horiz = clamped_horiz();
  const double vert = clamped_vert();
  if( (fabs((m_height-y) - horiz*m_height) < 2.*SNAP_DIST)
   && (fabs(x - vert*m_width) < 2.*SNAP_DIST))
  {
    glutSetCursor(GLUT_CURSOR_TOP_LEFT_CORNER);
  } else if( fabs((m_height-y) - horiz*m_height) < SNAP_DIST)
  {
    glutSetCursor(GLUT_CURSOR_UP_DOWN);
  } else if( fabs(x - vert*m_width) < SNAP_DIST)
  {
    glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
  } else
  {
    //glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
    glutSetCursor(GLUT_CURSOR_INHERIT);
  }
  return false;
}

bool FourView::up(const int /*x*/, const int /*y*/)
{
  m_horiz_on = m_vert_on = false;
  //glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
  glutSetCursor(GLUT_CURSOR_INHERIT);
  return false;
}

void FourView::add_to_reanttweakbar(igl::ReTwBar & rebar)
{
  using namespace igl;
  // loop over viewports
  for(int vp = 0;vp<NUM_VIEWPORTS;vp++)
  {
    //rebar.TwAddVarRW(C_STR("viewport_"<<vp<<"_zoom"),TW_TYPE_DOUBLE,&m_viewports[vp].camera.zoom,C_STR(
    //  "group='Viewport["<<vp<<"]' "
    //  "label='zoom' "
    //  "step=0.01 min="<<min_zoom<<" max="<<max_zoom<<" "
    //  "help='Zoom viewing camera in and out' "
    //  ));
    rebar.TwAddVarRW(C_STR("viewport_"<<vp<<"_rotation"),TW_TYPE_QUAT4D,
      m_viewports[vp].camera.m_rotation_conj.coeffs().data(),
      C_STR("group='Viewport["<<vp<<"]' "
      "label='rotation' "
      "open "
      "help='Rotate viewing camera (0 0 0 1 = identity)' "
      ));
    //rebar.TwAddVarRW(
    //  C_STR("viewport_"<<vp<<"_translation"),TW_TYPE_DIR3D,
    //  m_viewports[vp].camera.m_translation.data(), "visible=false");
    //rebar.TwAddVarRW(C_STR("viewport_"<<vp<<"_angle"),TW_TYPE_DOUBLE,&m_viewports[vp].camera.angle,
    //  C_STR("group='Viewport["<<vp<<"]' "
    //  "label='angle' "
    //  "help='Perspective viewing angle (45-natural, 0-ortho)' "
    //  "max=90 min=0")
    //  );
    rebar.TwSetParam(C_STR("Viewport["<<vp<<"]"), "opened", TW_PARAM_INT32, 1, &INT_ZERO);
    rebar.TwSetParam(C_STR("Viewport["<<vp<<"]"), "visible", TW_PARAM_INT32, 1, &INT_ZERO);
  }
  // so these are remembered
  rebar.TwAddVarRW("horiz",TW_TYPE_DOUBLE,&m_horiz,"visible=false");
  rebar.TwAddVarRW("vert",TW_TYPE_DOUBLE,&m_vert,"visible=false");
}

void FourView::update_anttweakbar_visibility(igl::ReTwBar & rebar) const
{
  using namespace igl;
  for(int vp = 0;vp<NUM_VIEWPORTS;vp++)
  {
    if(vp == m_down_vp)
    {
      rebar.TwSetParam(C_STR("Viewport["<<vp<<"]"), "visible", TW_PARAM_INT32, 1, &INT_ONE);
    }else 
    {
      rebar.TwSetParam(C_STR("Viewport["<<vp<<"]"), "visible", TW_PARAM_INT32, 1, &INT_ZERO);
    }
  }
}

igl::Camera & FourView::down_camera()
{
  return m_down_camera;
}

AugViewport * FourView::down_viewport()
{
  if(m_down_vp<0)
  {
    return NULL;
  }
  return &m_viewports[m_down_vp];
}

bool FourView::any_rotation_animating() const
{
  for(auto & vp  : m_viewports)
  {
    if(vp.rotation_is_animating)
    {
      return true;
    }
  }
  return false;
}

int FourView::in_viewport_index(const int x, const int _y) const
{
  const int y = m_height - _y;
  int in_vp = -1;
  for(int vp = 0;vp<NUM_VIEWPORTS;vp++)
  {
    if(
      x >= m_viewports[vp].x && 
      y >= m_viewports[vp].y && 
      x <  m_viewports[vp].x+m_viewports[vp].width && 
      y <  m_viewports[vp].y+m_viewports[vp].height)
    {
      in_vp = vp;
      break;
    }
  }
  return in_vp;
}

AugViewport * FourView::in_viewport(const int x, const int y)
{
  const int in_vp = in_viewport_index(x,y);
  if(in_vp<0)
  {
    return NULL;
  }
  return &m_viewports[in_vp];
}

void FourView::snap_to_fixed_up()
{
  using namespace igl;
  for(auto & vp : m_viewports)
  {
    auto & animation_from_quat = vp.animation_from_quat;
    auto & animation_to_quat = vp.animation_to_quat;
    animation_from_quat = vp.camera.m_rotation_conj;
    igl::snap_to_fixed_up(animation_from_quat,animation_to_quat);
    // start animation
    vp.animation_start_time = get_seconds();
    vp.rotation_is_animating = true;
  }
}
