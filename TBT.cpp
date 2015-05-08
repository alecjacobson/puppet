// Classes
#include "TBT.h"
#include "EndCap.h"
#include "Mesh.h"
#include "NodeCallbacks.h"

// Functions
#include "shift_color.h"
#include "draw_guide_triangle.h"
#include "dim_lights.h"
#include "push_string_return_char.h"

#include <igl/readOBJ.h>
#include <igl/per_face_normals.h>
#include <igl/report_gl_error.h>
#include <igl/material_colors.h>
#include <igl/draw_beach_ball.h>
#include <igl/quat_to_mat.h>
#include <igl/get_seconds.h>
#include <igl/STR.h>
#include <igl/C_STR.h>

#include <igl/PI.h>

// Convenience
#include "tinyxmlConvenience.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <numeric>
#include <functional>

#define NUM_AXES 3

// TODO: Is this choice consistent with documentation?
const double TBT::axis[NUM_AXES][3] = {
  {0,0,-1},
  {0,1,0},
  {0,0,-1}
};

const double TBT::biaxis[NUM_AXES][3] = {
  {0,-1,0},
  {-1,0,0},
  {0,-1,0}
};

bool TBT::angle_limits = true;
bool TBT::LEDs_visible = true;
int TBT::shine_tic = 0;
std::vector<GLuint >     TBT::shine_ids;
std::vector<GLuint >     TBT::flare_ids;
bool TBT::textures_initialized = false;


// Leap frog for AntTweakbar
static void tareCB(void *clientData)
{
  TBT * n = static_cast<TBT*>(clientData);
  n->tare();
}

void TBT::initialize_textures()
{
  using namespace igl;
  // Init flares
  lens_flare_load_textures(shine_ids,flare_ids);
  TBT::textures_initialized = true;
  report_gl_error();
}

static std::vector<igl::Flare > LED_flares(
  const float * A,
  const float * B,
  const float * C)
{
  //lens_flare_create(RED,GREEN,BLUE,flares);
  std::vector<igl::Flare> flares(4);
  flares[0] = igl::Flare(-1, 1.0f, 1.*0.1f,  C, 1.0);
  flares[1] = igl::Flare(-1, 1.0f, 1.*0.15f, B, 1.0);
  flares[2] = igl::Flare(-1, 1.0f, 1.*0.35f, A, 1.0);
  flares[3] = igl::Flare( 2, 1.0f, 1.*0.1f, A, 0.4);
  return flares;
}

TBT::TBT()
  :
  // Initialize according to Node super class
  Node(1),
  param_announce_tare(NULL),
  announce_tare(NULL)
{
  // Initialize members
  for(int a = 0;a<3;a++)
  {
    angle[a] = 0;
    angle_at_bind[a] = 0;
    angle_is_hover[a] = 0;
    angle_is_selected[a] = 0;
    angle_is_down[a] = 0;
    angle_drag[a] = 0;
  }
  // Pick on each angle
  pickles.resize(3);
  pickle_hit.resize(3);
  set_child(new EndCap(),0);
  // No lights
  LED_periods[0] = 0;
  LED_periods[1] = 0;
  LED_periods[2] = 0;
}

TBT::TBT(const TBT & that)
  :
  // Initialize according to Node super class
  Node(that),
  param_announce_tare(NULL),
  announce_tare(NULL)
{
  using namespace std;
  for(int a = 0;a<3;a++)
  {
    angle_at_bind[a] = that.angle_at_bind[a];
  }
}

TBT & TBT::operator=(const TBT & that)
{
  using namespace std;
  Node::operator=(that);
  for(int a = 0;a<3;a++)
  {
    angle_at_bind[a] = that.angle_at_bind[a];
  }
  return *this;
}

void TBT::draw_self() const 
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;

  // Draw drag line for rotating angles
  if(is_down && accumulate(angle_drag,angle_drag+3,false,logical_or<bool>()))
  {
    pushmv();
    double du[3],u[3],up[3];
    unproject(down_x,    down_y,du);
    unproject(last_x+(last_x==down_x?1:0),    last_y,u);
    unproject(last_x+(last_x==down_x?1:0),    down_y,up);
    draw_guide_triangle(u,du,up);
    popmv();
  }

  glEnable(GL_NORMALIZE);
  glPushMatrix();
  for(int i = 0;i<NUM_AXES+1;i++)
  {
    double eff_alpha = alpha;
    const float * color = 
      (Node::color_parts ? part_color[TBT_PLUG_PART+i]:this->color.data());
    const int angle_i = i<1?i:i-1;
    pickles[angle_i].activate_pickle_or_diffuse_material(color,eff_alpha);

    // Dim if hovered over
    if(angle_is_hover[angle_i])
    {
      dim_lights(Node::hover_dim);
    }else if (angle_is_selected[angle_i])
    {
      dim_lights(Node::selected_dim);
    }

    part[TBT_PLUG_PART+i].draw_and_cache();

    // Dim if hovered over
    if(angle_is_hover[angle_i])
    {
      dim_lights(1./Node::hover_dim);
    }else if (angle_is_selected[angle_i])
    {
      dim_lights(1./Node::selected_dim);
    }

    const double * eff_angle = (Node::show_bind?angle_at_bind:angle);
    if(i<3)
    {
      glRotated(eff_angle[i],TBT::axis[i][0],TBT::axis[i][1],TBT::axis[i][2]);
    }
  }
  glPopMatrix();

  if(LEDs_visible)
  {
    draw_LEDs();
  }


}

void TBT::draw_LEDs() const
{
  using namespace Eigen;
  using namespace igl;
  using namespace std;
  glPushMatrix();
  const double * eff_angle = (Node::show_bind?angle_at_bind:angle);
  glRotated(eff_angle[0],TBT::axis[0][0],TBT::axis[0][1],TBT::axis[0][2]);
  glRotated(eff_angle[1],TBT::axis[1][0],TBT::axis[1][1],TBT::axis[1][2]);
  glRotated(-90,1,0,0);
  glTranslated(0,0,0.077);
  const double r = 0.01;
  glScaled(r,r,r);
  glDisable(GL_LIGHTING);
  const double d0 = 4.9;
  const double d1 = 5.5;
  const double d2 = d0;
  const Vec3 light_offsets[3] = {
    d0*Vec3(-1.15,-1,0).normalized(),
    d1*Vec3(0,1,0),
    d2*Vec3( 1.15,-1,0).normalized()};
  const Vector4d colors[4] = 
  {
    Vector4d(1.0,0.0,0.0,0.9),
    Vector4d(0.0,1.0,0.0,0.9),
    //Vector4d(81./255.,228./255.,1.0,0.9),
    Vector4d(71./255.,150./255.,1.0,0.9),
    Vector4d(0.1,0.1,0.1,0.3)
  };
  for(int l = 0;l<3;l++)
  {
    Vector4d color = colors[3];
    const double period = LED_periods[l];
    // otherwise Not perceptible and interfers with vsync
    const bool light_on = 
      (LED_periods[l] > 0) &&
      (LED_periods[l] < 30 || fmod(get_seconds()*1000.0,2.0*period) > period);
    if(light_on)
    {
      color = colors[l];
    }

    glPushMatrix();
    glTranslated(
        light_offsets[l](0),
        light_offsets[l](1),
        light_offsets[l](2));

    //if(!light_on)
    {
      glEnable(GL_COLOR_MATERIAL);
      glColorMaterial(GL_FRONT,GL_AMBIENT);
      glColor4dv(color.data());
      glPushMatrix();
      glBegin(GL_TRIANGLE_FAN);
      //if(light_on)
      //{
      //  color[0] = color[1] = color[2] = 0;
      //  color[3] = 1.0;
      //}
      glColor4dv(color.data());
      glVertex3d(0,0,0);
      //color = colors[3];
      glColor4dv(color.data());
      for(double theta = 0; theta<=2.0*PI; theta+=2.0*PI/20)
      {
        glVertex3d(cos(theta),sin(theta),0);
      }
      glEnd();
      glPopMatrix();
    }
    if(light_on)
    {
      if(!TBT::textures_initialized)
      {
        TBT::initialize_textures();
      }
      const float RGB[3][3] = {
        {1,0.1,0.1},
        {0.1,1,0.1},
        {0.1,0.1,1}};
      vector<Flare> flares = LED_flares(RGB[l%3],RGB[(l+2)%3],RGB[(l+1)%3]);
      glScaled(8,8,8);
      glEnable(GL_POLYGON_OFFSET_FILL);
      glPolygonOffset(0,-100.0);
      lens_flare_draw(
        flares,
        TBT::shine_ids,
        TBT::flare_ids,Vector3f(0,0,0),1.0,TBT::shine_tic);
      glDisable(GL_POLYGON_OFFSET_FILL);
    }

    glPopMatrix();
  }
  glEnable(GL_LIGHTING);
  glPopMatrix();
}

// Helper for rigid_to_child and rigid_to_child_at_bind
static void rigid_to_child(
  const double * angle,
  const double radius,
#ifndef NDEBUG
  const int i,
#else
  const int /*i*/,
#endif
  Quat & q, Vec3 & v)
{
  using namespace Eigen;
  using namespace igl;
  assert(i==0);
  q = TBT::TBT_angles_to_quat(angle[0],angle[1],angle[2]);
  v = q*Vec3(0,0,-radius);
}

void TBT::rigid_to_child(const int i, Quat & q, Vec3 & v) const
{
  return ::rigid_to_child(angle,radius,i,q,v);
}

void TBT::rigid_to_child_at_bind(const int i, Quat & q, Vec3 & v) const
{
  return ::rigid_to_child(angle_at_bind,radius,i,q,v);
}

void TBT::bind()
{
  using namespace std;
  // Call base class
  Node::bind();
  copy(angle,angle+3,angle_at_bind);
}

bool TBT::right_down(const int x, const int y, const int /*modifier*/)
{
  using namespace std;
  // Call base class
  Node::right_down(x,y);
  // Turn *off* rotation : We're overriding it
  rotate_on = false;
  translate_on = false;
  copy(angle_is_down,angle_is_down+3,angle_drag);
  return is_down;
}

bool TBT::down(const int x, const int y, const int /*modifier*/)
{
  using namespace std;
  // Call base class
  Node::down(x,y);
  angle_drag[0] = false;
  angle_drag[1] = false;
  angle_drag[2] = false;
  return is_down;
}

bool TBT::drag(const int x, const int y, const int /*modifier*/)
{
  bool any = false;
  if(is_down)
  {
    for(int i = 0;i<3;i++)
    {
      if(angle_drag[i])
      {
        any |= true;
        // interpret drag as change in this angle
        double delta = 360.0*double(x-last_x)/400.0;
        angle[i] += delta;
        if(angle_limits)
        {
          switch(i)
          {
            case 0:
            case 2:
              angle[i] = (angle[i]>170.0 ? 170.0 : angle[i]);
              angle[i] = (angle[i]<-170.0 ? -170.0 : angle[i]);
              break;
            case 1:
              angle[i] = (angle[i]>92.0 ? 92.0 : angle[i]);
              angle[i] = (angle[i]<-92.0 ? -92.0 : angle[i]);
              break;
          }
        }
      }
    }
  }
  if(!any)
  {
    // Call base class
    Node::drag(x,y);
  }
  last_x = x;
  last_y = y;
  return is_down;
}

void TBT::set_is_hover(const bool v)
{
  for(int i = 0;i<3;i++)
  {
    angle_is_hover[i] = v && pickle_hit[i];
  }
  // Call base class
  Node::set_is_hover(v);
}

void TBT::set_is_selected(const bool v)
{
  using namespace std;
  bool was_selected = is_selected;
  for(int i = 0;i<3;i++)
  {
    angle_is_selected[i] = v && pickle_hit[i];
  }
  // Call base class
  Node::set_is_selected(v);
  is_selected = v;
  string prefix = STR("node"<<"_"<<name);
  if(is_selected && !was_selected)
  {
    // Add TBT relevant anttweakbar entries

    // Add entries for each angle
    for(int i = 0;i<3;i++)
    {
      int range = (i==1?100:180);
      TwAddVarRW(
        Node::bar,
        push_string_return_char(tw_vars,C_STR(prefix<<"_angle_"<<i)),
        TW_TYPE_DOUBLE,
        &angle[i],
        C_STR("label='angle["<<i<<"]' "<<
          "group='" << prefix << "' " <<
          "help='Set angle #"<<i<<" of TBT joint' "<<
          "min=-"<<range<<" max="<<range<<" step=1 ")
        );
    }
    TwAddButton(
      Node::bar,
      push_string_return_char(tw_vars,C_STR(prefix<<"_virtual_zero")),
      NodeCallbacks::zeroCB,
      this,
      C_STR("label='virtual_zero' "<<
        "group='" << prefix << "' " <<
        "key='r' "<<
        "help='Set all drawing angles to 0 (will be "
        " immediately replaced by device)' "));
    TwAddButton(
      Node::bar,
      push_string_return_char(tw_vars,C_STR(prefix<<"_tare")),
      ::tareCB,
      this,
      C_STR("label='tare' "<<
        "group='" << prefix << "' " <<
        "key='0' "<<
        "help='Set current angles to be `zero` angles' "));

  }else if(!is_selected && was_selected)
  {
    // Remove TBT relevant anttweakbar entries handled by base call because we
    // push_string_return_char(tw_vars,)
  }
}

void TBT::set_is_down(const bool v)
{
  for(int i = 0;i<3;i++)
  {
    angle_is_down[i] = v && pickle_hit[i];
  }
  // Call base class
  Node::set_is_down(v);
}

tinyxml2::XMLElement * TBT::appendXML( 
  tinyxml2::XMLDocument & doc, 
  tinyxml2::XMLElement * parent) const
{
  // Call base class
  tinyxml2::XMLElement * root = Node::appendXML(doc,parent);
  // TBT specific values
  root->SetAttribute("type","TBT");
  root->InsertFirstChild(
    doc.NewElement("angle_at_bind"))->ToElement()->InsertEndChild(
      doc.NewText(C_STR(
          angle_at_bind[0]<<" "<<angle_at_bind[1]<<" "<<angle_at_bind[2])));
  root->InsertFirstChild(doc.NewElement("angle"))->ToElement()->InsertEndChild(
    doc.NewText(C_STR(angle[0]<<" "<<angle[1]<<" "<<angle[2])));
  return root;
}

void TBT::parseXML(const tinyxml2::XMLElement * root)
{
  using namespace tinyxml2;
  // Call base class
  Node::parseXML(root);
  // Parse TBT specific values
  int ret = -1;
  (void)ret;// So that gcc is happy
  if(const XMLElement *ab = root->FirstChildElement("angle_at_bind"))
  {
    ret = sscanf(ab->GetText(),"%lf %lf %lf",
        &angle_at_bind[0], &angle_at_bind[1], &angle_at_bind[2]);
    assert(ret == 3);
  }
  if(const XMLElement *a = root->FirstChildElement("angle"))
  {
    ret = sscanf(a->GetText(),"%lf %lf %lf",
        &angle[0], &angle[1], &angle[2]);
    assert(ret == 3);
  }
  return;
}

bool TBT::tare()
{
  if(announce_tare)
  {
    announce_tare(param_announce_tare,*this);
    return true;
  }
  return false;
}

Vec3 TBT::quat_to_TBT_angles(const Quat & qcopy)
{
  Quat q = qcopy;
  q.normalize();
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  // TBT use a bizarre convention for euler angles
  Vec3 euler = q.matrix().eulerAngles(2,1,2);
  euler(0) *= -1;
  euler(2) *= -1;
  euler *= 180./PI;
  return euler;
}

Quat TBT::TBT_angles_to_quat(
  const double theta_0,
  const double theta_1,
  const double theta_2)
{
  using namespace Eigen;
  using namespace igl;
  Quat q;
  q = Quat(1,0,0,0);
  q = q*Quat(AngleAxisd(
    2*PI*theta_0/360.0,
    Vec3(TBT::axis[0][0],TBT::axis[0][1],TBT::axis[0][2])));
  q = q*Quat(AngleAxisd(
    2*PI*theta_1/360.0,
    Vec3(TBT::axis[1][0],TBT::axis[1][1],TBT::axis[1][2])));
  q = q*Quat(AngleAxisd(
    2*PI*theta_2/360.0,
    Vec3(TBT::axis[2][0],TBT::axis[2][1],TBT::axis[2][2])));
  return q;
}
