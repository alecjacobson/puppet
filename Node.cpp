// Classes
#include "Node.h"
#include "Mesh.h"
#include "TBT.h"
#include "Splitter.h"
#include "Root.h"
#include "EndCap.h"
#include "throw_at.h"
// Callbacks
#include "NodeCallbacks.h"
// Functions
#include "draw_guide_triangle.h"
#include "draw_infinite_line.h"
#include "unique_name.h"
#include "diffuse_material.h"
#include "draw_number.h"
#include "dim_lights.h"
#include "push_string_return_char.h"
#include "quattrans2affine.h"
#include "list_affine_to_stack.h"

// Macros
#include <igl/STR.h>
#include <igl/C_STR.h>
#include <igl/REDRUM.h>
// Constants
#include <igl/ONE.h>
#include <igl/material_colors.h>
#include <igl/ZERO.h>
#include <igl/PI.h>
// Classes
#include <igl/Timer.h>
// Functions
#include <igl/is_dir.h>
#include <igl/trackball.h>
#include <igl/report_gl_error.h>
#include <igl/quat_to_mat.h>
#include <igl/quat_conjugate.h>
#include <igl/quat_mult.h>
#include <igl/draw_beach_ball.h>
#include <igl/unproject_to_zero_plane.h>
#include <igl/unproject.h>
#include <igl/project.h>
#include <igl/project_to_line.h>
#include <igl/quat_mult.h>
#include <igl/quat_conjugate.h>
#include <igl/mat_to_quat.h>
#include <igl/draw_point.h>
#include <igl/list_to_matrix.h>
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/per_face_normals.h>
#include <igl/view_axis.h>
#include <igl/up_axis.h>
#include <igl/right_axis.h>
#include <igl/matlab_format.h>

// Convenience
#include "tinyxmlConvenience.h"

#ifdef __APPLE__
#  include <OpenGL/gl.h>
#elif defined(_WIN32)
#    define NOMINMAX
#    include <Windows.h>
#    undef NOMINMAX
#else
#  include <GL/gl.h>
#endif

#include <iostream>
#include <cassert>
#include <stdexcept>
#include <unistd.h>

double Node::shift[3],Node::scale = 1.0,Node::user_scale= 1.0;
const double Node::GLOBAL_SCALE = 0.2;
static bool initialized = false;
TwBar * Node::bar;
bool Node::beach_balls = false;
bool Node::drag_along_axis = true;
bool Node::show_bind = false;
bool Node::color_parts = false;
bool Node::visible = true;
bool Node::draw_numbers = false;
bool Node::node_type_tw_initialized = false;
bool Node::show_controls = true;
RotationControlType  Node::rotation_control_type = ROTATION_CONTROL_TYPE_AXIS;
RotationAxisType Node::rotation_axis_type = ROTATION_AXIS_TYPE_VIEW;
TwType Node::NodeTypeTW;
const double Node::hover_dim = 0.5;
const double Node::selected_dim = 0.65;
static Eigen::Vector4f DEFAULT_COLOR(127./255.,159./255.,44./255.,255.0/255.0);
//static float POINT_COLOR[4] = {239./255.,213./255.,46./255.,255.0/255.0};
//static float LINE_COLOR[4] = {106./255.,106./255.,255./255.,255./255.};
static const double ADDITIONAL_SCALE = 0.2;

// Match NodeTypes
const char * node_type_strings[NUM_NODE_TYPES] = 
{
  "Root",
  "TBT",
  "Splitter",
  "EndCap",
  "Unknown"
};

#ifdef __APPLE__
#define NUM_NODE_PARTS_DIRS 4
const char * NODE_PARTS_DIRS[NUM_NODE_PARTS_DIRS] = 
{
  "data/parts/",
  "puppet.app/Contents/Resources/parts/",
  "wackeldackel.app/Contents/Resources/parts/",
  "parts/"
};
#else
const char * NODE_PARTS_DIRS[NUM_NODE_PARTS_DIRS] = 
{
  "data/parts/",
  "puppet.app/Contents/Resources/parts/",
  "wackeldackel.app/Contents/Resources/parts/",
  "parts/"
};
#endif
const static std::string part_path[NUM_NODE_PARTS] =
{
  "plug_III",
  "base_III",
  "follower_III",
  "socket_III",
  "splitter_c", // ONE_TO_FIVE_REGULAR
  "splitter_y",  // ONE_TO_TWO_Y
  "bone_III",
  "wire_f_III",
  "terminator"
};

const float * Node::part_color[NUM_NODE_PARTS] = 
{
  igl::FAST_RED_DIFFUSE,
  igl::FAST_GREEN_DIFFUSE,
  igl::FAST_BLUE_DIFFUSE,
  igl::GOLD_DIFFUSE,
  igl::MIDNIGHT_BLUE_DIFFUSE,
  igl::MIDNIGHT_BLUE_DIFFUSE,
  igl::FAST_GRAY_DIFFUSE,
  igl::FAST_GRAY_DIFFUSE,
  igl::FAST_GRAY_DIFFUSE
};
Mesh Node::part[NUM_NODE_PARTS];


// Short hand for sqrt(2.0)/2.0
#ifndef SQRT_2_OVER_2
#  define SQRT_2_OVER_2 0.707106781f
#endif
#define s2o2 SQRT_2_OVER_2

static void initialize()
{
  using namespace std;
  using namespace igl;
  // Determine where parts are
  {
    string wd = getcwd(NULL,0);
    cout<<"Node: Current working directory: "<<wd<<endl;
  }
  string node_parts_dir = "";
  for(int d = 0;d<NUM_NODE_PARTS_DIRS;d++)
  {
    if(is_dir(NODE_PARTS_DIRS[d]))
    {
      node_parts_dir = NODE_PARTS_DIRS[d];
      continue;
    }else
    {
      cerr<<REDRUM( "Error: "<< NODE_PARTS_DIRS[d] <<" doesn't exist.")<<endl;
    }
  }
  if(!is_dir(node_parts_dir.c_str()))
  {
    throw std::runtime_error("Some nodes/parts directory must exist.");
  }
  // Center of the any node
  Node::shift[0] = 0;
  Node::shift[1] = -11.5;
  Node::shift[2] = 0;
  // Initialize part meshes
  //cout<<"Loading all Node parts..."<<endl; 
  // Don't care about texture coordinates or normals
  ScalarMat _TC,_N;
  IndexMat _FTC;
  bool success = false;
  // Load each part
  for(int i = 0;i<NUM_NODE_PARTS;i++)
  {
    Mesh & part_i = Node::part[i];
    const auto part_path_i = node_parts_dir + part_path[i];
    //success = readOBJ(part_path_i+".obj",part_i.dgetV(),part_i.dgetF());
    success =  readDMAT(part_path_i+".V.dmat",part_i.dgetV());
    success &= readDMAT(part_path_i+".F.dmat",part_i.dgetF());
    if(success)
    {
      //cout<<GREENGIN("Loaded "<<part_path_i<<" successfully.")<<endl;
    }else
    {
      cerr<<REDRUM("Error: Loading "<<part_path_i<<" failed.")<<endl;
      throw std::runtime_error(
        C_STR("TBT::initialize(): "<<part_path_i<<" not loaded"));
    }
    // scale so centered node minus hooks, fits in unit ball
    const double OLIVERS_MAGIC_SCALE = 0.0322;
    // Since model will also be scaled to fit unit ball, scale down even more
    part_i.scale = OLIVERS_MAGIC_SCALE * Node::GLOBAL_SCALE;
    // The models are inconsistently shifted, rotated, (and scaled?)
    switch(i)
    {
      case TERMINATOR_PART:
      {
        // Has wrong orientation and shift
        Quat t(Eigen::AngleAxis<double>(igl::PI*0.723,Vec3(0,0,1)));
        part_i.dgetV()  = part_i.dgetV() * t.matrix();
        //copy(Node::shift,Node::shift+3,part_i.shift);
        //part_i.shift(2) = 28.8;
        break;
      }
      case TBT_FOLLOWER_PART:
      case TBT_BASE_PART:
      {
        // These have wrong orientation
        Quat t(Eigen::AngleAxis<double>(igl::PI,Vec3(0,1,0)));
        part_i.dgetV()  = part_i.dgetV() * t.matrix();
        copy(Node::shift,Node::shift+3,part_i.shift.data());
        break;
      }
      case TBT_PLUG_PART:
      case TBT_SOCKET_PART:
        copy(Node::shift,Node::shift+3,part_i.shift.data());
        break;
      case SPLITTER_C_PART:
      {
        // Splitter model is wrong orientation
        Quat q(-s2o2,s2o2,0,0);
        part_i.dgetV()  = part_i.dgetV() * q.matrix();
        // Splitter is not offset like the others are
        //part_i.dgetV().col(1).array() += 11.5;
        break;
      }
      case SPLITTER_Y_PART:
      {
        // Splitter model is wrong orientation
        Quat q(-s2o2,s2o2,0,0);
        part_i.dgetV()  = part_i.dgetV() * q.matrix();
        Quat t(Eigen::AngleAxis<double>(0.06,Vec3(0,0,1)));
        part_i.dgetV()  = part_i.dgetV() * t.matrix();
        part_i.dgetV().col(0).array() += 0.6;
        part_i.dgetV().col(1).array() += 0.6;
        part_i.dgetV().col(2).array() += 4.8;
        break;
      }
      case WIRE_SOCKET_PART:
      {
        // Root model is wrong orientation
        Quat q(0.0,0.0,-1.0,0.0);
        part_i.dgetV()  = part_i.dgetV() * q.matrix();
        part_i.shift(1) = 0.0;
        part_i.shift(2) = -6.4;
        break;
      }
    }
    // Compute normals after all this adjustment
    part_i.dget_normal_type() = PER_FACE_NORMALS;
    per_face_normals(part_i.dgetV(),part_i.dgetF(),part_i.dgetFN());
  }
}

Node::Node(const int max_children):
  alpha(1.0),
  radius(1.*GLOBAL_SCALE),
  r_off(1,0,0,0),
  t_off(0,0,-radius),
  ct_off(NULL),
  color(DEFAULT_COLOR),
  pickles(),
  pickle_hit(),
  param_announce_is_selected_change(NULL),
  announce_is_selected_change(NULL),
  drag_arrow(),
  parent(NULL),
  parent_id(-1),
  child(NULL),
  max_children(max_children),
  is_hover(false),
  is_selected(false),
  is_down(false),
  tw_vars(),
  name(unique_name()),
  down_x(-1),
  down_y(-1),
  down_z(-1),
  last_x(-1),
  last_y(-1),
  down_r_off(1,0,0,0),
  down_t_off(0,0,0),
  rotate_on(false),
  translate_on(false),
  new_node_type(NODE_TYPE_TBT),
  new_node_cid(0),
  new_node_max_children(0)
{
  using namespace std;
  child = new Node*[max_children];
  // Initialize children pointers to null
  for(int i = 0;i<max_children;i++)
  {
    child[i] = NULL;
  }
  // Initialize children offsets to (0,0,0)
  ct_off = new Vec3[max_children];
  for(int c = 0;c<max_children;c++)
  {
    ct_off[c] = Vec3(0,0,0);
  }
  //// Initialize children pointers to EndCaps
  //for(int i = 0;i<max_children;i++)
  //{
  //  set_child(new EndCap(),i);
  //}

  if(!initialized)
  {
    initialize();
    initialized = true;
  }
}

Node::Node(const Node & that):
  alpha(that.alpha),
  radius(that.radius),
  r_off(that.r_off),
  t_off(that.t_off),
  color(DEFAULT_COLOR),
  ct_off(NULL),
  pickles(),
  pickle_hit(),
  param_announce_is_selected_change(NULL),
  announce_is_selected_change(NULL),
  drag_arrow(that.drag_arrow),
  parent(NULL),
  parent_id(-1),
  child(NULL),
  max_children(that.max_children),
  is_hover(false),
  is_selected(false),
  is_down(false),
  tw_vars(),
  name(unique_name()),
  down_x(-1),
  down_y(-1),
  down_z(-1),
  last_x(-1),
  last_y(-1),
  down_r_off(1,0,0,0),
  down_t_off(0,0,0),
  rotate_on(false),
  translate_on(false),
  new_node_type(NODE_TYPE_TBT),
  new_node_cid(0),
  new_node_max_children(0)
{
  using namespace std;
  // If really copying then one should worry about how to deal with children
  // and parent
  // Initialize children pointers to null
  child = new Node*[max_children];
  ct_off = new Vec3[max_children];
  // Copy any EndCaps because they won't be saved by PuppetReader
  for(int i = 0;i<max_children;i++)
  {
    child[i] = NULL;
    if(dynamic_cast<const EndCap * >(that.get_child(i)))
    {
      child[i] = new EndCap();
      *child[i] = *that.get_child(i);
    }
    ct_off[i] = that.ct_off[i];
  }
  if(!initialized)
  {
    initialize();
    initialized = false;
  }
}

Node & Node::operator=(const Node& that)
{
  using namespace std;
  alpha = that.alpha;
  radius = that.radius;
  r_off = that.r_off;
  t_off = that.t_off;
  assert(max_children == that.get_max_children());
  // copy children offsets
  for(int i = 0;i<that.get_max_children();i++)
  {
    // Replace any empty slots or EndCaps with that's EndCaps
    if(
      (child[i] == NULL || dynamic_cast<const EndCap * >(child[i])) && 
      dynamic_cast<const EndCap * >(that.get_child(i)))
    {
      delete child[i];
      child[i] = new EndCap();
      *child[i] = *that.get_child(i);
    }
    ct_off[i] = that.ct_off[i];
  }
  drag_arrow = that.drag_arrow;
  return *this;
}

Node::~Node()
{
  using namespace std;
  // Remove self from parent's child list
  if(parent != NULL)
  {
    parent->remove_child(parent_id);
  }
  // Delete any children
  for(int i = 0;i<max_children;i++)
  {
    delete child[i];
  }
  delete[] child;
  delete[] ct_off;
  // Make sure to deselect so that anttweakbar gets cleaned up
  set_is_selected(false);
}

void Node::push() const
{
  using namespace igl;
  glPushMatrix();
  // if parent exist then also translate according to its child offset
  if(parent != NULL)
  {
    const Vec3 & pt_off = parent->ct_off[parent_id];
    glTranslated(pt_off[0],pt_off[1],pt_off[2]);
  }
  glTranslated(t_off[0],t_off[1],t_off[2]);
  double mat[4*4];
  quat_to_mat(r_off.coeffs().data(),mat);
  glMultMatrixd(mat);
}

void Node::pop() const
{
  glPopMatrix();
}

void Node::pushmv() const
{
  using namespace Eigen;
  Quat pq(1,0,0,0);
  Vec3 pt(0,0,0);
  Quat pqc(1,0,0,0);
  Vec3 ptc(0,0,0);
  if(NULL!=parent)
  {
    if(Node::show_bind)
    {
      parent->rigid_to_self_at_bind(pq,pt);
      parent->rigid_to_child_at_bind(parent_id,pqc,ptc);
    }else
    {
      parent->rigid_to_self(pq,pt);
      parent->rigid_to_child(parent_id,pqc,ptc);
    }
  }
  glPushMatrix();
  glMultMatrixd(Affine3d(Translation3d(pt)).matrix().data());
  glMultMatrixd(Affine3d(pq).matrix().data());
  // Now origin is at center of interface with parent
  glMultMatrixd(Affine3d(Translation3d(ptc)).matrix().data());
  glMultMatrixd(Affine3d(pqc).matrix().data());
}

void Node::popmv() const
{
  glPopMatrix();
}

Vec3 Node::origin() const
{
  using namespace Eigen;
  using namespace std;
  Quat q;
  Vec3 t;
  rigid_to_self(q,t);
  return t;
}

Vec3 Node::origin_at_bind() const
{
  using namespace Eigen;
  using namespace std;
  Quat q;
  Vec3 t;
  rigid_to_self_at_bind(q,t);
  return t;
}

void Node::unproject( const int x, const int y, double * u) const
{
  using namespace igl;
  // down_t_off projected to screen to get depth value
  double p[3];
  igl::project(down_t_off(0),down_t_off(1),down_t_off(2),&p[0],&p[1],&p[2]);
  // unprojected x,y with down_t_off depth
  igl::unproject(x,y,p[2],&u[0], &u[1], &u[2]);
}

void Node::unproject_to_screen_z( const int x, const int y, double * u) const
{
  using namespace igl;
  // orthogonal projection in screen space
  // screen space origin
  double origin[3];
  igl::project(0,0,0,&origin[0],&origin[1],&origin[2]);
  double zaxis[3];
  igl::project(0,0,1,&zaxis[0],&zaxis[1],&zaxis[2]);
  zaxis[0] -= origin[0];
  zaxis[1] -= origin[1];
  zaxis[2] -= origin[2];
  double t, sqrd;
  project_to_line(
    double(x),double(y),0.0,
    origin[0],origin[1],0.0,
    origin[0]+zaxis[0],origin[1]+zaxis[1],0.0,
    t,sqrd);
  double sp[3];
  sp[0] = origin[0]+t*zaxis[0];
  sp[1] = origin[1]+t*zaxis[1];
  sp[2] = origin[2]+t*zaxis[2];
  igl::unproject(sp[0],sp[1],sp[2],&u[0],&u[1],&u[2]);
}

void Node::draw() const
{
  using namespace Eigen;
  using namespace std;
  using namespace igl;


  Vec3 tt_off = t_off;
  if(NULL!=parent)
  {
    tt_off += parent->ct_off[parent_id];
  }
  pushmv();
  glPushMatrix();
  glMultMatrixd(Affine3d(Translation3d(tt_off)).matrix().data());
  glMultMatrixd(Affine3d(r_off).matrix().data());

  // Now origin is at center of node and rotated to meet node's frame

  // Keep track of mv before drawing pieces so we can draw axis on top

  // Dim if hovered over
  if(is_hover)
  {
    dim_lights(hover_dim);
  }else if (is_selected)
  {
    dim_lights(selected_dim);
  }

  glPushMatrix();
  glScaled(Node::scale,Node::scale,Node::scale);
  if(Node::visible)
  {
    draw_self();
  }
  glPopMatrix();

  // Reset lighting
  if(is_hover)
  {
    dim_lights(1./Node::hover_dim);
  }else if (is_selected)
  {
    dim_lights(1./Node::selected_dim);
  }

  // Now frame is rotated to meet children
  
  // Go back and draw axis
  if(rotate_on && !(dynamic_cast<const Splitter*>(this)) && show_controls)
  {
    draw_rotation_controls();
  }

  if(is_down && drag_along_axis && parent && translate_on && show_controls)
  {
    draw_translation_controls();
  }

  for(int i = 0;i<max_children;i++)
  {
    glPushMatrix();
    Quat q;
    Vec3 v;
    if(Node::show_bind)
    {
      rigid_to_child_at_bind(i,q,v);
    }else
    {
      rigid_to_child(i,q,v);
    }
    glTranslatef(v(0),v(1),v(2));
    glMultMatrixd(Affine3d(q).data());

    // Now frame is rotated to meet child and translated to interface with
    // child
    if(child[i])
    {
      if(Node::visible)
      {
        // Draw bone cylinder to child
        glPushMatrix();
        const float * color = Node::part_color[BONE_PART];
        diffuse_material(color,alpha);
        glRotated(180.0,1,0,0);
        const double r = Node::GLOBAL_SCALE * Node::scale * 0.34;
        glTranslated(0.0,0.0,(Node::scale*v.norm() - v.norm()));
        // Assumes that ct_off is only interesting in z coordinate
        const double h = -ct_off[i](2) - 
          (Node::scale*v.norm() - v.norm()) - 
          (Node::scale*child[i]->t_off.norm() - child[i]->t_off.norm());
        glBegin(GL_TRIANGLE_STRIP);
        for(double theta = 0;theta<=2.*PI;theta+=2.*PI/20.)
        {
          const double off = sin(2.0*theta) * 3e-3;
          glNormal3d(r*cos(theta),r*sin(theta),0);
          glVertex3d(r*cos(theta),r*sin(theta),h+off);
          glNormal3d(r*cos(theta),r*sin(theta),0);
          glVertex3d(r*cos(theta),r*sin(theta),0+off);
        }
        glEnd();
        glPopMatrix();
      }
    }

    if(draw_numbers)
    {
      glColor4f(0,0,0,1);
      glEnable(GL_POLYGON_OFFSET_FILL);
      glPolygonOffset(0,-100000);
      draw_number(i);
      glDisable(GL_POLYGON_OFFSET_FILL);
    }
    glPopMatrix();
  }

  glPopMatrix();
  popmv();
  for(int i = 0;i<max_children;i++)
  {
    if(child[i])
    {
      child[i]->draw();
    }
  }
  report_gl_error();
}

void Node::draw_translation_controls() const
{
  // Assumes currently matrix stack is [...;MV;AX]
  double old_mv[16];
  glGetDoublev(GL_MODELVIEW_MATRIX,old_mv);
  glPopMatrix();
  // Project mouse to space
  double du[3],u[3],up[3];
  unproject(down_x,down_y,du);
  unproject(last_x,last_y,u);
  // Project onto axis
  du[0] = 0;
  du[1] = 0;
  up[0] = 0;
  up[1] = 0;
  up[2] = u[2];
  unproject_to_screen_z(last_x,last_y,up);
  draw_guide_triangle(u,du,up);
  glPushMatrix();
  glLoadIdentity();
  glMultMatrixd(old_mv);
}

void Node::draw_rotation_controls() const
{
  using namespace igl;
  switch(rotation_control_type)
  {
    case ROTATION_CONTROL_TYPE_TRACKBALL:
    {
      if(beach_balls)
      {
        glPushMatrix();
        glScaled(Node::scale,Node::scale,Node::scale);
        glScaled(0.4,0.4,0.4);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(0,-100000);
        draw_beach_ball();
        glDisable(GL_POLYGON_OFFSET_FILL);
        glPopMatrix();
      }
      break;
    }
    case ROTATION_CONTROL_TYPE_AXIS:
    {
      // Assumes currently matrix stack is [...;MV;AX]
      double old_mv[16];
      glGetDoublev(GL_MODELVIEW_MATRIX,old_mv);
      glPopMatrix();
      double mv[16];
      glGetDoublev(GL_MODELVIEW_MATRIX,mv);
      // Project mouse to space
      double down[3],last[3],last_x_only[3];
      unproject(down_x,down_y,down);
      unproject(last_x,last_y,last);
      unproject(last_x,down_y,last_x_only);
      draw_guide_triangle(last,down,last_x_only);
      glPushMatrix();
      glLoadIdentity();
      glMultMatrixd(old_mv);

      Vec3 axis;
      view_axis(&axis[0],&axis[1],&axis[2]);
      switch(rotation_axis_type)
      {
        case ROTATION_AXIS_TYPE_VIEW:
          view_axis(mv,&axis[0],&axis[1],&axis[2]);
          break;
        case ROTATION_AXIS_TYPE_UP:
          up_axis(mv,&axis[0],&axis[1],&axis[2]);
          break;
        case ROTATION_AXIS_TYPE_RIGHT:
          right_axis(mv,&axis[0],&axis[1],&axis[2]);
          break;
        default:{}
      }
      {
        axis = (r_off.conjugate().toRotationMatrix() * axis).eval();
        draw_infinite_line(&axis[0]);
        if(ROTATION_AXIS_TYPE_VIEW == rotation_axis_type)
        {
          glColor3fv(INFINITE_LINE_COLOR);
          int old_depth_test=0;
          glGetIntegerv(GL_DEPTH_TEST,&old_depth_test);
          glDisable(GL_DEPTH_TEST);
          draw_point(0,0,0,4,false);
          (old_depth_test ? glEnable(GL_DEPTH_TEST) : glDisable(GL_DEPTH_TEST));
        }
      }
      break;
    }
    default:{}
  }
}

void Node::rigid_to_self(Quat & q,Vec3 & t) const
{
  Quat pq(1,0,0,0);
  Vec3 pt(0,0,0);
  Quat pqc(1,0,0,0);
  Vec3 ptc(0,0,0);
  Vec3 tt_off = t_off;
  if(NULL!=parent)
  {
    parent->rigid_to_self(pq,pt);
    parent->rigid_to_child(parent_id,pqc,ptc);
    tt_off += parent->ct_off[parent_id];
  }
  q = pq*pqc*r_off;
  t = pq*pqc*tt_off+pq*ptc+pt;
}

void Node::rigid_to_self_at_bind(Quat & q,Vec3 & t) const
{
  Quat pq(1,0,0,0);
  Vec3 pt(0,0,0);
  Quat pqc(1,0,0,0);
  Vec3 ptc(0,0,0);
  Vec3 tt_off = t_off;
  if(NULL!=parent)
  {
    parent->rigid_to_self_at_bind(pq,pt);
    parent->rigid_to_child_at_bind(parent_id,pqc,ptc);
    tt_off += parent->ct_off[parent_id];
  }
  q = pq*pqc*r_off;
  t = pq*pqc*tt_off+pq*ptc+pt;
}

Node * Node::set_child(Node * c, const int cid)
{
  using namespace std;
  assert(cid>=0);
  assert(cid<max_children);
  // something weird might be going on if child[cid] == c


  // If new child points to instanciated Node
  if(c != NULL)
  {
    // If new child's parent points to instanciated Node
    if(c->parent!= NULL)
    {
      assert(c->parent->child[c->parent_id] == c);
      // Tell new child's parent it has lost a child
      c->parent->remove_child(c->parent_id);
    }
    // Tell new child its new parent
    c->parent = this;
    c->parent_id = cid;
  }

  // Keep track of previous child at cid
  Node * prev = child[cid];

  // If previous child points to instanciated Node
  if(prev != NULL)
  {
    const DragArrow prev_drag_arrow = prev->drag_arrow;
    if(prev_drag_arrow.activated)
    {
      if(c != NULL)
      {
        c->drag_arrow = prev_drag_arrow;
      }
    }
    // Tell prev it's lost its parent
    prev->parent = NULL;
    prev->parent_id = NULL_PARENT_ID;
  }

  // Set new child at cid
  child[cid] = c;
  // Return pointer to previous node
  return prev;
}

Node * Node::remove_child(const int cid)
{
  return set_child(NULL,cid);
}

Node * Node::get_child(const int cid) const 
{
  assert(cid<max_children);
  return child[cid];
}

const Node * Node::get_parent() const
{
  return parent;
}

int Node::get_parent_id() const
{
  return parent_id;
}

std::vector<Node*> Node::get_children() const
{
  using namespace std;
  vector<Node*> v;
  for(int i = 0;i<max_children;i++)
  {
    if(child[i] != NULL)
    {
      v.push_back(child[i]);
    }
  }
  return v;
}

std::vector<const Node*> Node::get_const_children() const
{
  using namespace std;
  vector<const Node*> v;
  for(int i = 0;i<max_children;i++)
  {
    if(child[i] != NULL)
    {
      v.push_back(child[i]);
    }
  }
  return v;
}

int Node::get_max_children() const
{
  return max_children;
}

GLuint Node::get_name() const
{
  return name;
}

bool Node::right_down(const int x, const int y, const int /*modifier*/)
{
  using namespace std;
  // Keep track of state on down
  down_x = x;
  down_y = y;
  last_x = x;
  last_y = y;
  down_r_off = r_off;
  Vec3 tt_off = t_off;
  if(parent != NULL)
  {
    tt_off += parent->ct_off[parent_id];
  }
  down_t_off = tt_off;
  // Don't turn on rotation (limit to certain joint types which will handle
  // this with an overload)
  //rotate_on = is_down;
  translate_on = false;
  return is_down;
}

bool Node::down(const int x, const int y, const int /*modifier*/)
{
  using namespace std;
  // Keep track of state on down
  down_x = x;
  down_y = y;
  last_x = x;
  last_y = y;
  down_r_off = r_off;
  Vec3 tt_off = t_off;
  if(parent != NULL)
  {
    tt_off += parent->ct_off[parent_id];
  }
  down_t_off = tt_off;
  rotate_on = false;
  translate_on = is_down;
  return is_down;
}

bool Node::drag(const int x, const int y, const int /*modifier*/)
{
  using namespace igl;
  using namespace std;
  // No drag action if not down
  if(!is_down)
  {
    return false;
  }
  if(rotate_on)
  {
    switch(rotation_control_type)
    {
      case ROTATION_CONTROL_TYPE_TRACKBALL:
      {
        // Pick up origin location
        pushmv();
        double mv[16];
        glGetDoublev(GL_MODELVIEW_MATRIX,mv);
        push();
        double origin[3];
        igl::project(0,0,0,&origin[0],&origin[1],&origin[2]);
        pop();
        popmv();
        double rot[4];
        int VP[4];
        glGetIntegerv(GL_VIEWPORT,VP);
        trackball<double>(
          VP[2],
          VP[3],
          2,
                (down_x-origin[0]+VP[2]/2),
          VP[3]-(down_y-origin[1]+VP[3]/2),
                (     x-origin[0]+VP[2]/2),
          VP[3]-(     y-origin[1]+VP[3]/2),
          rot);
        {
          // We've computed change in rotation according to this view:
          // R = mv * r, R' = rot * (mv * r)
          // But we only want new value for r:
          // R' = mv * r'
          // mv * r' = rot * (mv * r)
          // r' = mv* * rot * sr * r
          // Convert modelview matrix to quaternion
          // Normalize to unit quaternion (rotation only)
          double sr_conj[4],scene_rot[4],t1[4],t2[4];

          // remove scale from mv
          double s = 0;
          for(int i=0;i<3;i++)
          {
            for(int j=0;j<3;j++)
            {
              s += (mv[j*4+i])*(mv[j*4+i]);
            }
          }
          double mvs[16];
          for(int i=0;i<4;i++)
          {
            for(int j=0;j<4;j++)
            {
              mvs[j*4+i] = mv[j*4+i]/sqrt(s/3.0);
            }
          }
          mat4_to_quat(mvs,scene_rot);
          double len = 
            sqrt(scene_rot[0]*scene_rot[0]+
            scene_rot[1]*scene_rot[1]+
            scene_rot[2]*scene_rot[2]+
            scene_rot[3]*scene_rot[3]);
          for(int j = 0;j<4;j++)
          {
            scene_rot[j] /= len;
          }
          quat_conjugate(scene_rot,sr_conj);
          quat_mult<double>(sr_conj,rot,t1);
          quat_mult<double>(t1,scene_rot,t2);
          quat_mult<double>(t2,&down_r_off.x(),&r_off.x());
        }
        break;
      }
      case ROTATION_CONTROL_TYPE_AXIS:
      {
        pushmv();
        double mv[16];
        glGetDoublev(GL_MODELVIEW_MATRIX,mv);
        popmv();
        Vec3 axis;
        view_axis(&axis[0],&axis[1],&axis[2]);
        switch(rotation_axis_type)
        {
          case ROTATION_AXIS_TYPE_VIEW:
            view_axis(mv,&axis[0],&axis[1],&axis[2]);
            break;
          case ROTATION_AXIS_TYPE_UP:
            up_axis(mv,&axis[0],&axis[1],&axis[2]);
            break;
          case ROTATION_AXIS_TYPE_RIGHT:
            right_axis(mv,&axis[0],&axis[1],&axis[2]);
            break;
          default:{}
        }
        // Normalize the axis
        axis.normalize();
        double angle = -2.0*PI*360.0*( (x-last_x)/ 400.0)/360.0;
        // Get change in rotation as quaternion
        Quat delta_rot = Quat(Eigen::AngleAxis<double>(angle,axis)) ;
        r_off = delta_rot*r_off;
        break;
      }
      default:{}
    }
  }
  if(translate_on)
  {
    // We want that origin follows mouse move. First define plane we
    // projecteing screen mouse movement to as perpendicular plane passing
    // through this origin.
    pushmv();
    double du[3],u[3];
    unproject(down_x,down_y,du);
    unproject(x,y,u);
    // Dont' ever restrict roots
    if(drag_along_axis && parent)
    {
      // Project projected click onto bone axis (z-axis)
      du[0] = 0;
      du[1] = 0;
      unproject_to_screen_z(x,y,u);
    }

    // Then move this origin according to project mouse displacment
    Vec3 tt_off;
    tt_off(0) = down_t_off(0) + (u[0]-du[0]);
    tt_off(1) = down_t_off(1) + (u[1]-du[1]);
    tt_off(2) = down_t_off(2) + (u[2]-du[2]);
    if(parent == NULL)
    {
      t_off = tt_off;
    }else
    {
      // stretch should always be positive so tt_off should be negative
      //tt_off.array() = tt_off.array().min(0);
      // Assumes stretch is only along z-axis
      const int min_z = (t_off(2) + radius + parent->radius);
      tt_off(2) = (tt_off(2)<min_z?tt_off(2):min_z);
      parent->ct_off[parent_id] = tt_off - t_off;
    }
    popmv();
  }

  last_x = x;
  last_y = y;
  return is_down;
}

bool Node::up(const int /*x*/, const int /*y*/, const int /*modifier*/)
{
  rotate_on = false;
  translate_on = false;
  return is_down;
}

void Node::set_is_hover(const bool v)
{
  is_hover = v;
}

void Node::set_is_selected(const bool v)
{
  using namespace std;
  using namespace NodeCallbacks;
  using namespace igl;
  bool was_selected = is_selected;
  is_selected = v;
  // Build unique prefix
  string prefix = STR("node"<<"_"<<name);

  if(Node::bar)
  {
    if(is_selected && !was_selected)
    {
      // Use callbacks to adjust for scene rotation
      // Add anttweakbar entries
      TwAddVarRW(
        Node::bar,
        push_string_return_char(tw_vars,C_STR(prefix<<"_rot")),
        TW_TYPE_QUAT4D,
        &r_off.x(),
        C_STR("label='rot' " <<
        "group='" << prefix << "' " <<
        "help='Offset rotation of node' "<<
        "open")
        );

      TwAddButton(
        Node::bar,
        push_string_return_char(tw_vars,C_STR(prefix<<"_snap_r_off")),
        snapCB,
        this,
        C_STR("label='snap' " <<
        "key='G' "<<
        "group='" << prefix << "' " <<
        "help='Snap offset rotation of node to canonical rotation' "));

      for(int i = 0;i<3;i++)
      {
        TwAddVarRW(
          Node::bar,
          push_string_return_char(tw_vars,C_STR(prefix<<"_trans_"<<i)),
          TW_TYPE_DOUBLE,
          &t_off[i],
          C_STR("label='t["<<i<<"]' " <<
          "group='" << prefix << "' " <<
          "min=-10 max=10 step=0.1 "<<
          "help='Offset "<<i<<" componenet of translation of node' ")
          );
      }

      if(parent != NULL)
      {
        for(int i = 0;i<3;i++)
        {
          TwAddVarRW(
            Node::bar,
            push_string_return_char(tw_vars,C_STR(prefix<<"_ptrans_"<<i)),
            TW_TYPE_DOUBLE,
            &parent->ct_off[parent_id][i],
            C_STR("label='pt["<<i<<"]' " <<
            "group='" << prefix << "' " <<
            "min=-10 max=10 step=0.1 "<<
            "help='Offset "<<i<<" componenet of translation of node at parent' ")
            );
        }
      }

      TwAddVarRW(
        Node::bar,
        push_string_return_char(tw_vars,C_STR(prefix<<"_radius")),
        TW_TYPE_DOUBLE,
        &radius,
        C_STR("label='radius' " <<
        "group='" << prefix << "' " <<
        "min=-10 max=10 step=0.01 "<<
        "help='radius of node part in rest state' ")
        );

      if(this->parent)
      {
        TwAddButton(
          Node::bar,
          push_string_return_char(tw_vars,C_STR(prefix<<"_delete")),
          deleteCB,
          this,
          C_STR("label='delete' " <<
          "key='backspace' "<<
          "group='" << prefix << "' " <<
          "help='Delete this node and all its children' ")
          );
      }

      if(!Node::node_type_tw_initialized)
      {
        const int NUM_CHOOSABLE_NODE_TYPES = 3;
        TwEnumVal NodeTypeEV[NUM_CHOOSABLE_NODE_TYPES] = 
        {
          {NODE_TYPE_TBT,node_type_strings[NODE_TYPE_TBT]},
          {NODE_TYPE_SPLITTER,node_type_strings[NODE_TYPE_SPLITTER]},
          {NODE_TYPE_ENDCAP,node_type_strings[NODE_TYPE_ENDCAP]}
        };
        Node::NodeTypeTW = 
          TwDefineEnum(
            "NodeType", 
            NodeTypeEV, 
            NUM_CHOOSABLE_NODE_TYPES);
        Node::node_type_tw_initialized = true;
      }
      TwAddButton(
        Node::bar,
        push_string_return_char(tw_vars,C_STR(prefix<<"_reset_offsets")),
        reset_offsetsCB,
        this,
        C_STR("label='Reset' " <<
          "group='" << prefix << "' " <<
          "help='Reset state of this node.' ")
        );
      if(max_children>0)
      {
        TwAddSeparator(
          Node::bar,
          push_string_return_char(tw_vars,C_STR(prefix<<"_sep1")),
          C_STR("group='" << prefix << "' "));
        TwAddVarRW(
          Node::bar,
          push_string_return_char(tw_vars,C_STR(prefix<<"_new_node_cid")),
          TW_TYPE_INT32,
          &new_node_cid,
          C_STR("label='Child id' " <<
          "group='" << prefix << "' " <<
          "min=0 max="<<(max_children-1)<<" "<<
          "help='choose id of child' "));
        TwAddVarRW(
          Node::bar,
          push_string_return_char(tw_vars,C_STR(prefix<<"_new_node_type")),
          Node::NodeTypeTW,
          &new_node_type,
          C_STR("label='Type' " <<
          "group='" << prefix << "' " <<
          "help='choose type of child' "));
        TwAddVarRW(
          Node::bar,
          push_string_return_char(tw_vars,C_STR(prefix<<"_new_node_max_children")),
          TW_TYPE_INT32,
          &new_node_max_children,
          C_STR("label='Max children' " <<
          "group='" << prefix << "' " <<
          "min=0 "<<
          "help='choose num children of child' "));
        TwAddButton(
          Node::bar,
          push_string_return_char(tw_vars,C_STR(prefix<<"_new_node")),
          new_childCB,
          this,
          C_STR("label='New node' " <<
          "key='N' "<<
          "group='" << prefix << "' " <<
          "help='Delete this child and create new node' ")
          );
        TwAddSeparator(
          Node::bar,
          push_string_return_char(tw_vars,C_STR(prefix<<"_sep2")),
          C_STR("group='" << prefix << "' "));
      }


    }else if(!is_selected && was_selected)
    {
      // Remove anttweakbar entries
      for(
        vector<string>::iterator vit = tw_vars.begin();
        vit<tw_vars.end();
        vit++)
      {
        TwRemoveVar(Node::bar,vit->c_str());
      }
      tw_vars.clear();
    }
  }
  if(announce_is_selected_change && was_selected != is_selected)
  {
   announce_is_selected_change(param_announce_is_selected_change,*this); 
  }
}

void Node::set_is_down(const bool v)
{
  is_down = v;
}

bool Node::get_is_hover() const
{
  return is_hover;
}

bool Node::get_is_selected() const
{
  return is_selected;
}

bool Node::get_is_down() const
{
  return is_down;
}

bool Node::get_translate_on() const
{
  return translate_on;
}

#define NODE_DOM "Node"
#define NODETREE_DOM "NodeTree"

tinyxml2::XMLElement * Node::appendXML(
  tinyxml2::XMLDocument & doc, 
  tinyxml2::XMLElement * parent) const
{
  using namespace tinyxml2;
  using namespace std;
  assert(parent);
  // Create new node for this element
  XMLElement * root = 
    parent->InsertEndChild(doc.NewElement(NODE_DOM))->ToElement();
  if (!root)
  {
    return NULL;
  }
  // Set max_children as attribute
  root->SetAttribute("max_children",max_children);
  // Set basic info
  //root->InsertEndChild(doc.NewElement("alpha"))->ToElement()->InsertEndChild(
  //  doc.NewText(C_STR(alpha)));
  root->InsertEndChild(doc.NewElement("radius"))->ToElement()->InsertEndChild(
    doc.NewText(C_STR(radius)));
  root->InsertEndChild(doc.NewElement("r_off"))->ToElement()->InsertEndChild(
    doc.NewText(C_STR(r_off.vec().transpose()<<" "<<r_off.w())));
  root->InsertEndChild(doc.NewElement("t_off"))->ToElement()->InsertEndChild(
    doc.NewText(C_STR(t_off.transpose())));
  root->InsertEndChild(doc.NewElement("drag_arrow_from"))->ToElement()->InsertEndChild(
    doc.NewText(C_STR(drag_arrow.from.transpose())));
  if(drag_arrow.activated)
  {
    root->InsertEndChild(doc.NewElement("drag_arrow_to"))->ToElement()->InsertEndChild(
      doc.NewText(C_STR(drag_arrow.to.transpose())));
  }
  root->InsertEndChild(doc.NewElement("color"))->ToElement()->InsertEndChild(
    doc.NewText(C_STR(color(0)<<" "<<color(1)<<" "<<color(2)<<" "<<color(3))));
  // Add children
  for(int i = 0;i<max_children;i++)
  {
    root->InsertEndChild(doc.NewElement("ct_off"))->ToElement()->
      InsertEndChild( doc.NewText(C_STR(ct_off[i].transpose())));
    if(!child[i])
    {
      continue;
    }
    // append xml of child
    XMLElement * child_i = child[i]->appendXML(doc,root);
    // add branch id to child as attribute
    child_i->SetAttribute("parent_id",i);
  }
  return root;
}

void Node::parseXML(const tinyxml2::XMLElement * root)
{
  using namespace tinyxml2;
  assert(root);
  // Get basic info
  int ret = -1;
  //if(const XMLElement * h = root->FirstChildElement("alpha"))
  //{
  //  ret = sscanf(h->GetText(),"%lf",&alpha);
  //  assert(ret == 1);
  //}
  if(const XMLElement *h = root->FirstChildElement("radius"))
  {
    ret = sscanf(h->GetText(),"%lf",&radius);
    assert(ret == 1);
  }
  if(const XMLElement *h = root->FirstChildElement("r_off"))
  {
    ret = sscanf(h->GetText(),"%lf %lf %lf %lf",
      &r_off.coeffs().data()[0],
      &r_off.coeffs().data()[1],
      &r_off.coeffs().data()[2],
      &r_off.coeffs().data()[3]);
    assert(ret == 4);
  }
  if(const XMLElement *h = root->FirstChildElement("t_off"))
  {
    ret = sscanf(h->GetText(),"%lf %lf %lf",
      &t_off.data()[0], &t_off.data()[1], &t_off.data()[2]);
    assert(ret == 3);
  }
  if(const XMLElement *h = root->FirstChildElement("drag_arrow_from"))
  {
    Vec3 & from = drag_arrow.from;
    ret = sscanf(h->GetText(),"%lf %lf %lf",
      &from.data()[0], &from.data()[1], &from.data()[2]);
    assert(ret == 3);
  }
  if(const XMLElement *h = root->FirstChildElement("drag_arrow_to"))
  {
    Vec3 & to = drag_arrow.to;
    ret = sscanf(h->GetText(),"%lf %lf %lf",
      &to.data()[0], &to.data()[1], &to.data()[2]);
    assert(ret == 3);
    drag_arrow.activated = true;
  }

  if(const XMLElement *h = root->FirstChildElement("color"))
  {
    ret = sscanf(h->GetText(),"%f %f %f %f",
      color.data()+0, color.data()+1, color.data()+2, color.data()+3);
    assert(ret == 4);
  }
  {
    int i = 0;
    for(const XMLElement * ct_off_xml = root->FirstChildElement("ct_off");
      ct_off_xml;
      ct_off_xml = ct_off_xml->NextSiblingElement("ct_off"))
    {
      assert(i<max_children);
#ifndef NDEBUG
      int ret = 
#endif
      sscanf(ct_off_xml->GetText(),"%lf %lf %lf",
        &ct_off[i][0], &ct_off[i][1], &ct_off[i][2]);
      assert(ret == 3);
      i++;
    }
  }
  // Children will be added by readXML
  return void(ret);
}

void Node::clear_state()
{
  t_off = Vec3(0,0,-radius);
  for(int c = 0;c<max_children;c++)
  {
    ct_off[c] = Vec3(0,0,0);
  }
  r_off = Quat(1,0,0,0);
}

void Node::reset_offsets()
{
  t_off = Vec3(0,0,-radius);
  r_off = Quat(1,0,0,0);
  if(parent != NULL)
  {
    parent->ct_off[parent_id] = Vec3(0,0,0);
  }
}

// recursive helper for Node::matlab, appends root and root's subtree to C and
// BE
// 
// Inputs:
//   root  root node of subtree
//   vC  #Nodes-so-far by dim list of node positions
//   vBE  #edges-so-far by 2 list of edges indices into vC
//   vRP  #edges-so-far by 2 list of rigid piece ides
//   rigid_so_far  number of rigid pieces so far.
// Outputs:
//   vC  #Nodes-so-far+#subtree nodes by dim list of node positions
//   vBE  #edges-so-far+#subtree edges by 2 list of edges indices into vC
static void matlab(
  const Node * root,
  const bool at_bind,
  std::vector<std::vector<double> > & vC,
  std::vector<std::vector<int> >    & vBE,
  std::vector<int >    & vP,
  std::vector<int >    & vRP,
  int p_rigid,
  int & rigid_so_far)
{
  using namespace std;
  // Base case if first call is null
  if(root==NULL)
  {
    return;
  }
  // Index of root in vC
  int ri = vC.size();
  // Local work:
  Vec3 o = (at_bind?root->origin_at_bind():root->origin());
  vector<double> vo(3); vo[0] = o[0]; vo[1] = o[1]; vo[2] = o[2];
  vC.push_back(vo);
  // Index of incoming bone (last one pushed)
  const int bei = vBE.size()-1;
  // Recurisve calls:
  // Loop over children
  for(int i = 0;i<root->get_max_children();i++)
  {
    vector<int> e(2);
    e[0] = ri;
    // Index of child in vC
    e[1] = vC.size();
    // Base case handled here because no recursion on leaves
    if(root->get_child(i))
    {
      vP.push_back(bei);
      vBE.push_back(e);
      int this_rigid = -2;
      if(NULL != dynamic_cast<const Splitter*>(root))
      {
        // Splitter children edges are part of rigid piece with parent.
        assert(!vRP.empty());
        vRP.push_back(vRP[p_rigid]);
        this_rigid = p_rigid;
      }else
      {
        // otherwise increment (last item should be greatest)
        this_rigid = vRP.size();
        vRP.push_back(rigid_so_far);
        rigid_so_far++;
      }
      // Recurse (child[i] will be immediately appended to vC)
      matlab(root->get_child(i),at_bind,vC,vBE,vP,vRP,this_rigid,rigid_so_far);
    }
  }
}

void Node::matlab(
  const Node * root,
  const bool at_bind,
  Eigen::MatrixXd & C,
  Eigen::MatrixXi & BE,
  Eigen::VectorXi & P,
  Eigen::VectorXi & RP)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  // Temporarily keep in vectors
  vector<vector<double> > vC;
  vector<vector<int> > vBE;
  vector<int > vRP,vP;
  int rigid_so_far = 0;
  ::matlab(root,at_bind,vC,vBE,vP,vRP,-1,rigid_so_far);
  list_to_matrix(vC,C);
  list_to_matrix(vBE,BE);
  list_to_matrix(vP,P);
  list_to_matrix(vRP,RP);
}

// recursive helper for Node::transformations, appends root and root's subtree
// to T
// 
// Inputs:
//   root  root node of subtree
//   vT  #Nodes-so-far by 4x4 list of Affine transformations
// Outputs:
//   vT  #Nodes-so-far+#subtree nodes by 4x4 list of affine transformations
template <typename Q, typename QAlloc, typename T>
static void transformations(
  const Node * root,
  std::vector<Q,QAlloc> & vQ,
  std::vector<T> & vT)
{
  using namespace std;
  using namespace Eigen;
  // Base case if first call is null
  if(root==NULL)
  {
    return;
  }
  if(root->get_parent() != NULL)
  {
    // Local work:
    Quat q; Vec3 t; root->rigid_to_self(q,t);
    Quat qb; Vec3 tb; root->rigid_to_self_at_bind(qb,tb);
    // rigid change
    Quat dq = q * qb.conjugate();

    //dq.normalize();
    Vec3 dt = t - (dq*tb);
    vT.push_back(dt);
    vQ.push_back(dq);
  }

  // Recursive calls:
  // Loop over children
  for(int i = 0;i<root->get_max_children();i++)
  {
    if(root->get_child(i))
    {
      // Recurse (child[i] will be immediately appended to vC)
      transformations(root->get_child(i),vQ,vT);
    }
  }
}


template <typename DerivedT>
void Node::transformations(
  const Node * root,
  const bool select_rigid,
  Eigen::PlainObjectBase<DerivedT>& T)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  // Temporarily keep in vectors
  vector<Quaterniond,aligned_allocator<Quaterniond> > vQ;
  vector<Vec3> vT;
  vector<Affine3d,aligned_allocator<Affine3d> > vA;
  Node::transformations(root,select_rigid,vQ,vT);
  ::quattrans2affine(vQ,vT,vA);
  list_affine_to_stack(vA,T);
}

template <typename Q, typename QAlloc, typename T>
void Node::transformations(
  const Node * root,
  const bool select_rigid,
  std::vector<Q,QAlloc> & vQ,
  std::vector<T> & vT)
{
  using namespace std;
  using namespace Eigen;
  vQ.clear();
  vT.clear();
  ::transformations(root,vQ,vT);
  if(select_rigid)
  {
    MatrixXd C;
    MatrixXi BE;
    VectorXi P,RP;
    matlab(root,false,C,BE,P,RP);
    {
      auto copy = vQ;
      throw_at(vQ,RP,copy);
      vQ = copy;
    }
    {
      auto copy = vT;
      throw_at(vT,RP,copy);
      vT = copy;
    }
  }
}

template <typename Q_type, typename QAlloc>
void Node::relative_rotations(
  const Node * root,
  std::vector<Q_type,QAlloc> & vdQ)
{
  using namespace std;
  vdQ.clear();
  if(!root)
  {
    return;
  }
  // Should be const
  deque<const Node*> Q = deque<const Node*>(1,root);
  while(Q.size()>0)
  {
    const Node * n = Q.front(); 
    Q.pop_front();
    //Quat q(1,0,0,0),q_at_bind(1,0,0,0);
    if(n->parent != NULL)
    {
      // Rigid to self combines deformation rotations with rest bone rotations
      // of the z-axis:
      //   θr2s' = θfk' ⋅ Θz
      // Recall definition of forward kinematics:
      //   θfk' = θfk'(p) ⋅ Θr'
      // Then
      //   θr2s' = θfk' ⋅ Θz
      //   θr2s' ⋅ θz* = θfk'
      //   θr2s' ⋅ θz* = θfk'(p) ⋅ Θr'
      //   θr2s' ⋅ θz* = θr2s'(p) ⋅ θz(p)* ⋅ θr'
      //   (θr2s'(p) ⋅ θz(p)*)* ⋅ θr2s' ⋅ θz* = θr'
      Vec3 t;
      Quat qr2s,qr2s_at_bind,qr2sp,qr2sp_at_bind;
      n->rigid_to_self(qr2s,t);
      n->rigid_to_self_at_bind(qr2s_at_bind,t);
      n->parent->rigid_to_self(qr2sp,t);
      n->parent->rigid_to_self_at_bind(qr2sp_at_bind,t);
      vdQ.push_back((qr2sp * qr2sp_at_bind.conjugate()).conjugate() * qr2s * qr2s_at_bind.conjugate());
    }
    // add children: BFS order
    Node::add_children(n,Q,Q.begin());
  }
}


// Recursive helper for readXML
static Node * readXML(
  const tinyxml2::XMLElement * root)
{
  using namespace tinyxml2;
  // Basecase
  if(!root)
  {
    return NULL;
  }
  // Create new node
  const int max_children = root->IntAttribute("max_children");
  const char * type_str = root->Attribute("type");
  const NodeType type = Node::type_from_string(type_str);
  Node * n = Node::new_Node(type,max_children);
  if(!n)
  {
    return NULL;
  }
  n->parseXML(root);

  // Loop over children, recursively adding them
  for(const XMLElement * child = root->FirstChildElement(NODE_DOM);
    child;
    child = child->NextSiblingElement(NODE_DOM))
  {
    int cid = child->IntAttribute("parent_id");
    assert(cid < max_children && cid>=0);
    n->set_child(readXML(child),cid);
  }
  return n;
}

Node * Node::readXML(const std::string filename)
{
  using namespace tinyxml2;
  using namespace std;
  XMLDocument doc;
  if(doc.LoadFile(filename.c_str()) != XML_NO_ERROR)
  {
    cerr<<REDRUM("Node::readXML(): Error: could not load "<<filename)<<endl;
    return NULL;
  }
  XMLElement * root = doc.FirstChildElement(NODETREE_DOM);
  if(root == NULL)
  {
    cerr<<"readXML: Null root found."<<endl;
    return NULL;
  }
  // Static information
  Node::color_parts = root->BoolAttribute("color_parts");
  // Recursively read node xml and create appropriate nodes
  return ::readXML(root->FirstChildElement(NODE_DOM));
}

bool Node::writeXML(const std::string filename, const Node * n)
{
  using namespace std;
  using namespace tinyxml2;
  XMLDocument doc;
  XMLElement * root = 
    doc.InsertEndChild(doc.NewElement(NODETREE_DOM))->ToElement();
  // Static information
  //root->SetAttribute("beach_balls",beach_balls);
  //root->SetAttribute("bones",bones);
  //root->SetAttribute("drag_along_axis",drag_along_axis);
  //root->SetAttribute("show_bind",show_bind);
  root->SetAttribute("color_parts",color_parts);
  // Try to append XML recursively
  if(n!=NULL && !n->appendXML(doc,root))
  {
    cerr<<"null root found."<<endl;
    return false;
  }
  // Public fields
  return XML_NO_ERROR == doc.SaveFile(filename.c_str());
}

NodeType Node::type_from_string(const std::string str)
{
  for(int n = 0;n<NUM_NODE_TYPES;n++)
  {
    if(str == node_type_strings[n])
    {
      return (NodeType)n;
    }
  }
  return NODE_TYPE_UNKNOWN;
}

Node * Node::new_Node(const NodeType type, const int max_children)
{
  using namespace std;
  switch(type)
  {
    case NODE_TYPE_TBT:
    {
      assert(max_children == 1);
      return new TBT();
    }case NODE_TYPE_SPLITTER:
    {
      assert(max_children > 0);
      return new Splitter(max_children);
    }case NODE_TYPE_ROOT:
    {
      assert(max_children == 1);
      return new Root();
    }case NODE_TYPE_ENDCAP:
    {
      assert(max_children == 0);
      return new EndCap();
    }default:
    {
      cerr<<REDRUM("Node::new_Node(): Error: unknown type ("<<type<<").")<<endl;
      return NULL;
    }
  }
}

void Node::add_children(
  const Node * n,
  std::deque<Node*> & Q,
  const std::deque<Node*>::iterator & Qit)
{
  using namespace std;
  vector<Node*> c(n->get_children()); 
  Q.insert(Qit, c.begin(), c.end());
}

void Node::add_children(
  const Node * n,
  std::deque<const Node*> & Q,
  const std::deque<const Node*>::iterator & Qit)
{
  using namespace std;
  vector<const Node*> c(n->get_const_children()); 
  Q.insert(Qit, c.begin(), c.end());
}

//template <typename return_type,typename func_type>
//static return_type Node::for_all(
//  Node * root,
//  const func_type & func)
//{
//  using namespace std;
//  return_type ret;
//  deque<Node*> Q = deque<Node*>(1,root);
//  while(Q.size()>0)
//  {
//    Node * n = Q.front(); 
//    Q.pop_front();
//    // Reset "from" to origin
//    ret |= func(n);
//    // add children
//    add_children(n,Q,Q.end());
//  }
//  return ret;
//}

// Explicit instnaciation
template void Node::transformations<Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Node const*, bool, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);
template void Node::relative_rotations<Eigen::Quaternion<double, 0>, Eigen::aligned_allocator<Eigen::Quaternion<double, 0> > >(Node const*, std::vector<Eigen::Quaternion<double, 0>, Eigen::aligned_allocator<Eigen::Quaternion<double, 0> > >&);
