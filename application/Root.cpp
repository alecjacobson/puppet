#include "Root.h"
#include "Mesh.h"

// Functions
#include <igl/STR.h>
#include <igl/C_STR.h>
#include "shift_color.h"

#include <igl/readOBJ.h>
#include <igl/per_face_normals.h>
#include <igl/canonical_quaternions.h>
#include <igl/material_colors.h>

// Convenience
#include "tinyxmlConvenience.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <cassert>


#ifndef SQRT_2_OVER_2
#  define SQRT_2_OVER_2 0.707106781f
#endif
#define s2o2 SQRT_2_OVER_2

Root::Root():
  // Only one child
  Node(1)
{
  radius *= 0.21;
  pickles.resize(1);
  pickle_hit.resize(1);
  reset_offsets();
}

void Root::draw_self() const 
{
  using namespace igl;
  using namespace std;
  glEnable(GL_NORMALIZE);
  glPushMatrix();
  double eff_alpha = alpha;
  const float * color = 
    (Node::color_parts?Node::part_color[WIRE_SOCKET_PART]:this->color.data());
  pickles[0].activate_pickle_or_diffuse_material(color,eff_alpha);
  part[WIRE_SOCKET_PART].draw_and_cache();
  glPopMatrix();
}

void Root::rigid_to_child(
#ifndef NDEBUG
    const int i,
#else
    const int /*i*/,
#endif
    Quat & q, Vec3 & v) const
{
  using namespace Eigen;
  using namespace igl;
  assert(i==0);
  q = Quat(1,0,0,0);
  v = q*Vec3(0,0,-radius);
}

bool Root::right_down(const int x, const int y, const int /*modifier*/)
{
  using namespace std;
  // Call base class
  Node::right_down(x,y);
  // Turn *on* rotation.
  rotate_on = is_down;
  translate_on = false;
  return is_down;
}

void Root::set_is_selected(const bool v)
{
  using namespace std;

  bool was_selected = is_selected;
  // Call base class
  Node::set_is_selected(v);
  is_selected = v;
  string prefix = STR("node"<<"_"<<name);
  if(is_selected && !was_selected)
  {
    // Add Root relevant anttweakbar entries

  }else if(!is_selected && was_selected)
  {
    // Remove Root relevant anttweakbar entries
  }
}

tinyxml2::XMLElement * Root::appendXML( 
  tinyxml2::XMLDocument & doc, 
  tinyxml2::XMLElement * parent) const
{
  // Call base class
  tinyxml2::XMLElement * root = Node::appendXML(doc,parent);
  // Root specific values
  root->SetAttribute("type","Root");
  return root;
}

void Root::reset_offsets()
{
  using namespace igl;
  using namespace std;
  Node::reset_offsets();
  // Pointing up with ridge facing out
  copy(XZ_PLANE_QUAT_D,XZ_PLANE_QUAT_D+4,r_off.coeffs().data());
  r_off.x() *= -1.0;
}
