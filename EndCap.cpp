#include "EndCap.h"
#include "Mesh.h"

// Functions
#include <igl/STR.h>
#include <igl/C_STR.h>
#include "shift_color.h"

#include <igl/readOBJ.h>
#include <igl/per_face_normals.h>
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

EndCap::EndCap():
  // No children
  Node(0)
{
  radius *= 0.05;
  // ... and adjust t_off
  t_off = Vec3(0,0,-radius);
  pickles.resize(1);
  pickle_hit.resize(1);
  // Endcaps do not store colors (or anything) so use a default color
  std::copy(
    Node::part_color[TERMINATOR_PART],
    Node::part_color[TERMINATOR_PART]+4,
    color.data());
}

void EndCap::draw_self() const 
{
  using namespace igl;
  using namespace std;
  glEnable(GL_NORMALIZE);
  glPushMatrix();
  double eff_alpha = alpha;
  const float * color = 
    (Node::color_parts?Node::part_color[TERMINATOR_PART]:this->color.data());
  pickles[0].activate_pickle_or_diffuse_material(color,eff_alpha);
  part[TERMINATOR_PART].draw_and_cache();
  glPopMatrix();
}

void EndCap::rigid_to_child(
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

void EndCap::set_is_selected(const bool v)
{
  using namespace std;

  bool was_selected = is_selected;
  // Call base class
  Node::set_is_selected(v);
  is_selected = v;
  string prefix = STR("node"<<"_"<<name);
  if(is_selected && !was_selected)
  {
    // Add EndCap relevant anttweakbar entries

  }else if(!is_selected && was_selected)
  {
    // Remove EndCap relevant anttweakbar entries
  }
}

tinyxml2::XMLElement * EndCap::appendXML( 
  tinyxml2::XMLDocument & doc, 
  tinyxml2::XMLElement * parent) const
{
  // Call base class
  tinyxml2::XMLElement * endcap = Node::appendXML(doc,parent);
  endcap->SetAttribute("type","EndCap");
  return endcap;
}

