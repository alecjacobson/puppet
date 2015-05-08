#include "Splitter.h"

// Classes
#include "Mesh.h"
#include "TBT.h"

// Functions
#include "diffuse_material.h"
#include "shift_color.h"
#include "dim_lights.h"
#include "push_string_return_char.h"

// Constants
#include <igl/PI.h>
#include <igl/material_colors.h>

// Functions
#include <igl/REDRUM.h>
#include <igl/project.h>
#include <igl/project_to_line.h>
#include <igl/quat_mult.h>
#include <igl/quat_conjugate.h>
#include <igl/mat_to_quat.h>
#include <igl/trackball.h>
#include <igl/quat_to_mat.h>
#include <igl/quat_conjugate.h>
#include <igl/quat_mult.h>
#include <igl/STR.h>
#include <igl/C_STR.h>

// Convenience
#include "tinyxmlConvenience.h"

#include <iostream>
#include <stdexcept>


// Round splitter angles to
#define SPLITTER_ANGLE_ROUND 15.0


// Static leap frog for AntTweakbar
static void set_angles(void *clientData)
{
  Splitter * s = static_cast<Splitter*>(clientData);
  s->set_angles(/*only_if_selected=*/false);
}

static void set_selected_angle(void *clientData)
{
  Splitter * s = static_cast<Splitter*>(clientData);
  s->set_angles(/*only_if_selected=*/true);
}



Splitter::Splitter(const int max_children) :
  param_announce_angles_change(NULL),
  announce_angles_change(NULL),
  Node(max_children),
  angles(max_children,std::vector<double>(3,0)),
  child_is_hover(max_children,false),
  child_is_down(max_children,false),
  child_is_selected(max_children,false),
  child_drag(max_children,false),
  down_angles(max_children,std::vector<double>(3,0)),
  is_hand(false)
{
  using namespace std;
  // Adjust radius
  radius *= 0.65;
  // ... and adjust t_off
  t_off = Vec3(0,0,-radius);
  // one for self + each child
  pickles.resize(max_children+1);
  pickle_hit.resize(max_children+1);
  for(int c = 0;c<max_children;c++)
  {
    angles[c][1] = 0;//int(180+(c+1)*(360.0/(double(max_children)+1)))%360;
  }
}

Splitter::Splitter(const Splitter & /*that*/) :
  param_announce_angles_change(NULL),
  announce_angles_change(NULL),
  Node(max_children),
  angles(max_children,std::vector<double>(3,0)),
  child_is_hover(max_children,false),
  child_is_down(max_children,false),
  child_is_selected(max_children,false),
  child_drag(max_children,false),
  down_angles(max_children,std::vector<double>(3,0)),
  is_hand(false)
{
  assert(false);
  throw std::runtime_error("Splitter(const Splitter &) not allowed");
}

Splitter & Splitter::operator=(const Splitter& /*that*/)
{
  assert(false);
  throw std::runtime_error("Splitter(const Splitter &) not allowed");
  return *this;
}

void Splitter::draw_self() const
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  glEnable(GL_NORMALIZE);
  double eff_alpha = alpha;
  const Vector4f HAND_COLOR(1.0,0.7,0.7,1.0);
  const float * color = (is_hand?HAND_COLOR.data():
    (Node::color_parts?
      Node::part_color[SPLITTER_C_PART]:
      this->color.data()));
  pickles[max_children].activate_pickle_or_diffuse_material(color,eff_alpha);
  // Draw plug to parent
  glPushMatrix();
  glTranslated(0,0,-radius/0.65*0.34);

  // http://stackoverflow.com/a/6506716/148668 
  bool some_child_is_hover = 
    find(child_is_hover.begin(), child_is_hover.end(), true) != 
    child_is_hover.end();
  bool some_child_is_selected = 
    find(child_is_selected.begin(), child_is_selected.end(), true) != 
    child_is_selected.end();

  // Dim if hovered over
  if(is_hover && ! some_child_is_hover)
  {
    dim_lights(0.25);
  }else if (is_selected && !some_child_is_selected)
  {
    dim_lights(0.365);
  }

  Node::part[TBT_PLUG_PART].draw_and_cache();
  glPopMatrix();
  // Draw sphere
  GLUquadricObj * sphere = gluNewQuadric();
  assert(sphere);
  gluQuadricNormals(sphere, GLU_SMOOTH);
  //gluQuadricDrawStyle(sphere, GLU_SILHOUETTE);
  gluQuadricDrawStyle(sphere, GLU_FILL);
  gluQuadricTexture(sphere, GL_TRUE); // Tried both GL_TRUE and GL_FALSE
  gluSphere(sphere,radius/0.65*0.57,20,20);

  // Dim if hovered over
  if(is_hover && ! some_child_is_hover)
  {
    dim_lights(1./0.25);
  }else if (is_selected && !some_child_is_selected)
  {
    dim_lights(1./0.365);
  }

  // Draw outlets to children
  for(int i =0;i<max_children;i++)
  {
    glPushMatrix();
    Quat q;
    Vec3 v;
    rigid_to_child(i,q,v);
    glTranslatef(v(0),v(1),v(2));
    glMultMatrixd(Affine3d(q).data());
    glTranslated(0,0,radius + radius/0.65 * 0.34);
    const float * color = (is_hand?HAND_COLOR.data():
      (Node::color_parts?
        Node::part_color[TERMINATOR_PART]:
        this->color.data()));
    pickles[i].activate_pickle_or_diffuse_material(color,eff_alpha=alpha);

    // Dim if hovered over
    float ld[4];
    glGetLightfv(GL_LIGHT0,GL_DIFFUSE,ld);
    if(child_is_hover[i])
    {
      dim_lights(0.25);
    }else if (child_is_selected[i])
    {
      dim_lights(0.365);
    }

    Node::part[TBT_SOCKET_PART].draw_and_cache();

    // Reset lighting
    if(child_is_hover[i])
    {
      dim_lights(1./0.25);
    }else if (child_is_selected[i])
    {
      dim_lights(1./0.365);
    }

    glPopMatrix();
  }
}

void Splitter::rigid_to_child(const int i, Quat & q, Vec3 & v) const
{
  using namespace Eigen;
  using namespace igl;
  using namespace std;
  assert(i<max_children);
  assert(i>=0);

  if(is_hand)
  {
    // Should be able to trust that angles are set correctly, but in any case
    // we do not know which outlet is which child so we'll have to hard code
    // this.

    // One-off to fit to chimpanzee-hand-model
    const double s = 0.5;

    switch(i)
    {
      case 1:
      case 2:
      case 3:
      case 4:
      {
        const double offset = 1.5*radius*((4-(double)i)-2);
        q = Quaterniond::Identity();
        v = s*Vector3d(0,offset,-5.*radius);
        break;
      }
      case 0:
        q = 
          Quaterniond(AngleAxisd(PI/4.0,Vector3d(1,0,0))) *
          Quaterniond(AngleAxisd(PI/2.0,Vector3d(0,0,1)));
        v = s*Vector3d(0,3.5*radius,-1.2*radius);
        break;
      default:
        assert(false);
        break;
    };
  }else
  {
    // Q: if q = TBT(θ1,θ2,θ3), then q.conj = TBT(-θ3,-θ2,-θ1)?
    // A: Yes. q = R1 R2 R3, q.conj = (R1 R2 R3)^T = R3^T R2^T R1^T
    q = TBT::TBT_angles_to_quat(-angles[i][2],-angles[i][1],-angles[i][0]);
    v = q*Vec3(0,0,-radius);
  }
}

bool Splitter::right_down(const int x, const int y, const int /*modifier*/)
{
  using namespace std;
  // Call base class
  Node::right_down(x,y);
  // default to false
  std::fill(child_drag.begin(),child_drag.end(),false);
  // copy angles at down
  down_angles = angles;
  for(int c = 0;c<max_children;c++)
  {
    if(child_is_down[c])
    {
      child_drag[c] = true;
      // Turn *on* trackball : we're not overriding it (?) Aren't we though?
      rotate_on = true;
      translate_on = false;
      break;
    }
  }
  return is_down;
}

bool Splitter::down(const int x, const int y, const int /*modifier*/)
{
  // Call base class
  Node::down(x,y);
  std::fill(child_drag.begin(),child_drag.end(),false);
  return is_down;
}

bool Splitter::drag(const int x, const int y, const int /*modifier*/)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  bool any = false;
  if(is_down)
  {
    for(int i = 0;i<max_children;i++)
    {
      if(child_drag[i])
      {
        any |= true;
        // interpret drag as change in this angle
        {
          // Pick up origin location
          pushmv();
          double mv[16];
          glGetDoublev(GL_MODELVIEW_MATRIX,mv);
          push();
          // Also push to child socket
          Quat q;
          Vec3 v;
          rigid_to_child(i,q,v);
          glTranslatef(v(0),v(1),v(2));
          glMultMatrixd(Affine3d(q).data());
          glTranslated(0,0,0);

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
            double mvs[16];
            {
              for(int i=0;i<3;i++)
              {
                for(int j=0;j<3;j++)
                {
                  s += (mv[j*4+i])*(mv[j*4+i]);
                }
              }
              for(int i=0;i<4;i++)
              {
                for(int j=0;j<4;j++)
                {
                  mvs[j*4+i] = mv[j*4+i]/sqrt(s/3.0);
                }
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
            // Get child quaternion at down
            Quat down_q = 
              TBT::TBT_angles_to_quat(
                -down_angles[i][2],
                -down_angles[i][1],
                -down_angles[i][0]);
            Quat q;
            quat_mult<double>(t2,&down_q.x(),&q.x());
            // Convert to tbt angles
            Vec3 euler = TBT::quat_to_TBT_angles(q);
            // Reverse order since the splitter angles are "undoing" a TBT
            // transformation.
            angles[i][0] = -euler[2];
            angles[i][1] = -euler[1];
            angles[i][2] = -euler[0];
            //// Snap angles
            //for(int j = 0;j<3;j++)
            //{
            //  // copy and round by 10 degrees
            //  angles[i][j] = round(angles[i][j]/SPLITTER_ANGLE_ROUND)*
            //    SPLITTER_ANGLE_ROUND;
            //}
          }
        }
      }
    }
  }
  // Call base class
  if(!any)
  {
    Node::drag(x,y);
  }
  last_x = x;
  last_y = y;
  return is_down;
}


void Splitter::set_is_hover(const bool v)
{
  for(int i = 0;i<max_children;i++)
  {
    child_is_hover[i] = v && pickle_hit[i];
  }
  // Call base class
  Node::set_is_hover(v);
}

void Splitter::set_is_selected(const bool v)
{
  using namespace std;
  bool was_selected = is_selected;
  for(int i = 0;i<max_children;i++)
  {
    child_is_selected[i] = v && pickle_hit[i];
  }
  // Call base class
  Node::set_is_selected(v);
  is_selected = v;
  string prefix = STR("node"<<"_"<<name);
  if(is_selected && !was_selected)
  {
    // Add TBT relevant anttweakbar entries
    TwAddButton(
      Node::bar,
      push_string_return_char(tw_vars,C_STR(prefix<<"_set_angles")),
      ::set_angles,
      this,
      C_STR("label='set_angles' "<<
        "group='" << prefix << "' " <<
        "help='Set angles of splitter (should have TBTs in each child slot)' ")
      );
    TwAddButton(
      Node::bar,
      push_string_return_char(tw_vars,C_STR(prefix<<"_set_selected_angle")),
      ::set_selected_angle,
      this,
      C_STR("label='set_selected_angle' "<<
        "group='" << prefix << "' " <<
        "help='Set angle of splitter for selected outlet (should have TBT slot)' ")
      );
  }
}

void Splitter::set_is_down(const bool v)
{
  for(int i = 0;i<max_children;i++)
  {
    child_is_down[i] = v && pickle_hit[i];
    if(child_is_down[i])
    {
      new_node_cid = i;
    }
  }
  // Call base class
  Node::set_is_down(v);
}

void Splitter::set_is_hand(const bool v)
{
  is_hand = v;
  if(is_hand)
  {
    // do something
  }
}

bool Splitter::get_is_hand() const
{
  return is_hand;
}

tinyxml2::XMLElement * Splitter::appendXML( 
  tinyxml2::XMLDocument & doc, 
  tinyxml2::XMLElement * parent) const
{
  using namespace tinyxml2;
  // Call base class
  XMLElement * root = Node::appendXML(doc,parent);
  // Splitter specific values
  root->SetAttribute("type","Splitter");
  root->SetAttribute("is_hand",C_STR(is_hand));
  // Angles list
  assert((int)angles.size() == max_children);
  // Backwards order because we're adding at front
  for(int i = (int)angles.size()-1;i>=0;i--)
  {
    assert(angles[i].size() == 3);
    root->InsertFirstChild(doc.NewElement("angle"))->ToElement()->InsertEndChild(
      doc.NewText(C_STR(angles[i][0]<<" "<<angles[i][1]<<" "<<angles[i][2])));
  }
  return root;
}

void Splitter::parseXML(const tinyxml2::XMLElement * root)
{
  using namespace tinyxml2;
  using namespace std;
  // Call base class
  Node::parseXML(root);
  const char * is_hand_str = root->Attribute("is_hand");
  if(NULL != is_hand_str && string(is_hand_str)=="1")
  {
    cout<<"I read a ham!"<<endl;
    set_is_hand(true);
  }
  // Parse Splitter specific values
  // Loop over angles
  {
    int i = 0;
    for(const XMLElement * angle = root->FirstChildElement("angle");
      angle;
      angle = angle->NextSiblingElement("angle"))
    {
      assert(i<max_children);
#ifndef NDEBUG
      int ret = 
#endif
      sscanf(angle->GetText(),"%lf %lf %lf",
        &angles[i][0], &angles[i][1], &angles[i][2]);
      //cout<<"angles["<<i<<"] "<<
      //  angles[i][0]<<" "<<
      //  angles[i][1]<<" "<<
      //  angles[i][2]<<" "<<endl;
      assert(ret == 3);
      i++;
    }
  }
}

void Splitter::set_angles(bool only_if_selected)
{
  using namespace std;
  for(int i = 0;i<max_children;i++)
  {
    if(child[i]!=NULL && (!only_if_selected || child_is_selected[i]))
    {
      TBT* tbt = dynamic_cast<TBT*>(child[i]);
      if(tbt)
      {
        // Copy angles from TBT child
        copy(tbt->angle,tbt->angle+3,&angles[i][0]);
        for(int j = 0;j<3;j++)
        {
          // copy and round by 10 degrees
          angles[i][j] = round(tbt->angle[j]/SPLITTER_ANGLE_ROUND)*
            SPLITTER_ANGLE_ROUND;
        }
      }else
      {
        cout<<
          REDRUM("Splitter::set_angles(): child #"<<i<<" not TBT, skipped.")<<
          endl;
      }
    }else
    {
      cout<<
        REDRUM("Splitter::set_angles(): no child #"<<i<<", skipped.")<< endl;
    }
  }
  if(announce_angles_change)
  {
    announce_angles_change(param_announce_angles_change,*this);
  }
}

const std::vector<std::vector<double> > & Splitter::get_angles() const
{
  return angles;
}
