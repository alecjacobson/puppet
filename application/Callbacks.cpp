#include "Callbacks.h"

#include "PupSkin.h"
#include "Node.h"
#include "TBT.h"
#include "bind.h"
#include "rig.h"

#include <igl/REDRUM.h>
#include <igl/writeTGF.h>
#include <igl/colon.h>
#include <igl/launch_medit.h>
#include <igl/writeDMAT.h>
#include <igl/writeMESH.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/canonical_quaternions.h>
//#include <igl/matlab/MatlabWorkspace.h>
#include <igl/get_seconds.h>
#include <igl/anttweakbar/ReAntTweakBar.h>

#include <deque>
#include <algorithm>
#include <iostream>

// No-op setter, does nothing
void TW_CALL no_op(const void * /*value*/, void * /*clientData*/)
{
}
// No-op getter, does nothing
void TW_CALL no_op(void * /*value*/, void * /*clientData*/)
{
}

void TW_CALL CopyStdStringToClient(std::string& destinationClientString, const std::string& sourceLibraryString)
{
    destinationClientString = sourceLibraryString;
}

void TW_CALL bindCB(void *clientData)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  PupSkin & p = *static_cast<PupSkin*>(clientData);
  // Bind device to skeleton
  ::bind(p.C,p.BE,p.get_root(),p.nodes_to_rig);
  p.mouse.set_size(p.BE.rows());
  if(p.BE.rows() != (int)p.rig_animation.bone_animations().size())
  {
    p.rig_animation.clear(p.BE.rows());
  }
  p.init_rig_animation_is_listening();
  // Turning on skinning
  p.set_cpu_lbs(true);
}

void TW_CALL rigCB(void *clientData)
{
  using namespace std;
  using namespace igl;
  using namespace igl::bbw;
  using namespace Eigen;
  PupSkin * p = static_cast<PupSkin*>(clientData);
  // Remember this skeleton
  Node::matlab(p->get_const_root(),false,p->C,p->BE,p->P,p->RP);
  // Bind (compute weights etc.)
  BBWData bbw_data;
  bbw_data.mosek_data.douparam[MSK_DPAR_INTPNT_TOL_REL_GAP]=1e-4;
  bbw_data.mosek_data.intparam[MSK_IPAR_NUM_THREADS] = 6;
  bbw_data.active_set_params.max_iter = 4;
  bbw_data.active_set_params.Auu_pd = true;
  bbw_data.verbosity = 1;
  bbw_data.active_set_params.max_iter = p->bbw_max_iter;
  if(!rig(p->get_const_mesh(),bbw_data,p->get_const_root(),p->W,p->M))
  {
    cout<<REDRUM("Rig failed.")<<endl;
    return;
  }
  // Also bind
  bindCB(clientData);
  // Turning on skinning
  p->set_cpu_lbs(true);
}

void TW_CALL snapCB(void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  if(AugViewport * vp = p->four_view.down_viewport())
  {
    igl::snap_to_canonical_view_quat(
      vp->camera.m_rotation_conj,
      1.0,
      vp->camera.m_rotation_conj);
  }
}

static void set_down_rot(
  PupSkin * p,
  const double * r)
{
  if(AugViewport * vp = p->four_view.down_viewport())
  {
    std::copy(r,r+4,vp->camera.m_rotation_conj.coeffs().data());
  }
}

void TW_CALL xyCB(void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  set_down_rot(p,igl::XY_PLANE_QUAT_D);
}

void TW_CALL xzCB(void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  set_down_rot(p,igl::XZ_PLANE_QUAT_D);
}

void TW_CALL yzCB(void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  set_down_rot(p,igl::YZ_PLANE_QUAT_D);
}

void TW_CALL orthoCB(void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  if(AugViewport * vp = p->four_view.down_viewport())
  {
    auto & camera = vp->camera;
    if(camera.m_angle == 0)
    {
      camera.dolly_zoom(IGL_CAMERA_MIN_ANGLE-camera.m_angle);
      camera.dolly_zoom(15.-camera.m_angle);
    }else
    {
      camera.dolly_zoom(IGL_CAMERA_MIN_ANGLE-camera.m_angle);
      camera.dolly_zoom(-camera.m_angle);
    }
  }
}

void TW_CALL get_mesh_VrowsCB(void * value, void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  *(int *)(value) = p->get_const_mesh().getV().rows();
}
void TW_CALL get_mesh_FrowsCB(void * value, void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  *(int *)(value) = p->get_const_mesh().getF().rows();
}

void TW_CALL set_cpu_lbsCB(const void * value, void *clientData)
{
  using namespace std;
  PupSkin * p = static_cast<PupSkin*>(clientData);
  p->set_cpu_lbs(*(const bool *)(value));
}
void TW_CALL get_cpu_lbsCB(void * value, void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  *(bool *)(value) = p->get_cpu_lbs();
}

void TW_CALL set_auto_fit_activatedCB(const void * value, void *clientData)
{
  using namespace std;
  PupSkin * p = static_cast<PupSkin*>(clientData);
  p->auto_fit_activated = *(const bool *)(value);
  if(p->auto_fit_activated)
  {
    p->was_cpu_lbs = false;
  }
}
void TW_CALL get_auto_fit_activatedCB(void * value, void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  *(bool *)(value) = p->auto_fit_activated;
}


void TW_CALL reset_all_anglesCB(void *clientData)
{
  using namespace std;
  PupSkin * p = static_cast<PupSkin*>(clientData);
  if(p->get_root())
  {
    deque<Node*> Q = deque<Node*>(1,p->get_root());
    while(Q.size()>0)
    {
      Node * n = Q.front(); 
      Q.pop_front();
      if(TBT * t = dynamic_cast<TBT*>(n))
      {
        for(int a = 0;a<3;a++)
        {
          t->angle[a] = t->angle_at_bind[a];
        }
      }
      // add children
      vector<Node*> c(n->get_children()); Q.insert(Q.end(), c.begin(), c.end());
    }
  }
}

void TW_CALL reset_all_offsetsCB(void *clientData)
{
  using namespace std;
  PupSkin * p = static_cast<PupSkin*>(clientData);
  if(p->get_root())
  {
    deque<Node*> Q = deque<Node*>(1,p->get_root());
    while(Q.size()>0)
    {
      Node * n = Q.front(); 
      Q.pop_front();
      n->reset_offsets();
      // add children
      vector<Node*> c(n->get_children()); Q.insert(Q.end(), c.begin(), c.end());
    }
  }
}

void TW_CALL reset_all_drag_arrowsCB(void *clientData)
{
  using namespace std;
  PupSkin * p = static_cast<PupSkin*>(clientData);
  if(p->get_root())
  {
    deque<Node*> Q = deque<Node*>(1,p->get_root());
    while(Q.size()>0)
    {
      Node * n = Q.front(); 
      Q.pop_front();
      n->drag_arrow.activated = false;
      // add children
      vector<Node*> c(n->get_children()); Q.insert(Q.end(), c.begin(), c.end());
    }
  }
}

void TW_CALL living_willCB(void *clientData)
{
  using namespace std;
  PupSkin * p = static_cast<PupSkin*>(clientData);
  p->save("autosave");
}

void TW_CALL set_mesh_invert_orientationCB(const void * value, void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  p->get_mesh().set_invert_orientation(*(const bool *)(value));
}

void TW_CALL get_mesh_invert_orientationCB(void * value, void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  *(bool *)(value) = p->get_mesh().get_invert_orientation();
}

void TW_CALL set_rotation_typeCB(const void * value, void *clientData)
{
  using namespace Eigen;
  using namespace std;
  using namespace igl;
  PupSkin * p = static_cast<PupSkin*>(clientData);
  const PupSkin::RotationType old_rotation_type = p->rotation_type;
  p->rotation_type = *(const PupSkin::RotationType *)(value);
  if(p->rotation_type == PupSkin::ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP && 
    old_rotation_type != PupSkin::ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP)
  {
    p->four_view.snap_to_fixed_up();
  }
}

void TW_CALL get_rotation_typeCB(void * value, void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  PupSkin::RotationType * rt = (PupSkin::RotationType *)(value);
  *rt = p->rotation_type;
}

void TW_CALL reset_rotationsCB(void *clientData)
{
  PupSkin * p = static_cast<PupSkin*>(clientData);
  switch(p->control_type)
  {
    case PupSkin::CONTROL_TYPE_PUPPET:
      reset_all_anglesCB(p);
      break;
    case PupSkin::CONTROL_TYPE_MOUSE:
    {
      auto S_copy = p->mouse.selection();
      p->mouse.reset_rotations();
      if(S_copy.rows() == p->BE.rows())
      {
        p->mouse.set_selection( S_copy, p->C, p->BE, p->P, p->RP);
      }
      break;
    }
    default:
      assert(false);
      break;

  }

}
