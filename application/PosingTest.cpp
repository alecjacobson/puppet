#include "PosingTest.h"
#include "Callbacks.h"
#include "diffuse_material.h"
#include "throw_at.h"
#include <igl/quats_to_column.h>
#include <igl/writeDMAT.h>
#include <igl/readDMAT.h>
#include <igl/REDRUM.h>
#include <igl/forward_kinematics.h>
#include <igl/material_colors.h>
#include <igl/STR.h>
#include <igl/get_seconds.h>
#include <igl/ZERO.h>
#include <igl/report_gl_error.h>
#include <iostream>
#include <cassert>
#include <algorithm>

static Eigen::IOFormat CommaFormat(Eigen::FullPrecision, Eigen::DontAlignCols,", ", ", ", "", "", "", "");

PosingTest::PosingTest():
  m_poses(),
  m_cur_pose_id(-1),
  m_dl_id(0),
  m_rig_C(),
  m_rig_BE(),
  m_rig_P(),
  m_rig_m(),
  m_rig_skinning_method(SKINNING_METHOD_LBS),
  m_rig_W(),
  m_rig_M(),
  m_mean_norm(0),
  m_cur_Q(),
  m_is_testing(false),
  m_pose_is_visible(true),
  m_color(
    igl::FAST_RED_DIFFUSE[0],
    igl::FAST_RED_DIFFUSE[1],
    igl::FAST_RED_DIFFUSE[2],
    1.0),
  m_alpha(1.0),
  m_log(),
  m_log_filename("posing.log")
{
};


bool PosingTest::save_poses(const std::string & filename) const
{
  using namespace Eigen;
  using namespace igl;
  if(m_poses.size() == 0)
  {
    return writeDMAT(filename,MatrixXd(0,0));
  }
  // Save each pose as a column (x0,y0,z0,w0,x1,y1,z1,w1,...)
  MatrixXd Q(4*m_poses.front().size(),m_poses.size());
  // Loop over poses/columns
  for(int p = 0;p<(int)m_poses.size();p++)
  {
    // Loop over rotations/rows
    const auto & pose = m_poses[p];
    assert((int)pose.size()*4 == Q.rows());
    Q.col(p) = quats_to_column(pose);

  }
  return writeDMAT(filename,Q,false);
}

bool PosingTest::load_poses(const std::string & filename)
{
  using namespace Eigen;
  using namespace igl;
  using namespace std;
  MatrixXd Q;
  if(!readDMAT(filename,Q))
  {
    cerr<<"Failed to load poses from "<<filename<<endl;
    return false;
  }
  // Each column is a stream of quaternions
  assert(Q.rows()%4 == 0);
  const int np = Q.cols();
  const int nq = Q.rows()/4;
  m_poses.resize(np,RotationList(nq));
  // Loop over poses/columns
  for(int p = 0;p<np;p++)
  {
    // Loop over rotations/rows
    auto & pose = m_poses[p];
    assert((int)pose.size()*4 == Q.rows());
    for(int q = 0;q<nq;q++)
    {
      // Constructor uses wxyz
      pose[q] = Quaterniond( Q(q*4+3,p), Q(q*4+0,p), Q(q*4+1,p), Q(q*4+2,p));
    }
  }
  return true;
}

bool PosingTest::push_pose(const RotationList & pose)
{
  using namespace std;
  if(m_poses.size() > 0 && pose.size() != m_poses.front().size())
  {
    cerr<<REDRUM("PosingTest::push_pose: #pose "<<pose.size()<<
      " does not match poses in list so far "<<m_poses.front().size())<<endl;
    return false;
  }
  if(pose.size() == 0)
  {
    cerr<<REDRUM("PosingTest::push_pose: #pose = 0")<<endl;
    return false;
  }
  m_poses.push_back(pose);
  return true;
}

bool PosingTest::pop_pose()
{
  if(m_poses.size() == 0)
  {
    return false;
  }
  m_poses.pop_back();
  if(m_cur_pose_id>=(int)m_poses.size())
  {
    set_cur_pose(m_poses.size()-1);
  }
  return true;
}

void PosingTest::shuffle()
{
  std::random_shuffle ( m_poses.begin(), m_poses.end() );
}

const PosingTest::RotationList * PosingTest::cur_pose() const
{
  //assert(m_cur_pose_id>=0&&m_cur_pose_id<(int)m_poses.size());
  if(m_cur_pose_id>=0&&m_cur_pose_id<(int)m_poses.size())
  {
    return &m_poses[m_cur_pose_id];
  }else
  {
    return NULL;
  }
}

void PosingTest::add_to_reanttweakbar(igl::ReTwBar & rebar)
{
  using namespace igl;
  rebar.TwAddVarRW("posing_test_is_testing",TW_TYPE_BOOLCPP,&m_is_testing,
    "label='testing' group='Test'");
  rebar.TwAddVarRW("posing_test_pose_is_visible",TW_TYPE_BOOLCPP,
    &m_pose_is_visible,
    "label='visible' key=T group='Test'");
  const auto & get_num_posesCB = [](void *value, void *clientData)
  {
    const PosingTest & pt = *static_cast<const PosingTest*>(clientData);
    int & np = *(int*)(value);
    np = pt.poses().size();
  };
  rebar.TwAddVarCB("posing_test_num_poses",TW_TYPE_INT32,
    no_op,
    get_num_posesCB,
    this,
    "label='#poses' group='Test' readonly=true");
  rebar.TwAddVarRO("posing_test_cur_pose_id",TW_TYPE_INT32,&m_cur_pose_id,
    "label='pose' group='Test'");
  const auto & next_poseCB = [](void *clientData)
  {
    PosingTest & pt = *static_cast<PosingTest*>(clientData);
    pt.next_pose();
  };
  rebar.TwAddButton("posing_test_next_pose",
    next_poseCB,
    this,
    "label='next' group='Test'");
  const auto & log_helpCB = [](void *clientData)
  {
    PosingTest & pt = *static_cast<PosingTest*>(clientData);
    pt.log_help();
  };
  rebar.TwAddButton("posing_test_log_help",
    log_helpCB,
    this,
    "label='log help' group='Test' key=h");
  rebar.TwAddVarRO("posing_test_norm",TW_TYPE_DOUBLE,&m_mean_norm,
    "label='norm' group='Test'");
  rebar.TwSetParam("Test", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
}

void PosingTest::draw() const
{
  if(m_pose_is_visible && glIsList(m_dl_id))
  {
    diffuse_material(m_color.data(),m_alpha);
    draw_raw_pose_mesh();
  }
}

void PosingTest::draw_raw_pose_mesh() const
{
  if(m_pose_is_visible && glIsList(m_dl_id))
  {
    glCallList(m_dl_id);
  }
}

bool PosingTest::set_cur_pose( const int pose_id)
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;

  if(pose_id == -1)
  {
    // Should only set to -1 if empty
    assert(m_poses.size() == 0);
    m_cur_pose_id = -1;
    if(glIsList(m_dl_id))
    {
      glDeleteLists(m_dl_id,1);
    }
    m_dl_id = 0;
    return false;
  }

  if(!(pose_id>=0&&pose_id<(int)m_poses.size()))
  {
    cerr<<REDRUM("Pose id ("<<pose_id<<"<0 or >"<<m_poses.size())<<endl;
    return false;
  }
  const auto & pose = m_poses[pose_id];
  if(m_rig_BE.rows() != (int)pose.size())
  {
    cerr<<REDRUM("m_rig_BE.rows() ("<<m_rig_BE.rows()<<
      " != "<<" #pose ("<<pose.size()<<")")<<endl;
    return false;
  }

  m_cur_pose_id = pose_id; m_cur_Q = quats_to_column(pose);
  if_testing_log(STR("target_rel,"<<pose_id<<","<<m_cur_Q.format(CommaFormat)));

  vector<Quaterniond,aligned_allocator<Quaterniond> > vQ;
  vector<Vector3d> vT;
  forward_kinematics(m_rig_C,m_rig_BE,m_rig_P,*cur_pose(),vQ,vT);
  vQ = throw_at(vQ,m_rig_RP);
  vT = throw_at(vT,m_rig_RP);
  if(m_is_testing)
  {
    VectorXd Q = quats_to_column(vQ);
    if_testing_log(STR("target_fk,"<<pose_id<<","<<
      Q.format(CommaFormat)));
  }
  if(!m_rig_m.deformed_copy(
    m_rig_skinning_method,m_rig_W,vQ,vT,m_rig_M,m_pose_mesh))
  {
    return false;
  }

  // compile deformed mesh
  if(glIsList(m_dl_id))
  {
    glDeleteLists(m_dl_id,1);
  }
  m_dl_id = glGenLists(1);
  glNewList(m_dl_id,GL_COMPILE);
  m_pose_mesh.push_matrix();
  m_pose_mesh.draw();
  m_pose_mesh.pop_matrix();
  glEndList();
  
  return true;
}

bool PosingTest::set_rig(
  const Eigen::MatrixXd & C,
  const Eigen::MatrixXi & BE,
  const Eigen::VectorXi & P,
  const Eigen::VectorXi & RP,
  const Mesh & m,
  const SkinningMethod & skinning_method,
  const Eigen::MatrixXd & W,
  const Eigen::MatrixXd & M)
{
  m_rig_C = C;
  m_rig_BE = BE;
  m_rig_P = P;
  m_rig_RP = RP;
  m_rig_m = m;
  m_rig_skinning_method = skinning_method;
  m_rig_W = W;
  m_rig_M = M;
  return set_cur_pose(0);
}

bool PosingTest::update(const RotationList & pose)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  if(m_poses.size() == 0 || pose.size() != m_poses.front().size())
  {
    return false;
  }
  static RotationList prev_pose;
  static VectorXd prev_Q;
  static double t_prev_pose_capture = get_seconds();
  bool update_prev = false;
  const VectorXd Q = quats_to_column(pose);
  //if(pose.size() == prev_pose.size())
  //{
  //  const double mean_diff_to_prev = 
  //    (Q - prev_Q).array().pow(2).sum()/pose.size();
  //  update_prev = mean_diff_to_prev > 0.0001;
  //}
  update_prev |= (get_seconds() - t_prev_pose_capture)>0.25;
  if(update_prev)
  {
    if(m_is_testing)
    {
      if_testing_log(STR("pose_rel,"<< Q.format(CommaFormat)));
      vector<Quaterniond,aligned_allocator<Quaterniond> > vQ;
      vector<Vector3d> vT;
      forward_kinematics(m_rig_C,m_rig_BE,m_rig_P,pose,vQ,vT);
      vQ = throw_at(vQ,m_rig_RP);
      const VectorXd Qfk = quats_to_column(vQ);
      if_testing_log(STR("pose_fk,"<< Qfk.format(CommaFormat)));
    }

    prev_pose = pose;
    prev_Q = Q;
    t_prev_pose_capture = get_seconds();
  }
  if(Q.size() == m_cur_Q.size())
  {
    m_mean_norm = (Q - m_cur_Q).array().pow(2).sum()/Q.rows();
  }
     
  return true;
}

void PosingTest::if_testing_log(const std::string & msg)
{
  if(m_is_testing)
  {
    m_log<<msg;
  }
}

bool PosingTest::next_pose()
{
  if(m_poses.size()==0)
  {
    return false;
  }
  return set_cur_pose((m_cur_pose_id+1)%m_poses.size());
}

bool PosingTest::finish_pose()
{
  using namespace std;
  if(!m_is_testing)
  {
    return false;
  }
  if(m_mean_norm > 0.1)
  {
    cerr<<REDRUM("Can't advance, error ("<<m_mean_norm<<") too large...")<<endl;
    return false;
  }
  if_testing_log(STR("finish,"<<m_mean_norm));
  // Always write log on finish
  if(m_log.save(m_log_filename))
  {
    cout<<GREENGIN("Saved log to "<<m_log_filename<<".")<<endl;
  }else
  {
    cout<<REDRUM("Saving log to "<<m_log_filename<<" failed.")<<endl;
  }
  return true;
}

void PosingTest::log_help()
{
  if_testing_log("help");
}

void PosingTest::log_view_change(
  const Eigen::Quaterniond & down_rot,
  const Eigen::Quaterniond & up_rot)
{
  if_testing_log(STR("view,"<<
     down_rot.coeffs().format(CommaFormat)<<","<<
     up_rot.coeffs().format(CommaFormat)<<","));

}
