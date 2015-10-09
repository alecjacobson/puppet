#ifndef POSINGTEST_H
#define POSINGTEST_H
#include "SkinningMethod.h"
#include "Log.h"
#include "Mesh.h"
#include <igl/anttweakbar/ReAntTweakBar.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "GL_include.h"
#include <vector>
#include <string>
class PosingTest
{
  private:
    typedef std::vector<
      Eigen::Quaterniond,
      Eigen::aligned_allocator<Eigen::Quaterniond> > RotationList;
    // m_poses  list of list of rotations (assumed that size of each pose is
    //   the same)
    // m_cur_pose_id index into m_poses of current pose
    // m_dl_id  display list id
    // m_is_testing  whether currently testing
    // m_pose_is_visible  whether posed mesh is visible
    // m_color  color of pose mesh
    // m_log  event log 
    // m_log_filename  logging filename
    // m_last_err  measure at update compared to current pose
    // m_rig_C   Rigs joint positions
    // m_rig_BE  Rig's edge indices into C
    // m_rig_P  Rig's parent indices into BE 
    // m_rig_RP  Rig's rigid piece mapping BE to weights
    // m_rig_m  Rig's mesh
    // m_pose_mesh  Posed mesh
    // m_rig_skinning_method  Rig's skinning method
    // m_rig_W  Rig's weights
    // m_rig_M  Rig's lbs matrix
    std::vector<RotationList > m_poses;
    int m_cur_pose_id;
    GLuint m_dl_id;
    //double m_last_err;
    // Excessive, I guess these should be pointers
    Eigen::MatrixXd m_rig_C;
    Eigen::MatrixXi m_rig_BE;
    Eigen::VectorXi m_rig_P, m_rig_RP;
    Mesh m_rig_m, m_pose_mesh;
    SkinningMethod m_rig_skinning_method;
    Eigen::MatrixXd m_rig_W;
    Eigen::MatrixXd m_rig_M;
    double m_mean_norm;
    Eigen::VectorXd m_cur_Q;
  public:
    bool m_is_testing;
    bool m_pose_is_visible;
    Eigen::Vector4f m_color;
    float m_alpha;
    Log<std::string> m_log;
    std::string m_log_filename;
  public:
    PosingTest();
    // Returns const reference to m_cur_pose_id
    const int & cur_pose_id() const{return m_cur_pose_id;}
    // Returns const reference to m_poses
    const std::vector<RotationList> & poses() const{return m_poses;}
    //                        〃 to m_pose_mesh;
    const Mesh & pose_mesh() const{return m_pose_mesh;}
    // Save poses to file
    //
    // Inputs:
    //   filename  path to .dmat file
    // Returns iff success
    bool save_poses(const std::string & filename) const;
    // Load poses from a file
    //
    // Inputs:
    //   filename  path to .dmat file
    // Returns iff success
    bool load_poses(const std::string & filename);
    // Try to push a pose onto the list of poses.
    //
    // Inputs:
    //   pose  list of rotations specifying a pose
    bool push_pose(const RotationList & pose);
    // Pop last pose from list of poses
    //
    // Returns true if something was popped
    bool pop_pose();
    // Shuffle list of poses.
    void shuffle();
    // Returns const reference to current pose
    const RotationList * cur_pose() const;
    // Add items to an existing reanttweakbar object
    //
    // Inputs:
    //   rebar  reference to reanttweakbar object
    void add_to_reanttweakbar(igl::anttweakbar::ReTwBar & rebar);
    // Draw current target pose etc.
    void draw() const;
    // Draw the pose without setting any colors etc.
    void draw_raw_pose_mesh() const;
    // Sets the current pose add prepares for drawing
    //
    // Inputs:
    //   pose_id  id of pose to compile
    // Returns true if cur_pose_id was set, false if #BE != #pose
    bool set_cur_pose( const int pose_id);
    // Initialize posing for a given rig
    //
    // Inputs:
    //   C  #C by dim list of joint positions
    //   BE  #BE by 2 list of edge indices
    //   P  #BE by 1 list of parent indices into BE (-1 means root)
    //   m  mesh
    //   skinning_meshod  lbs or dqs
    //   W  #V by #W list of weights
    //   M  #V by  
    // Returns true iff rig matches poses (success of set_cur_pose(0))
    bool set_rig(
      const Eigen::MatrixXd & C,
      const Eigen::MatrixXi & BE,
      const Eigen::VectorXi & P,
      const Eigen::VectorXi & RP,
      const Mesh & m,
      const SkinningMethod & skinning_method,
      const Eigen::MatrixXd & W,
      const Eigen::MatrixXd & M);
    // Update called every draw frame or when rotations have changed
    //
    // Inputs:
    //   pose  list of current rotations from user 
    // Returns whether update was accepted
    bool update( const RotationList & pose);
    // Inputs:
    //   msg  message to log
    void if_testing_log(const std::string & msg);
    // Set current pose to (m_cur_pose_id+1) % #poses
    bool next_pose();
    // Called when user says she's complete this pose. Returns whether finish
    // was accepted (if norm is sane)
    bool finish_pose();
    // Log that help was requested
    void log_help();
    // Log view change
    //
    // Inputs:
    //   down_rot  rotation at down
    //   up_rot  rotation at up
    void log_view_change(
      const Eigen::Quaterniond & down_rot,
      const Eigen::Quaterniond & up_rot);
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
