#ifndef UNIPED_H
#define UNIPED_H
#include "EigenConvenience.h"
#include "GL_include.h"
#include "Animation.h"
#include "BezierAnimation.h"
//#include "BezierKeyframe.h"

#define UNIPED_BODY 0
#define UNIPED_UPPER_LEG 1
#define UNIPED_KNEE 2
#define UNIPED_LOWER_LEG 3
#define UNIPED_ANKLE 4
#define UNIPED_FOOT 5
class Uniped
{
  public:
  // Mesh vertex positions, deformed positions, tex-coords, joint locations
  Eigen::MatrixXd V,U,UV,C,color;
  // Mesh faces, groups, bone edges
  Eigen::MatrixXi F,G,BE;
  // Bone rotation angles
  Eigen::Vector3d theta;
  // Angle offsets for 4 bones
  Eigen::Vector4d offsets;
  // Texture id
  GLuint tex;
  enum ControlType
  {
    CONTROL_TYPE_PUPPET = 0,
    CONTROL_TYPE_KEYBOARD = 1,
    CONTROL_TYPE_MOUSE = 2,
    CONTROL_TYPE_AUTO = 3,
    NUM_CONTROL_TYPES = 4
  } control_type;
  Eigen::Vector3d pos;
  //typedef Animation<BezierKeyframe<double> > ThetaAnimationType;
  typedef BezierAnimation<double> ThetaAnimationType;
  ThetaAnimationType theta_animation[3];
  Eigen::Vector3d prev_theta;
  Uniped();
  void update();
  void draw();
  void draw_progress_bar() const;
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
