#include "shadow_matrix.h"
#include <algorithm>

// http://www.ia.hiof.no/~borres/cgraph/explain/shadow/p-shadow.html#Projection_shadows2142941168
void shadow_matrix(
  const Eigen::Vector4d & plane,
  const Eigen::Vector4d & light_pos,
  Eigen::Matrix4d & S)
{
  using namespace std;
  using namespace Eigen;
  const double dot = plane.dot(light_pos);
  const double eps = 0;
  S = light_pos * plane.transpose();
  S += (eps-dot)*Matrix4d::Identity();
}
