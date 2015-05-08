#include "diffuse_material.h"
#include <igl/material_colors.h>
#include <algorithm>


#ifdef __APPLE__
#  include <OpenGL/gl.h>
#elif defined(_WIN32)
#    define NOMINMAX
#    include <Windows.h>
#    undef NOMINMAX
#else
#  include <GL/gl.h>
#endif

void diffuse_material(const float * diffuse, const float alpha)
{
  using namespace std;
  using namespace igl;
  // Turn-off per vertex/manual colors befor specifying material properties
  glDisable(GL_COLOR_MATERIAL);
  float mat_ambient[4], mat_diffuse[4], mat_specular[4], mat_shininess=128;
  copy(GOLD_AMBIENT,GOLD_AMBIENT+4,mat_ambient);
  copy(diffuse,diffuse+4,mat_diffuse);
  const float BLACK[4] = {0,0,0,1};
  copy(BLACK,BLACK+4,mat_specular);
  // alpha
  mat_ambient[3] =  alpha;
  mat_diffuse[3] =  alpha;
  mat_specular[3] = alpha;
  // Set material properties
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,  mat_ambient  );
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,  mat_diffuse  );
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular );
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
  // Set color to diffuse color to be sure that a proper color is in the color
  // field in case anybody looks there hoping to find something interesting
  glColor4fv(mat_diffuse);
}
