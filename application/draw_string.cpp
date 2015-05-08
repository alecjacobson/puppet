#include "draw_string.h"
#include <algorithm>


void draw_string(
    const double x, 
    const double y, 
    const double z, 
    const std::string str,
    void * font )
{
  using namespace std;
  //  Specify the raster position for pixel operations.
  glRasterPos3d(x, y, z);

  for_each(str.begin(),str.end(),bind1st(ptr_fun(&glutBitmapCharacter),font));

  // This is hilariously bad:
  //glScalef(1/152.38, 1/152.38, 1/152.38);
  //for_each(str.begin(),str.end(),bind1st(ptr_fun(&glutStrokeCharacter),GLUT_STROKE_ROMAN));
}
