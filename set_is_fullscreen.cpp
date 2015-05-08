#include "set_is_fullscreen.h"
#include <Eigen/Core>
#include "GLUT_include.h"

void TW_CALL set_is_fullscreen(const void * value, void * clientData)
{
  using namespace Eigen;
  bool & is_fullscreen  = *static_cast< bool*>(clientData);
  static int x = 0,y = 0,w = 1024, h = 512;
  const bool & new_b = *(const bool *)value;
  if(new_b == is_fullscreen)
  {
    // no change.
    return;
  }
  if(is_fullscreen)
  {
    // Restore old window settings
    glutPositionWindow(x,y);
    glutReshapeWindow(w,h);
  }else
  {
    // Remember old window settings
    x = glutGet((GLenum)GLUT_WINDOW_X);
    y = glutGet((GLenum)GLUT_WINDOW_Y);
    w = glutGet((GLenum)GLUT_WINDOW_WIDTH);
    h = glutGet((GLenum)GLUT_WINDOW_HEIGHT);
    glutFullScreen();
  }
  is_fullscreen = !is_fullscreen;
}

void TW_CALL get_is_fullscreen(void * value, void * clientData)
{
  const bool & is_fullscreen  = *static_cast<const bool*>(clientData);
  bool & b = *(bool *) value;
  b = is_fullscreen;
}

