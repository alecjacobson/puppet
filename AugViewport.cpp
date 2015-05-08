#include "Augviewport.h"

AugViewport::AugViewport(
  const int _x, 
  const int _y, 
  const int _width, 
  const int _height):
  camera(),
  rotation_is_animating(false),
  animation_start_time(0),
  animation_from_quat(),
  animation_to_quat(),
  Viewport(_x,_y,_width,_height)
{
}

