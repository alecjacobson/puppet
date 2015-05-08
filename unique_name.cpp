#include "unique_name.h"
#include <algorithm>
#include <cassert>

// Implementation
static GLuint next_name = 1;
GLuint unique_name()
{
  return next_name++;
}

float next_color[] = {0.0f, 0.0f, 0.0f, 1.0f};
void unique_color(float * color)
{
  using namespace std;
  copy(next_color,next_color+4,color);
  next_color[0] = float(int(next_color[0]*255.0 +   7)%256)/255.0;
  next_color[1] = float(int(next_color[1]*255.0 +  71)%256)/255.0;
  next_color[2] = float(int(next_color[2]*255.0 + 121)%256)/255.50;
  assert( next_color[0] != 0 || next_color[1] != 0 || next_color[2] != 0 );
}
