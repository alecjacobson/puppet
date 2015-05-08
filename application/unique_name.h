#ifndef UNIQUE_NAME_H
#define UNIQUE_NAME_H

#if __APPLE__
#  include <OpenGL/gl.h>
#else
#  ifdef _WIN32
#    define NOMINMAX
#    include <Windows.h>
#    undef NOMINMAX
#  endif
#  include <GL/gl.h>
#endif

// Generate a unique name for use in things like the open gl name stack
GLuint unique_name();
void unique_color(float * color);

#endif
