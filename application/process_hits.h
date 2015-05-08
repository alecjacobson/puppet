#ifndef PROCESS_HITS_H
#define PROCESS_HITS_H

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

#include <vector>
// Given a number of hits and a select buffer, determine a list of hit names
// for the closest hit (minimum Z depth)
//
// Inputs:
//   hits  number of hits
//   buffer  stencil buffer
////   max_buffer_size  max size of buffer
std::vector<GLuint> process_hits(
  const GLint hits, 
  const GLuint buffer[]);
#endif
