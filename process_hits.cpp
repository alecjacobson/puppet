#include "process_hits.h"
// For NULL!
#include <cstddef>

std::vector<GLuint> process_hits(
  const GLint hits, 
  const GLuint buffer[])
{
  using namespace std;
  vector<GLuint> names;

  // http://www.lighthouse3d.com/opengl/picking/index.php?openglway3
  // process hits
  GLuint *ptr, *ptrNames=NULL, numberOfNames = 0;

  ptr = (GLuint *) buffer;
  GLuint minZ = 0xffffffff;
  // Loop over hits
  for (int i = 0; i < hits; i++)
  {
    GLuint names = *ptr;
    ptr++;
    // Keep track of closest hit
    if (*ptr < minZ)
    {
      numberOfNames = names;
      minZ = *ptr;
      ptrNames = ptr+2;
    }
    ptr += names+2;
  }
  // May have multiple names in stack for this hit
  //printf ("The closest hit names are ");
  names.resize(numberOfNames);
  for (int j = 0; j < (int)numberOfNames; j++,ptrNames++)
  {
    //printf ("%d ", *ptrNames);
    names[j] = *ptrNames;
  }
  //printf ("\n");
  return names;
}
