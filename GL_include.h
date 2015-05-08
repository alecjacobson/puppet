#if __APPLE__
#  include <OpenGL/gl.h>
#  include <OpenGL/glu.h>
#else
#  ifdef _WIN32
#    define NOMINMAX
#    include <Windows.h>
#    undef NOMINMAX
#  endif
#  include <GL/gl.h>
#  include <GL/glu.h>
#endif

