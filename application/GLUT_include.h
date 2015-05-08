#ifdef __APPLE__
#  include <GLUT/glut.h>
//#  include <GL/freeglut.h>
#elif defined(_WIN32)
#  define NOMINMAX
#  include <Windows.h>
#  undef NOMINMAX
#  include <GL/glut.h>
#else
// Linux
#  include <GL/freeglut.h>
#endif

