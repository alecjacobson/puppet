#ifdef __llvm__
#  error "LLVM sux0rs and you're not allowed to use it, switch to gcc"
#endif


//#include "stop_bouncing_dock_icon.h"
#include "GLUT_include.h"
#include "PupSkin.h"

#include <igl/REDRUM.h>
#include <igl/dirname.h>

#include <unistd.h>
#include <iostream>


// CATCH ^C
int end = 0;
// handle that's called when ^C is pressed
void sighdl(int /*n*/)
{
  // TODO: see if why this is crashing my mac
  if(end==0)
  {
    end++;
    //::terminate();
    // At least on linux, exit is alread hooked up to ::terminate()
    exit(1);
  }else
  {
    abort();
  }
}

PupSkin * ps = NULL;

void display()
{
  ps->display();
}

void reshape(int width, int height)
{
  ps->reshape(width,height);
  glutPostRedisplay();
}

void key(unsigned char key, int mouse_x, int mouse_y)
{
  using namespace std;
  if(!ps->key(key,mouse_x,mouse_y))
  {
    switch(key)
    {
      // ^C: Hard exit
      case char(3):
        exit(1);
        break;
      // ESC: Soft exit
      case char(27):
        delete ps;
        exit(0);
        break;
      default:
        cout<<REDGIN("Unknown key command: '"<<key<<"' "<<int(key))<<endl;
    }
  }
  glutPostRedisplay();
}

void mouse(int glutButton, int glutState, int mouse_x, int mouse_y)
{
  ps->mouse_glut(glutButton,glutState,mouse_x,mouse_y);
  glutPostRedisplay();
}

void mouse_drag(int mouse_x, int mouse_y)
{
  ps->mouse_drag(mouse_x,mouse_y);
  glutPostRedisplay();
}

void mouse_move(int mouse_x, int mouse_y)
{
  ps->mouse_move(mouse_x,mouse_y);
  glutPostRedisplay();
}

void mouse_wheel(int wheel, int direction, int mouse_x, int mouse_y)
{
  ps->mouse_wheel(wheel,direction,mouse_x,mouse_y);
  glutPostRedisplay();
}

int main(int argc,char * argv[])
{
  using namespace igl;
  using namespace std;

  ps = new PupSkin();

  //// Hook up handle to ^C
  //signal(SIGINT,sighdl);
  // Initialize GLUT
  string wd = getcwd(NULL,0);
  glutInit(&argc,argv);
  // change path back
  chdir(wd.c_str());

  // Make directory same as executable/.app
  // *Must come after glutInit
  string exe = argv[0];
  int Contents;
  Contents = exe.find("/Contents/MacOS/");
  if(Contents>=0)
  {
    exe.erase(Contents);
  }
  string dir = dirname(exe);
  chdir(dir.c_str());
#ifdef PATH_FUCK
  //printf("getcwd(): %s\n", getcwd(NULL, 0));
  //printf("DYLD_LIBRARY_PATH: %s\n", getenv("DYLD_LIBRARY_PATH"));
  //printf("DYLD_FALLBACK_LIBRARY_PATH: %s\n", getenv("DYLD_FALLBACK_LIBRARY_PATH"));
#endif

//  // Load mesh
//  if(argc==2)
//  {
//    ps->load_mesh(argv[1]);
//  }else
//  {
//#ifdef __APPLE__
//    ps->load_mesh("/Users/ajx/Dropbox/models/gargoyle.off");
//#else
//    ps->load_mesh(    "/local/Dropbox/models/gargoyle.off");
//#endif
//  }
  glutInitDisplayString( "rgba depth double samples>=8 stencil");
  glutInitWindowSize(glutGet(GLUT_SCREEN_WIDTH),glutGet(GLUT_SCREEN_HEIGHT)*0.667);
  glutCreateWindow("puppet");
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(key);
  glutMouseFunc(mouse);
  glutMotionFunc(mouse_drag);
  //glutMouseWheelFunc(mouse_wheel);
  glutPassiveMotionFunc(mouse_move);
#ifdef __APPLE__
  //atexit(::terminate);  // this was causing funny issues in the destructor (weird string bugs)
#else
  //glutWMCloseFunc(::terminate);
  // FUCK!! Pressing escape or <CTRL>+C works fine but clicking the little x in
  // the window causes a segmentation fault in TwTerminate. Ridiculously this
  // only happens when using the armadillo mesh.
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif
  // Initialize PupSkin (should be after glutInit, and after initWindow?
  if(!ps->init(argc,argv))
  {
    return 1;
  }
  //stop_bouncing_dock_icon();
  glutMainLoop();
  cout<<"after glutMainLoop()"<<endl;
  delete ps;

  return 0;
}

