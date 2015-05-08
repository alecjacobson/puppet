#include "set_is_fullscreen.h"
#include "EigenConvenience.h"
#include "AugViewport.h"
#include "Sound.h"
#include "PuppetReader.h"
#include "PuppetReaderCallbacks.h"
#include "TBT.h"
#include "draw_string.h"
#include "draw_genie.h"
#include "push_scene.h"
#include "add_puppetreader_anttweakbar.h"
#include "place_lights.h"
#include "Log.h"
#include "shadow_matrix.h"

#include <igl/report_gl_error.h>
#include <igl/readDMAT.h>
#include <igl/draw_mesh.h>
#include <igl/draw_floor.h>
#include <igl/matlab_format.h>
#include <igl/per_face_normals.h>
#include <igl/material_colors.h>
#include <igl/trackball.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/REDRUM.h>
#include <igl/Camera.h>
#include <igl/get_seconds.h>
#include <igl/normalize_row_lengths.h>
#include <igl/C_STR.h>
#include <igl/STR.h>
#include <igl/PI.h>
#include <igl/EPS.h>
#include <igl/Viewport.h>
#include <igl/get_seconds.h>
#include <igl/ReAntTweakBar.h>
#include <igl/ZERO.h>
#include <igl/randperm.h>
#include <igl/slice.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_fixed_up.h>
#include <igl/cat.h>
#include <igl/dated_copy.h>
#include <igl/path_to_executable.h>
#include <igl/ray_sphere_intersect.h>

#include <YImage.hpp>

#ifdef WIN32
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif
#ifndef GLUT_WHEEL_UP
#define GLUT_WHEEL_UP    3
#define GLUT_WHEEL_DOWN  4
#define GLUT_WHEEL_RIGHT 5
#define GLUT_WHEEL_LEFT  6
#define GLUT_ACTIVE_COMMAND 1
#endif

#include <OpenAL/al.h>
#include <OpenAL/alc.h>
#include <AL/alut.h>

#include <boost/program_options.hpp>

#include <string>
#include <stack>
#include <iostream>
#include <algorithm>

#define BINARY_BACKUP_DIR "./binary-backups/"

bool is_fullscreen = false;
const double Cmin_angle = -0.49;
const double Cmax_angle = 0.51;
const double Tmin_angle = -2.79;
const double Tmax_angle = 2.79;
const double PLANE = 25;
Eigen::Vector3d TANK_POS_0(0,0,0);
const Eigen::Vector4f
  CHERRY_RED(1.0,0.176,0.176,1.0),
  YELLOW(0.98,0.866,0,1.),
  LIGHT_BLUE(0.129,0.65,1,1.);
struct Tank
{
  // Rocket, body, turret, cannon
  Eigen::MatrixXd BV,TV,CV,BN,TN,CN,BC,TC,CC;
  Eigen::VectorXd BS,TS,CS;
  Eigen::MatrixXi BF,TF,CF;
  Eigen::Vector4f Bcolor,Tcolor,Ccolor;
  // Rotation centers and axes
  const Eigen::Vector3d Ccenter,Tcenter,Caxis,Taxis;
  double Tangle, Cangle;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  const Eigen::Vector3d acc;
  int diff_charge;
  int charge;
  double min_charge;
  bool charged;
  bool is_firing;
  enum ControlType
  {
    CONTROL_TYPE_PUPPET = 0,
    CONTROL_TYPE_KEYBOARD = 1,
    CONTROL_TYPE_MOUSE = 2,
    NUM_CONTROL_TYPES = 3
  } control_type;
  Sound pfft;
  Tank():
    BV(),TV(),CV(),BN(),TN(),CN(),BC(),TC(),CC(),
    BS(),TS(),CS(),
    BF(),TF(),CF(),
    Bcolor(CHERRY_RED),
    Tcolor(LIGHT_BLUE),
    Ccolor(YELLOW),
    Ccenter(-1.62108,1.91406,0), Tcenter(0,0,0), Caxis(0,0,-1), Taxis(0,-1,0),
    Tangle(0),Cangle(0),
    pos(TANK_POS_0),
    vel(igl::DOUBLE_EPS,0,0),
    acc(0,0,0),
    //vel(-0.15,0,0),
    //acc(0,0,0.0001),
    diff_charge(0),
    charge(3),
    min_charge(0.0),
    charged(true),
    is_firing(false),
    control_type(CONTROL_TYPE_KEYBOARD),
    pfft(10)
  {}
} tank;

bool shadows_enabled = true;
bool ao_enabled = true;
const double Cdangle = 0.05;
const double Tdangle = 0.05;

bool is_testing = false;

// yo dawg, stupid math.h defines log
Log<std::string> lawg;
std::string log_filename = "tank.log";

const Eigen::Vector3d RPOS_RELATIVE_REST(-3.08777,1.86252,0);
// velocity and position
struct Rocket
{
  Eigen::MatrixXd V,N;
  Eigen::MatrixXi F;
  Eigen::Vector4f color;
  double init_vel;
  Eigen::Vector3d vel;
  Eigen::Vector3d pos;
  Rocket():
    V(),N(),
    F(),
    color(LIGHT_BLUE),
    init_vel(15),
    vel(-1,0,0),
    pos(RPOS_RELATIVE_REST)
  {}

} rocket;

#define NUM_TURRET_STEPS 6
#define NUM_CANNON_STEPS 3
#define NUM_DISTANCE_STEPS 4
#define NUM_TARGET_POS (NUM_TURRET_STEPS*NUM_CANNON_STEPS*NUM_DISTANCE_STEPS)

struct Target
{
  static GLuint tex;
  Eigen::MatrixXd PV,PUV,PN,SV,SUV,SN;
  Eigen::MatrixXi PF,SF;
  Eigen::Vector3d pos;
  double radius;
  double thickness;
  int rpi;
  Eigen::MatrixXd positions;
  Eigen::VectorXd radii;
  Sound pop;
  Sound fffew;
  void random_pos()
  {
    using namespace std;
    pos = positions.row(rpi);
    radius = radii(rpi);
    rpi = (rpi+1)%NUM_TARGET_POS;
  }
  //void random_pos()
  //{
  //  // [-PLANE,-PLANE/2]
  //  pos(0) = ((double)rand()/(double)RAND_MAX*-1.) * PLANE/2 - PLANE/2;
  //  // [-PLANE,PLANE]
  //  pos(2) = ((double)rand()/(double)RAND_MAX*2.0 - 1.) * PLANE;
  //  // [0.5,2.5]
  //  radius = ((double)rand()/(double)RAND_MAX*2.5 + 0.5);
  //  // [r+1/2,r+8]
  //  pos(1) = radius + ((double)rand()/(double)RAND_MAX*7.5+0.5);
  //}

  Target():
    PV(),PUV(),PN(),SV(),SUV(),SN(),
    PF(),SF(),
    pos(-7,3,0),
    radius(1), // Better defaults should be embedded in .obj models
    thickness(0.1),
    rpi(0),
    positions(NUM_TARGET_POS,3),
    radii(NUM_TARGET_POS),
    pop(1),
    fffew(1)
  {
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    std::srand ( unsigned ( std::time(0) ) );
    const double CMN = 0.;
    const double CMAX = 0.4;
    //const double TMIN =-2.57;
    //const double TMAX = 2.57;
    const double TMIN =-1.57;
    const double TMAX = 1.57;
    const double RMIN = 0.9;
    const double RMAX = 1.7;
    const double DMIN = PLANE*0.33;
    const double DMAX = PLANE*0.9;
    {
      int k = 0;
#if NUM_TURRET_STEPS == 1
      {
        double t = TMIN+(TMAX-TMIN)*0.5;
#else
      for(int ti = 0;ti<NUM_TURRET_STEPS;ti++)
      {
        double t = TMIN+(TMAX-TMIN)*(double)ti/(double)(NUM_TURRET_STEPS-1);
#endif
        // Be sure keyboard can reach this
        t = round(t/Tdangle)*Tdangle;
#if NUM_CANNON_STEPS == 1
        {
          double c = CMN+(CMAX-CMN)*0.5;
#else
        for(int ci = 0;ci<NUM_CANNON_STEPS;ci++)
        {
          double c = CMN+(CMAX-CMN)*(double)ci/(double)(NUM_CANNON_STEPS-1);
#endif
          // Be sure keyboard can reach this (up to gravity)
          //c = round(c/Cdangle)*Cdangle;
#if NUM_DISTANCE_STEPS == 1
          {
            double d = DMIN+(DMAX-DMIN)*0.5;
#else
          for(int di = 0;di<NUM_DISTANCE_STEPS;di++)
          {
            double d = DMIN+(DMAX-DMIN)*(double)di/(double)(NUM_DISTANCE_STEPS-1);
#endif
            positions.row(k) = Vector3d(cos(c)*-cos(t),sin(c),cos(c)*sin(t));
            radii(k) = ((double)rand()/(double)RAND_MAX*(RMAX-RMIN) + RMIN);
            //positions.row(k) = Vector3d(t,0,0);
            positions.row(k) *= d;
            positions.row(k) += TANK_POS_0 + Vector3d(0,2,0);
            radii(k) = std::min(radii(k),positions(k,1));
            k++;
          }
        }
      }
      assert(k==NUM_TARGET_POS);
    }
    //VectorXi I = randperm<VectorXi>(NUM_TARGET_POS);
    // Alternate left and right sides
    const int cnt = ceil(0.5*NUM_TARGET_POS);
    const int fnt = floor(0.5*NUM_TARGET_POS);
    VectorXi I(NUM_TARGET_POS);
    Map<VectorXi, 0, InnerStride<2> >(I.data(),cnt) = randperm<VectorXi>(cnt);
    Map<VectorXi, 0, InnerStride<2> >(I.data()+1,fnt) = randperm<VectorXi>(fnt).array() + cnt;
    positions = slice(positions,I,1);
    radii = slice(radii,I,1);
    random_pos();
  }
} target;

int num_hits = 0;
//int num_targets = NUM_TARGET_POS;
int num_targets = 10;
GLuint Target::tex = 0;

const double PAN_Y_OFFSET = -1;
struct State
{
  State()
  {}
} s;

// See README for descriptions
enum RotationType
{
  ROTATION_TYPE_IGL_TRACKBALL = 0,
  ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP = 1,
  ROTATION_TYPE_TURRET = 2,
  NUM_ROTATION_TYPES = 3,
} rotation_type = ROTATION_TYPE_TURRET;

bool is_rotating = false;
int down_x,down_y;
igl::Camera down_camera;

AugViewport main_vp(0,0,0,0),genie_vp(0,0,150,150);
Eigen::Vector4f flashlight_pos(-0.1,-0.1,0.9,0);
Eigen::Vector4f light_pos(0.7,0.5,-0.2,0);

std::string rebar_in_name = "tank.rbr";
const std::string rebar_out_name = "tank.rbr";
bool bar_is_visible = true;
igl::ReTwBar rebar;

PuppetReader * pr;

///*const*/ Eigen::Vector3d GRAVITY(0,-9.8*5,0);
/*const*/ Eigen::Vector3d GRAVITY(0,0,0);
double t_step = 0;
double display_t = igl::get_seconds();

bool init_puppet()
{
  using namespace Eigen;
  using namespace std;
  genie_vp.camera.look_at(
    Vector3d(0,0.25,1),Vector3d(0,0.25,0),Vector3d(0,1,0));
  pr = new PuppetReader("ignored",115200);
  return pr->is_attached();
}

double & set_Cangle(Tank & tank, Log<std::string> & lawg, const double v)
{
  using namespace std;
  static double prev_Cangle = -10;
  // Lossy run-length encoding
  if(is_testing && fabs(prev_Cangle - v) > 5e-3 )
  {
    lawg<<STR("Cangle,"<<v);
    prev_Cangle = v;
  }
  return tank.Cangle = std::min(Cmax_angle,std::max(Cmin_angle,v));
}

double & set_Tangle(Tank & tank, Log<std::string> & lawg, const double v)
{
  static double prev_Tangle = -10;
  // Lossy run-length encoding
  if(is_testing && fabs(prev_Tangle - v) > 5e-3 )
  {
    lawg<<STR("Tangle,"<<v);
    prev_Tangle = v;
  }
  return tank.Tangle = std::min(Tmax_angle,std::max(Tmin_angle,v));
}

void fire(Tank & tank, Log<std::string> & lawg)
{
  if(!tank.is_firing)
  {
    if(is_testing)
    {
      lawg<<STR("fire");
    }
    tank.is_firing = true;
    tank.pfft.play();
  }
}

void TW_CALL fireCB(void * /*clientData*/)
{
  fire(tank,lawg);
}

Eigen::Quaterniond follow_turret(const Tank & tank, const Eigen::Quaterniond & from)
{
  using namespace Eigen;
  Vector3d in(-1,0,0);
  Vector3d over(0,0,1);
  Quaterniond t = Quaterniond(AngleAxisd(tank.Tangle,tank.Taxis));
  Vector3d rin = in;
  Vector3d fin = from.matrix() * t.matrix() * over;
  return Quaterniond::FromTwoVectors(fin,rin) * from;
}

void TW_CALL set_rotation_type(const void * value, void * /*clientData*/)
{
  using namespace Eigen;
  using namespace std;
  using namespace igl;
  const RotationType old_rotation_type = rotation_type;
  rotation_type = *(const RotationType *)(value);
  if(rotation_type == ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP && 
    old_rotation_type != ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP)
  {
    auto & animation_from_quat = main_vp.animation_from_quat;
    auto & animation_to_quat = main_vp.animation_to_quat;
    animation_from_quat = main_vp.camera.m_rotation_conj;
    snap_to_fixed_up(animation_from_quat,animation_to_quat);
    // start animation
    main_vp.animation_start_time = get_seconds();
    main_vp.rotation_is_animating = true;
  }
}
void TW_CALL get_rotation_type(void * value, void * /*clientData*/)
{
  RotationType * rt = (RotationType *)(value);
  *rt = rotation_type;
}

void reshape(int width, int height)
{
  using namespace std;
  main_vp.width = width;
  main_vp.height = height;
  genie_vp.x = main_vp.x+width-genie_vp.width;
  genie_vp.y = main_vp.y+height-genie_vp.height;
  // Send the new window size to AntTweakBar
  TwWindowSize(width, height);
  main_vp.camera.m_aspect = (double)width/(double)height;
}

bool init_anttweakbar(const std::string & rebar_name)
{
  using namespace igl;
  using namespace std;
  if( !TwInit(TW_OPENGL, NULL) )
  {
    // A fatal error occured
    fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
    return false;
  }
  // Create a tweak bar
  rebar.TwNewBar("bar");
  TwDefine("bar label='tank' size='200 550' text=light alpha='200' color='68 68 68'");

  rebar.TwAddVarRW("camera_rotation", TW_TYPE_QUAT4D,
    main_vp.camera.m_rotation_conj.coeffs().data(), "open readonly=true");
  rebar.TwAddVarRW("camera_translation",TW_TYPE_DIR3D,
    main_vp.camera.m_translation.data(), "visible=false");
  rebar.TwAddVarRW("camera_at_dist",TW_TYPE_DOUBLE,
    &main_vp.camera.m_at_dist, "visible=false");
  TwType RotationTypeTW = ReTwDefineEnumFromString("RotationType","igl_trackball,two_axis_fixed_up,turret");
  rebar.TwAddVarCB("rotation_type", RotationTypeTW,
    set_rotation_type,get_rotation_type,NULL,"keyIncr=] keyDecr=[");
  rebar.TwAddVarRW("init_vel",TW_TYPE_DOUBLE,&rocket.init_vel,"");
  rebar.TwAddVarRW("gravity",TW_TYPE_DOUBLE,GRAVITY.data()+1,"");
  TwType ControlTypeTW = 
    ReTwDefineEnumFromString("ControlType","puppet,keyboard,mouse");
  rebar.TwAddVarRW("control_type",ControlTypeTW,&tank.control_type,"label=control key=c");
  rebar.TwAddVarRW("num_targets",TW_TYPE_INT32,&num_targets,"");
  rebar.TwAddVarRW("shadows_enabled",TW_TYPE_BOOLCPP,&shadows_enabled,"");
  rebar.TwAddVarRW("ao_enabled",TW_TYPE_BOOLCPP,&ao_enabled,"");
  rebar.TwAddVarRW("light_pos",TW_TYPE_DIR3F,light_pos.data(),"");
  rebar.TwAddVarCB("is_fullscreen",TW_TYPE_BOOLCPP,
    set_is_fullscreen,get_is_fullscreen,&is_fullscreen,"key=f");
  add_puppetreader_anttweakbar(pr,rebar);
  rebar.TwSetParam("PuppetReader", "opened", TW_PARAM_INT32, 1, &INT_ZERO);

  rebar.load(rebar_name.c_str());

  return true;
}

Eigen::Matrix4d rigid(
  const Eigen::Vector3d & center,
  const Eigen::Vector3d & axis,
  const double angle)
{
  using namespace Eigen;
  Affine3d t = Affine3d::Identity();
  t.translate(center);
  t.rotate(AngleAxisd(angle,axis));
  t.translate(-center);
  return t.matrix();
}

const TBT * get_const_tbt(const PuppetReader * pr)
{
  if(const Node * r = pr->get_const_root())
  {
    return dynamic_cast<const TBT *>(r->get_child(0));
  }
  return NULL;
}

void update_camera(const Tank & tank, igl::Camera & camera)
{
  using namespace Eigen;
  using namespace std;
  //{
  //  Quaterniond q; 
  //  q.setFromTwoVectors(Vector3d(-1,0,0),tank.vel.normalized());
  //  glMultMatrixd(Affine3d(q).matrix().data());
  //}
  //camera.pan[0] = -tank.pos[0];
  //camera.pan[1] = -tank.pos[1]+PAN_Y_OFFSET;
  //camera.pan[2] = -tank.pos[2];
  if(rotation_type == ROTATION_TYPE_TURRET)
  {
    Quaterniond from = camera.m_rotation_conj;
    camera.m_rotation_conj = follow_turret(tank,from);
  }
}

void update_tank(Tank & tank)
{
  using namespace Eigen;
  using namespace std;
  const double old_vel_norm = tank.vel.norm();
  tank.vel += t_step * tank.acc;
  tank.vel.normalize();
  tank.vel *= old_vel_norm;
  tank.pos += t_step * tank.vel;
  if(tank.control_type == Tank::CONTROL_TYPE_KEYBOARD)
  {
    // don't log this rounding
    tank.Cangle = round(tank.Cangle/Cdangle)*Cdangle;
    tank.Tangle = round(tank.Tangle/Tdangle)*Tdangle;
  }
}

bool out_of_bounds(const Rocket & rocket)
{
  return rocket.pos(1)<0  ||
    fabs(rocket.pos(0)) > PLANE*1.5 ||
    fabs(rocket.pos(2)) > PLANE*1.5;
}

bool is_hit(const Rocket & rocket, const Target & target)
{
  using namespace std;
  using namespace igl;
  //return (target.pos - rocket.pos).norm() < target.radius;
  double t0=-1,t1=-1;
  bool hit = ray_sphere_intersect(
    rocket.pos,(rocket.vel*t_step).eval(),
    target.pos,target.radius,
    t0,t1);
  //cout<<
  //  "hit: "<<hit<<endl;
  if(hit)
  {
    //cout<<"  t0: "<<t0<<endl;
    //cout<<"  t1: "<<t1<<endl;
    return (t0>=0 && t0<=1) || (t1>=0 && t1<=1);
  }
  return false;
}

void update_rocket(Tank & tank, Rocket & rocket, Target & target)
{
  using namespace Eigen;
  using namespace std;
  using namespace igl;

  if(tank.is_firing)
  {
    rocket.vel += t_step * GRAVITY;
    rocket.pos += t_step * rocket.vel;
    if(out_of_bounds(rocket))
    {
      target.fffew.play();
      tank.is_firing = false;
    }else if(is_hit(rocket,target))
    {
      target.pop.play();
      num_hits++;
      tank.is_firing = false;
      if(is_testing)
      {
        lawg<<STR("hit,"<<target.pos.transpose()<<","<<target.radius);
      }
      if(num_hits == num_targets)
      {
        if(is_testing)
        {
          lawg<<"finish";
          if(lawg.save(log_filename))
          {
            cout<<GREENGIN("Saved log to "<<log_filename<<".")<<endl;
          }else
          {
            cout<<REDRUM("Saving log to "<<log_filename<<" failed.")<<endl;
          }
          exit(0);
        }
      }
      target.random_pos();
    }
  }else
  {
    Affine3d t = Affine3d::Identity();
    t.translate(tank.pos);
    t.matrix() = t.matrix() * 
      rigid(tank.Tcenter,tank.Taxis,tank.Tangle) * 
      rigid(tank.Ccenter,tank.Caxis,tank.Cangle);
    rocket.pos = t * RPOS_RELATIVE_REST;
    rocket.vel = t.rotation() * 
      Vector3d(-std::max(rocket.init_vel*std::min((double)tank.charge,7.0),DOUBLE_EPS),0,0);
  }
}

void update_charge(const double new_charge, Tank & tank)
{
  using namespace std;
  using namespace Eigen;
  const double diff_charge = new_charge - (double)tank.charge;

  if(fabs(new_charge-tank.min_charge)>=0.5)
  {
    tank.charged = true;
  }
  if(tank.charged && diff_charge <= -0.5)
  {
    // fire
    if(!tank.is_firing)
    {
      tank.is_firing = true;
      tank.min_charge = new_charge;
    }
  }else
  {
    tank.min_charge = min(tank.min_charge,new_charge);
  }
  tank.charge = round(new_charge);
}

void apply_ao(Tank & tank)
{
  tank.BC.resize(tank.BS.rows(),3);
  tank.TC.resize(tank.TS.rows(),3);
  tank.CC.resize(tank.CS.rows(),3);
  for(int c = 0;c<3;c++)
  {
    tank.BC.col(c).array() = (1.-(double)(ao_enabled)*tank.BS.array()) * tank.Bcolor(c);
    tank.TC.col(c).array() = (1.-(double)(ao_enabled)*tank.TS.array()) * tank.Tcolor(c);
    tank.CC.col(c).array() = (1.-(double)(ao_enabled)*tank.CS.array()) * tank.Ccolor(c);
  }
}

void sync(PuppetReader * pr, Tank & tank)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;

  if(pr->is_attached())
  {
    // Allow puppet to steal attention once
    static bool once = false;
    if(!once && !is_testing)
    {
      tank.control_type = Tank::CONTROL_TYPE_PUPPET;
      once = true;
    }
    pr->sync();
    if(Node * r = pr->get_root())
    {
      r->r_off = Quat(0.5,0.5,0.5,-0.5);
    }
    const TBT * t = get_const_tbt(pr);
    if(tank.control_type == Tank::CONTROL_TYPE_PUPPET && t)
    {
      set_Cangle(tank,lawg, t->angle[1]/180.*igl::PI + 0.5*igl::PI);
      set_Tangle(tank,lawg,-t->angle[0]/180.*igl::PI);
      if(!is_testing && t->color != tank.Bcolor)
      {
        tank.Bcolor = t->color;
        tank.Ccolor = Vector4f(0.8,0.8,0.8,1.0);
        tank.Tcolor = 1.-t->color.array();
        rocket.color = tank.Tcolor;
        apply_ao(tank);
      }
      //const int NUM_CHARGE_LEVELS = 20;
      //const double new_charge = std::max(-(t->angle[2]-45.)/180.*NUM_CHARGE_LEVELS,0.0);
      //update_charge(new_charge,tank);
    }
  }
}

std::string control_type_as_string(const Tank::ControlType & control_type)
{
  switch(control_type)
  {
    case Tank::CONTROL_TYPE_PUPPET:
      return "puppet";
    case Tank::CONTROL_TYPE_MOUSE:
      return "mouse";
    case Tank::CONTROL_TYPE_KEYBOARD:
      return "keyboard";
    default:
      return "unknown";
  }
}

void draw_control_type_name()
{
  using namespace std;
  // True screen space
  glViewport(0,0,main_vp.width,main_vp.height);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0,main_vp.width,0,main_vp.height);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glDisable(GL_LIGHTING);
  glColor4f(1.0,0.8,0.9,1);
  //string str = STR("Hits: "<<num_hits);
  string str = control_type_as_string(tank.control_type);
  //draw_string(0.5*main_vp.width,main_vp.height-18-5,0,str,GLUT_BITMAP_HELVETICA_18);
  glTranslated(0.49*main_vp.width,50,0);
  //glScalef(30,30,30);
  //glScalef(1,1,1);
  glScalef(0.27,0.27,0.27);
  glLineWidth(3);
  for_each(str.begin(),str.end(),bind1st(ptr_fun(&glutStrokeCharacter),GLUT_STROKE_ROMAN));
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void draw_hit_count()
{
  using namespace std;
  // True screen space
  glViewport(0,0,main_vp.width,main_vp.height);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0,main_vp.width,0,main_vp.height);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glDisable(GL_LIGHTING);
  glColor4f(0.9,1.0,0.8,1);
  //string str = STR("Hits: "<<num_hits);
  string str = STR("Target #"<<num_hits+1<<" of "<<num_targets);
  //draw_string(0.5*main_vp.width,main_vp.height-18-5,0,str,GLUT_BITMAP_HELVETICA_18);
  glTranslated(0.45*main_vp.width,150,0);
  //glScalef(30,30,30);
  //glScalef(1,1,1);
  glScalef(0.3,0.3,0.3);
  glLineWidth(3);
  for_each(str.begin(),str.end(),bind1st(ptr_fun(&glutStrokeCharacter),GLUT_STROKE_ROMAN));
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void draw_scene()
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  // Set material properties
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);

  glPushMatrix();
  glTranslated(
    tank.pos(0),
    tank.pos(1),
    tank.pos(2));
  draw_mesh(tank.BV,tank.BF,tank.BN,tank.BC);
  glPushMatrix();
  glMultMatrixd(rigid(tank.Tcenter,tank.Taxis,tank.Tangle).data());
  draw_mesh(tank.TV,tank.TF,tank.TN,tank.TC);
  glPushMatrix();
  glMultMatrixd(rigid(tank.Ccenter,tank.Caxis,tank.Cangle).data());
  //glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
  draw_mesh(tank.CV,tank.CF,tank.CN,tank.CC);
  glPopMatrix();
  glPopMatrix();
  glPopMatrix();

  glPushMatrix();
  glTranslated(
    rocket.pos(0),
    rocket.pos(1),
    rocket.pos(2));
  {
    Quaterniond q; 
    q.setFromTwoVectors(Vector3d(-1,0,0),rocket.vel.normalized());
    glMultMatrixd(Affine3d(q).matrix().data());
  }
  glColor3fv(rocket.color.data());
  draw_mesh(rocket.V,rocket.F,rocket.N);
  glPopMatrix();

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, Target::tex);
//#define SHOW_ALL_TARGETS
#ifdef SHOW_ALL_TARGETS
  for(int t = 0;t<NUM_TARGET_POS;t++)
#endif
  {
    glPushMatrix();
    glTranslated(target.pos(0), 0, target.pos(2));
    {
      Quaterniond q; 
      Vector3d t2t = target.pos-tank.pos;
      t2t(1) = 0;
      t2t.normalize();
      q.setFromTwoVectors(Vector3d(-1,0,0),t2t);
      glMultMatrixd(Affine3d(q).matrix().data());
    }
    glColor4f(1,1,1,1);
    glPushMatrix();
    glScaled(target.thickness, target.pos(1),target.thickness);
    draw_mesh(target.PV,target.PF,target.PN,MatrixXd(),target.PUV);
    glPopMatrix();
    glColor4f(1,1,1,1);
    glPushMatrix();
    glTranslated(0, target.pos(1) , 0);
    glScaled(target.radius,target.radius,target.radius);
    draw_mesh(target.SV,target.SF,target.SN,MatrixXd(),target.SUV);
    glPopMatrix();
    glPopMatrix();
#ifdef SHOW_ALL_TARGETS
    target.random_pos();
#endif
  }
  glBindTexture(GL_TEXTURE_2D, 0);
  glDisable(GL_TEXTURE_2D);
  

}

const float back[4] = {30.0/255.0,30.0/255.0,50.0/255.0,0};
void display()
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;

  const double last_t = display_t;
  display_t = get_seconds();
  t_step = display_t - last_t;

  glClearColor(back[0],back[1],back[2],0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  sync(pr,tank);
  update_rocket(tank,rocket,target);
  update_tank(tank);

  if(main_vp.rotation_is_animating)
  {
    const double ANIMATION_DURATION = 0.5;
    double t = (get_seconds() - main_vp.animation_start_time)/ANIMATION_DURATION;
    if(t > 1)
    {
      t = 1;
      main_vp.rotation_is_animating = false;
    }
    Quaterniond q = main_vp.animation_from_quat.slerp(
      t,main_vp.animation_to_quat).normalized();
    auto & camera = main_vp.camera;
    camera.orbit(q.conjugate());
  }

  update_camera(tank,main_vp.camera);

  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
  const Vector4f BLACK(0,0,0,1);
  const Vector4f WHITE(1,1,1,1);
  Vector4f l_color(0,0,0,1);
  l_color(0) = l_color(1) = l_color(2) = 
    1./(double)(flashlight_pos.cols() + light_pos.cols());
  place_lights(flashlight_pos,0,BLACK,l_color,BLACK);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glViewport( main_vp.x, main_vp.y, main_vp.width, main_vp.height);

  // http://nehe.gamedev.net/tutorial/cool_looking_fog/19001/
  glFogi(GL_FOG_MODE, GL_LINEAR);
  glFogfv(GL_FOG_COLOR, back);            // Set Fog Color
  glFogf(GL_FOG_DENSITY, 0.0001f);              // How Dense Will The Fog Be
  glHint(GL_FOG_HINT, GL_DONT_CARE);          // Fog Hint Value
  glFogf(GL_FOG_START, (main_vp.camera.m_at_dist));             // Fog Start Depth
  glFogf(GL_FOG_END, main_vp.camera.m_far);               // Fog End Depth
  glEnable(GL_FOG);

  push_scene(main_vp.camera);
  place_lights(light_pos,1,BLACK,l_color,BLACK);

  // DRAW FLOOR THAT CATCHES SHADOW
  const float GREY[4] = {0.5,0.5,0.6,1.0};
  const float DARK_GREY[4] = {0.2,0.2,0.3,1.0};
  {
    glPushMatrix();
    glScaled(4,0,4);
    draw_floor(GREY,DARK_GREY,16,16);
    glPopMatrix();
  }
  if(shadows_enabled)
  {
    // CAST SHADOWS FOR EACH LIGHT
    glEnable(GL_STENCIL_TEST);
    glDepthFunc(GL_ALWAYS);
    // Turn off all lights
    int max_lights = 0;
    glGetIntegerv(GL_MAX_LIGHTS,&max_lights);
    vector<bool> light_enabled(max_lights);
    Vector4f global_ambient;
    glGetFloatv(GL_LIGHT_MODEL_AMBIENT,global_ambient.data());
    {
      Vector4f BLACK(0,0,0,1.0);
      glLightModelfv(GL_LIGHT_MODEL_AMBIENT,BLACK.data());
      for(int l = 0; l<max_lights; l++)
      {
        light_enabled[l] = (glIsEnabled(GL_LIGHT0+l));
        glDisable(GL_LIGHT0+l);
      }
    }
    for(int l = 0;l<light_pos.cols();l++)
    {
      // Clear here so the shadows from different lights indeed layer
      glClearStencil(1);
      glStencilMask(0xFF);
      glClear(GL_STENCIL_BUFFER_BIT);
      glStencilFunc(GL_NEVER, 1, 0xff);
      glStencilOp(GL_ZERO, GL_ZERO, GL_ZERO);
      // First draw backwards floor to block shadow from below
      glEnable(GL_CULL_FACE);
      glCullFace(GL_FRONT);
      // DRAW FLOOR THAT CATCHES SHADOW
      {
        glPushMatrix();
        glScaled(4,0,4);
        draw_floor(GREY,DARK_GREY,16,16);
        glPopMatrix();
      }
      glEnable(GL_CULL_FACE);
      glCullFace(GL_BACK);
      glStencilFunc(GL_EQUAL, 1, 0xff);
      const Vector4d pos = light_pos.col(l).cast<double>();
      const Vector4d plane(0,-1,0,0);
      Matrix4d shadow_floor;
      shadow_matrix(plane,pos,shadow_floor);
      glPushMatrix();
      glMultMatrixd(shadow_floor.data());
      // DRAW OBJECTS THAT CAST SHADOWS
      draw_scene();
      glPopMatrix();
    }
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
    // Turn back on lights 
    for(int l = 0; l<max_lights; l++)
    {
      if(light_enabled[l])
      {
        glEnable(GL_LIGHT0+l);
      }
    }
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,global_ambient.data());
  }
   
  // DRAW OBJECTS THAT CAST SHADOWS

  draw_scene();

  pop_scene();
  glDisable(GL_FOG);

  if(tank.control_type == Tank::CONTROL_TYPE_PUPPET)
  {
    Vector4f bg_color(1.,0.,0.,1.);
    if(get_const_tbt(pr))
    {
      bg_color = Vector4f(0.8,0.8,0.8,1);
    }
    draw_genie(genie_vp,pr->get_const_root(),bg_color);
  }


  draw_hit_count();
  draw_control_type_name();

  report_gl_error();

  if(bar_is_visible && !is_testing)
  {
    TwDraw();
  }
  glutSwapBuffers();
  glutPostRedisplay();
}

void mouse_wheel(int wheel, int direction, int mouse_x, int mouse_y)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  if(is_testing)
  {
    return;
  }
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT,viewport);
  if(wheel == 0 && TwMouseMotion(mouse_x, mouse_y))
  {
    static double mouse_scroll_y = 0;
    const double delta_y = 0.125*direction;
    mouse_scroll_y += delta_y;
    TwMouseWheel(mouse_scroll_y);
    return;
  }

  auto & camera = main_vp.camera;
  if(wheel==0)
  {
    // factor of zoom change
    double s = (1.-0.01*direction);
    camera.push_away(s);
  }else
  {
    // Dolly zoom:
    camera.dolly_zoom((double)direction*1.0);
  }
}

void mouse(int glutButton, int glutState, int mouse_x, int mouse_y)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  if(is_testing)
  {
    return;
  }
  bool tw_using = TwEventMouseButtonGLUT(glutButton,glutState,mouse_x,mouse_y);
  switch(glutButton)
  {
    case GLUT_RIGHT_BUTTON:
    case GLUT_LEFT_BUTTON:
    {
      switch(glutState)
      {
        case 1:
          // up
          glutSetCursor(GLUT_CURSOR_INHERIT);
          is_rotating = false;
          break;
        case 0:
          if(!tw_using)
          {
            glutSetCursor(GLUT_CURSOR_CYCLE);
            // collect information for trackball
            is_rotating = true;
            down_camera = main_vp.camera;
            down_x = mouse_x;
            down_y = mouse_y;
          }
        break;
      }
      break;
      // Scroll down
      case GLUT_WHEEL_DOWN:
      {
        mouse_wheel(0,-1,mouse_x,mouse_y);
        break;
      }
      // Scroll up
      case GLUT_WHEEL_UP:
      {
        mouse_wheel(0,1,mouse_x,mouse_y);
        break;
      }
      // Scroll left
      case GLUT_WHEEL_LEFT:
      {
        mouse_wheel(1,-1,mouse_x,mouse_y);
        break;
      }
      // Scroll right
      case GLUT_WHEEL_RIGHT:
      {
        mouse_wheel(1,1,mouse_x,mouse_y);
        break;
      }
    }
  }
}

double clamp(const double f, const double a, const double b)
{
  return std::min(std::max(f,a),b);
}

double clamp_f(const double f, const double a, const double b)
{
  const double cf = clamp(f,a,b);
  return (cf-a)/(b-a);
}

double lerp(const double f, const double a, const double b)
{
  return (1.0-f)*a+f*b;
}

void mouse_move(int mouse_x, int mouse_y)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  if(bar_is_visible)
  {
    bool tw_using = TwMouseMotion(mouse_x,mouse_y);
    if(tw_using)
    {
      return;
    }
  }
  glutSetCursor(GLUT_CURSOR_NONE);
  if(tank.control_type == Tank::CONTROL_TYPE_MOUSE)
  {
    if(is_testing)
    {
      lawg<<STR("mouse,"<<mouse_x<<","<<mouse_y);
    }
    // Directly map mouse position linearly to angles
    set_Tangle(tank,lawg,lerp(clamp_f( 
      mouse_x,main_vp.width*0.1,main_vp.width*0.9), Tmin_angle,Tmax_angle));

    set_Cangle(tank,lawg,lerp(
      clamp_f(main_vp.height - mouse_y,main_vp.height*0.1,main_vp.height*0.9), 
      Cmin_angle,Cmax_angle));
  }
}

void mouse_drag(int mouse_x, int mouse_y)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  if(is_testing)
  {
    return;
  }
  if(bar_is_visible)
  {
    /*bool tw_using =*/ TwMouseMotion(mouse_x,mouse_y);
  }

  if(is_rotating)
  {
    glutSetCursor(GLUT_CURSOR_CYCLE);
    Quaterniond q;
    auto & camera = main_vp.camera;
    switch(rotation_type)
    {
      case ROTATION_TYPE_IGL_TRACKBALL:
      {
        // Rotate according to trackball
        igl::trackball(
          main_vp.width,
          main_vp.height,
          2.0,
          down_camera.m_rotation_conj,
          down_x,
          down_y,
          mouse_x,
          mouse_y,
          q);
          break;
      }
      case ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP:
      {
        // Rotate according to two axis valuator with fixed up vector 
        two_axis_valuator_fixed_up(
          main_vp.width, main_vp.height,
          2.0,
          down_camera.m_rotation_conj,
          down_x, down_y, mouse_x, mouse_y,
          q);
        break;
      }
      case ROTATION_TYPE_TURRET:
      {
        Quaterniond down_q = down_camera.m_rotation_conj;
        Vector3d axis(0,1,0);
        q = down_q;
        q.normalize();
        {
          Vector3d axis(1,0,0);
          const double speed = 2.0;
          if(axis.norm() != 0)
          {
            q = 
              Quaterniond(
                AngleAxisd(
                  M_PI*(mouse_y-down_y)/(double)main_vp.width*speed/2.0,
                  axis.normalized())) * q;
            q.normalize();
          }
        }
        break;
      }
      default:
        break;
    }
    camera.orbit(q.conjugate());
  }
}

void init_tank()
{
  using namespace Eigen;
  using namespace igl;
  using namespace std;
  const string
    target_path = "data/tank/target",
    post = "data/tank/post",
    rocket_path = "data/tank/rocket",
    body = "data/tank/body",
    turret = "data/tank/turret",
    cannon = "data/tank/cannon";
  YImage yimg;
#ifndef NDEBUG
  const bool success =
#endif
    readDMAT(target_path+".V.dmat",target.SV) && 
    readDMAT(target_path+".UV.dmat",target.SUV) && 
    readDMAT(target_path+".F.dmat",target.SF) && 
    readDMAT(post+".V.dmat",target.PV) && 
    readDMAT(post+".UV.dmat",target.PUV) && 
    readDMAT(post+".F.dmat",target.PF) && 
    readDMAT(rocket_path+".V.dmat",rocket.V) && 
    readDMAT(rocket_path+".F.dmat",rocket.F) && 
    readDMAT(body+".V.dmat",  tank.BV) && 
    readDMAT(body+".F.dmat",  tank.BF) && 
    readDMAT(body+".S.dmat",  tank.BS) && 
    readDMAT(turret+".V.dmat",tank.TV) && 
    readDMAT(turret+".F.dmat",tank.TF) && 
    readDMAT(turret+".S.dmat",tank.TS) && 
    readDMAT(cannon+".V.dmat",tank.CV) && 
    readDMAT(cannon+".F.dmat",tank.CF) &&
    readDMAT(cannon+".S.dmat",tank.CS) &&
    yimg.load((target_path + ".png").c_str());
  assert(success);

  per_face_normals(target.PV,target.PF,target.PN);
  per_face_normals(target.SV,target.SF,target.SN);
  per_face_normals(rocket.V,rocket.F,rocket.N);
  per_face_normals(tank.BV,tank.BF,tank.BN);
  per_face_normals(tank.TV,tank.TF,tank.TN);
  per_face_normals(tank.CV,tank.CF,tank.CN);

  if(Target::tex != 0)
  {
    glDeleteTextures(1,&Target::tex);
  }
  glGenTextures(1, &Target::tex);
  glBindTexture(GL_TEXTURE_2D, Target::tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexImage2D(
    GL_TEXTURE_2D, 0, GL_RGBA, 
    yimg.width(), yimg.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, yimg.data());
  glBindTexture(GL_TEXTURE_2D, 0);

  target.pop.load("data/sounds/pop.wav");
  target.fffew.load("data/sounds/fffew.wav");
  tank.pfft.load("data/sounds/pfft.wav");
  apply_ao(tank);
}

void on_exit()
{
  using namespace std;
  using namespace igl;
  static int terminate_once = 0;
  assert(terminate_once == 0);
  target.pop.unload();
  target.fffew.unload();
  tank.pfft.unload();
  if(!alutExit ())
  {
    ALenum error = alutGetError ();
    fprintf (stderr, "2:%s\n", alutGetErrorString (error));
  }
  string path = path_to_executable();
  if(path.find(BINARY_BACKUP_DIR) == string::npos)
  {
    dated_copy(path_to_executable(),BINARY_BACKUP_DIR);
  }
  terminate_once++;
}

#define KEY_FOOT_PEDAL       'b'
#define KEY_TURRET_LEFT  '4'
#define KEY_TURRET_RIGHT '6'
#define KEY_CANNON_UP    '8'
#define KEY_CANNON_DOWN  '5'
#define KEY_FIRE         '0'
void key(unsigned char key, int mouse_x, int mouse_y)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  if(is_testing)
  {
    lawg<<STR("key,"<<key);
  }
  //int mod = glutGetModifiers();
  switch(key)
  {
    // ESC
    case char(27):
       rebar.save(rebar_out_name.c_str());
     //// ^C
     //case char(3):
       exit(0);
    case 'H':
      fire(tank,lawg);
      rocket.pos = target.pos;
      update_rocket(tank,rocket,target);
      break;
    case 'z':
      {
        if(!is_testing)
        {
          Quaterniond q;
          snap_to_canonical_view_quat(main_vp.camera.m_rotation_conj,1.0,q);
          main_vp.camera.orbit(q.conjugate());
          break;
        }
      }
      // map space to 0 only if not testing
    case ' ':
      if(is_testing)
      {
        break;
      }
    // map b to 0
    case KEY_FOOT_PEDAL:
      return ::key(KEY_FIRE,mouse_x,mouse_y);
    case KEY_FIRE:
      fire(tank,lawg);
      break;
    case KEY_CANNON_UP:
      if(tank.control_type == Tank::CONTROL_TYPE_KEYBOARD)
      {
        set_Cangle(tank,lawg,tank.Cangle+Tdangle);
      }
      break;
    case KEY_CANNON_DOWN:
      if(tank.control_type == Tank::CONTROL_TYPE_KEYBOARD)
      {
        set_Cangle(tank,lawg,tank.Cangle-Tdangle);
      }
      break;
    case KEY_TURRET_RIGHT:
      if(tank.control_type == Tank::CONTROL_TYPE_KEYBOARD)
      {
        set_Tangle(tank,lawg,tank.Tangle+Tdangle);
      }
      break;
    case KEY_TURRET_LEFT:
      if(tank.control_type == Tank::CONTROL_TYPE_KEYBOARD)
      {
        set_Tangle(tank,lawg,tank.Tangle-Tdangle);
      }
      break;
    default:
      // Even if bar is hidden, process keys
      if(!TwEventKeyboardGLUT(key,mouse_x,mouse_y))
      {
        cout<<"Unknown key command: "<<key<<" "<<int(key)<<endl;
      }
  }
}


void special(int k, int mouse_x, int mouse_y)
{
  using namespace std;
  switch(k)
  {
    case GLUT_KEY_RIGHT:
      return key(KEY_TURRET_RIGHT,mouse_x,mouse_y);
    case GLUT_KEY_LEFT:
      return key(KEY_TURRET_LEFT,mouse_x,mouse_y);
    case GLUT_KEY_DOWN:
      return key(KEY_CANNON_DOWN,mouse_x,mouse_y);
    case GLUT_KEY_UP:
      return key(KEY_CANNON_UP,mouse_x,mouse_y);
    default:
      cout<<"Unknown special key command: "<<k<<endl;
  }
}

int main(int argc, char * argv[])
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;

  namespace po = boost::program_options;
  po::options_description desc("Options");
  try
  {
    desc.add_options()
      ("help,h","produce help message")
      ("log,l",
        po::value<string>(&log_filename)->implicit_value(""),
        "path to log file")
      ("testing,t",
        po::value<string>(&rebar_in_name)->implicit_value(""),
        "run in testing mode, followed by path to .rbr preferences file");
    po::options_description all("Allowed options");
    all.add(desc);
    
    po::variables_map vm;
    po::parsed_options parsed =
      po::command_line_parser(argc, argv)
        .options(all)
        .allow_unregistered()
        .run();
    po::store(parsed,vm);
    po::notify(vm);
    vector<string> unknown = po::collect_unrecognized(parsed.options, po::include_positional);
    for(auto & u : unknown)
    {
      cout<<"Ignoring unknown option: "<<u<<endl;
    }
    if(vm.count("help"))
    {
      cout << "Usage:"<<endl<<"  ./tank [options]"<<endl<<endl;
      cout << desc << endl;
      return 1;
    }
    if(vm.count("logfile"))
    {
      if(vm["log"].as<string>().empty())
      {
        cerr<<"-l,--log requires argument"<<endl;
        exit(1);
      }
    }
    if(vm.count("testing"))
    {
      is_testing = true;
      if(vm["testing"].as<string>().empty())
      {
        cerr<<"-t,--test requires argument"<<endl;
        exit(1);
      }
    }
  } catch( const exception& e)
  {
    cerr << e.what() << endl << endl;
    cout << desc << endl;
    return 1;
  }

  if(is_testing)
  {
    bar_is_visible = false;
  }
  // print key commands
  cout<<"[Click] and [drag]  Rotate model using trackball."<<endl;
  cout<<"[Z,z]               Snap rotation to canonical view."<<endl;
  cout<<"[^C,ESC]            Exit."<<endl;
  cout<<"[8,5]               Rotate cannon up and down."<<endl;
  cout<<"[4,6]               Rotate turret left and right."<<endl;

  // Init alut
  if (!alutInit (&argc, argv))
  {
    ALenum error = alutGetError ();
    fprintf (stderr, "1:%s\n", alutGetErrorString (error));
    exit (EXIT_FAILURE);
  }

  // Init glut
  glutInit(&argc,argv);

  // Init antweakbar
#ifdef __APPLE__
  glutInitDisplayString( "rgba depth double samples>=8 stencil");
#else
  glutInitDisplayString( "rgba depth double stencil");   // samples>=8 somehow not supported on Kenshi's machines...?
#endif
  if(is_testing)
  {
    glutInitWindowSize(glutGet(GLUT_SCREEN_WIDTH),glutGet(GLUT_SCREEN_HEIGHT));
  }else
  {
    glutInitWindowSize(
      glutGet(GLUT_SCREEN_WIDTH)/2.0,glutGet(GLUT_SCREEN_HEIGHT)/2.0);
  }
  glutCreateWindow(C_STR("tank - ("<<rebar_in_name<<")"));
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(key);
  glutMouseFunc(mouse);
  glutSpecialFunc(special);
  glutMotionFunc(mouse_drag);
  glutPassiveMotionFunc(mouse_move);

  // After window is created
  srand((unsigned)time(NULL));
  init_tank();
  init_puppet();
  {
    bool f = false,_;
    set_is_fullscreen(&f,&_);
  }
  init_anttweakbar(rebar_in_name);
  main_vp.camera.look_at(Vector3d(36,22,0),Vector3d(0,0,0),Vector3d(0,1,0));
  main_vp.animation_from_quat = Quaterniond(1,0,0,0);
  main_vp.animation_to_quat = main_vp.camera.m_rotation_conj;
  main_vp.animation_start_time = get_seconds();

  if(is_testing)
  {
    // Reset log
    lawg = Log<std::string>();
    string control_type_str = control_type_as_string(tank.control_type);
    lawg<<STR("control_type,"<<control_type_str);
  }

  atexit(::on_exit);
  glutMainLoop();


  return 0;
}
