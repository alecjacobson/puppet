#ifndef PUPSKIN_H
#define PUPSKIN_H

//#ifdef __llvm__
//#  error "LLVM sux0rs and you're not allowed to use it, switch to gcc"
//#endif


// Forward declarations
class PuppetReader;
class Node;
#include "Mesh.h"
#include "SkinningMethod.h"
#include "tree_fit.h"

#include "FourView.h"
#include "PosingTest.h"
#include "RigAnimation.h"

#include <igl/opengl2/MouseController.h>
#include <igl/Viewport.h>
#include <igl/Camera.h>
#include <igl/anttweakbar/ReAntTweakBar.h>
#include <igl/bbw/bbw.h>
#include <igl/embree/EmbreeIntersector.h>

#include "GL_include.h"

#include <string>
#include <vector>

const unsigned int PUPSKIN_LOAD_NONE     = 0;
const unsigned int PUPSKIN_LOAD_MODEL    = 1<<0;
const unsigned int PUPSKIN_LOAD_NODES    = 1<<1;
const unsigned int PUPSKIN_LOAD_REBAR    = 1<<2;
const unsigned int PUPSKIN_LOAD_WEIGHTS  = 1<<3;
const unsigned int PUPSKIN_LOAD_SKELETON = 1<<4;
const unsigned int PUPSKIN_LOAD_POSES    = 1<<5;
const unsigned int PUPSKIN_LOAD_TEXTURE  = 1<<6;
const unsigned int PUPSKIN_LOAD_ALL = 
  PUPSKIN_LOAD_MODEL |
  PUPSKIN_LOAD_NODES |
  PUPSKIN_LOAD_REBAR |
  PUPSKIN_LOAD_WEIGHTS | 
  PUPSKIN_LOAD_SKELETON |
  PUPSKIN_LOAD_POSES |
  PUPSKIN_LOAD_TEXTURE;

class PupSkin
{
  // Puppet reader object
  static bool pr_is_attached();
  public:
    typedef std::vector<
      Eigen::Quaterniond,
      Eigen::aligned_allocator<Eigen::Quaterniond> > RotationList;
    static PuppetReader * pr;
    PosingTest posing_test;
    igl::opengl2::MouseController mouse;
    RigAnimation rig_animation;
    enum RotationType
    {
      ROTATION_TYPE_IGL_TRACKBALL = 0,
      ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP = 1,
      NUM_ROTATION_TYPES = 2,
    } rotation_type;
  /////////////////////////////////////////////////////////////////////////
  // Private fields
  /////////////////////////////////////////////////////////////////////////
  private:
    // UI
    bool is_rotating;
    double down_x,down_y,hover_x,hover_y;
    // Picking window "radius"
    int pick_s;
    // Whether should pick on hover for next draw
    bool hover_next;
    int render_count;
    bool render_to_png_on_next;
    double fps;
    const double min_zoom;
    const double max_zoom;
    bool zoom_lock, angle_lock, is_fullscreen;
    bool should_init_lbs;
    bool mirror_mode;
    SkinningMethod skinning_method;
    GLuint background_tex_id;
  public:
    FourView four_view;
  private:
    Eigen::Matrix<float,4,Eigen::Dynamic> flash_lights;
    Eigen::Matrix<float,4,Eigen::Dynamic> scene_lights;
    bool mesh_is_visible;
    Mesh m;
    Mesh m_def;
    igl::embree::EmbreeIntersector ei;
  public:
    // Skinning weights for m
    Eigen::MatrixXd W;
    // LBS matrix correspoinding to V and W
    Eigen::MatrixXd M;
    // rig Skeleton
    Eigen::MatrixXd C;
    Eigen::MatrixXi BE;
    Eigen::VectorXi P,RP;
    // mapping of node transformations to rig
    Eigen::VectorXi nodes_to_rig;
  private:
    bool auto_scale;
  public:
    bool auto_fit_activated;
  private:
    TreeFitParams tf_params, tf_last_tf_params;
    Eigen::MatrixXd tf_last_O;
    Eigen::MatrixXd tf_last_bc;
    // Skinning on the CPU
    bool cpu_lbs; 
  public:
    bool was_cpu_lbs; 
  private:
    bool pick_on_mesh;
  public:
    // TODO: should reveal bar through getter/setter
    igl::anttweakbar::ReTwBar rebar;
  private:
    const double min_angle;
    bool skeleton_visible; 
    bool rig_skeleton_visible; 
    bool rig_mapping_visible; 
    bool drag_arrows_visible;
    bool ui_on_top;
    bool floor_visible;
    // Seconds since birth
    double age;
  public:
    // Bounded biharmonic weights solve data
    //igl::BBWData bbw_data;
    int bbw_max_iter;
    int genie_width;
    int genie_height;
    bool genie_is_visible;
    enum SkeletonStyleType
    {
      SKELETON_STYLE_TYPE_3D = 0,
      SKELETON_STYLE_TYPE_VECTOR_GRAPHICS = 1,
      NUM_SKELETON_STYLE_TYPES = 2,
    } skel_style;
    enum ControlType
    {
      CONTROL_TYPE_PUPPET = 0,
      CONTROL_TYPE_MOUSE = 1,
      NUM_CONTROL_TYPES = 2
    } control_type;
    bool correct_relative_frames;
    PupSkin();
    ~PupSkin();
    /////////////////////////////////////////////////////////////////////////
    // Initializers and de-initializers
    /////////////////////////////////////////////////////////////////////////
    // Called when desiring termination of the main program
    void terminate(void);
    // Save everything
    //
    // Inputs:
    //   prefix  file prefix
    bool save(const std::string prefix);
    // Initialize variables
    //
    // Inputs:
    //   argc  number of character arguments
    //   argv  point to arguements
    // Returns true only on success
    bool init(int argc, char * argv[]);
  private:
    // Initialize AntTweakBar and reload from rebar file
    void init_anttweakbar();
    void add_display_group_to_anttweakbar();
    void add_node_group_to_anttweakbar();
    void add_mesh_group_to_anttweakbar();
    void add_skinning_group_to_anttweakbar();
    // Initialize puppet
    bool init_puppet();
  public:
    // Load mesh, weights, rebar, nodes from 
    //   [prefix]-model.{off|obj}
    //   [prefix]-weights.dmat
    //   [prefix]-nodes.xml
    //   [prefix]-rebar.rbr
    // Inputs:
    //   prefix  path prefix
    //   Optional:
    //     load only these types
    // Returns load type
    unsigned int load(const std::string prefix, const unsigned int types = PUPSKIN_LOAD_ALL);
    // Load a mesh from a .obj or .off file
    //
    // Inputs:
    //   filename  path to .obj or .off file containing triangle mesh
    // Returns whether mesh loaded correctly
    bool load_mesh(const std::string filename);
    // Load weights from a .dmat file.
    //
    // Inputs:
    //   filename  path to .dmat file containing weights
    // Returns whether weights loaded correctly
    bool load_weights(const std::string filename);
    // Load skeleton from a .mat file.
    //
    // Inputs:
    //   filename  path to .mat file containing skeleton
    // Returns whether skeleton loaded correctly
    bool load_skeleton(const std::string filename);
    // Initialize LBS matrix, etc. Must be called *after* first draw.
    bool init_lbs();
  public:
    // Depending on whether a puppet reader is being used return the puppet
    // readers root node or the in class one
    //
    // USE RESPONSIBLY OR USE get_const_root()
    Node * get_root();
    const Node * get_const_root() const;
    const Mesh & get_const_mesh() const;
    Mesh & get_mesh();
    // Sets and gets cpu_lbs. This variable should always be set through this
    // function
    bool get_cpu_lbs() const;
    void set_cpu_lbs(const bool v);
    /////////////////////////////////////////////////////////////////////////
    // Drawing
    /////////////////////////////////////////////////////////////////////////
  private:
    // Push the scene view matrix to OpenGL stack
    void push_scene(const AugViewport & vp) const;
    double y_boost() const;
    // Pop the scene view matrix to OpenGL stack
    void pop_scene() const;
    bool deform_mesh();
    // Deform the current rig skeleton with the current transformations
    //
    // Outputs:
    //   CT  #BE*2 by 3 list of deformed joint positions
    //   BET  #BE by 2 list of bone edge indices (maintains order)
    void deform_rig_skeleton(
      Eigen::MatrixXd & CT,
      Eigen::MatrixXi & BET);
    // Compute transformations corresponding to each bone in the rig skeleton
    //
    // Outputs:
    //   vQ  #BE list of quaternions
    //   vT  #BE list of translations
    // Returns on success
    bool rig_transformations(
      RotationList & vQ,
      std::vector<Eigen::Vector3d> & vT);
    // Releative rotations used for forward kinematics to pose current rig.
    //
    // Outputs:
    //   vQ  #BE list of quaternions
    bool rig_pose_from_control(RotationList & vdQ);
    // Could be from animation
    bool rig_pose(RotationList & vdQ);
    // Output:
    //   T  #BE*4 by 3 stack of transpose transformations
    bool rig_transformations(Eigen::MatrixXd & T);
    // Main draw routine, assume view has been set up
    void draw_objects(const int vp);
    void draw_floor(const double floor_scale, const double floor_offset);
    // Main draw single pass, assumes transparency (Culling and depth func) for this pass has been set up
    //
    // Inputs:
    //   i  pass number
    void pass(const int i);
    // Draw mesh (without any color setting blah blah)
    void draw_mesh();
    // Draw the puppet nodes
    void draw_puppet(const bool show_controls);
    // Draw UI: drag arrows, skeletons, widgets etc.
    void draw_ui();
    // Called when trying to determine hovered upon objects during (or on the draw
    // right after) a passive mouse move event
    //
    // Inputs:
    //   hover_x  x position of hovering mouse
    //   hover_y  y position of hovering mouse
    // Returns whether any object was set to being hovered upon
    bool hover(const int hover_x, const int hover_y);
    // Main display function, clears the screen and draws the current screen and
    // GUI elements
  public:
    void display();
  private:
    // Automatically fit nodes according to drag arrows using tree_fit
    bool auto_fit();
    // Draw a little "genie" a small rendering of the puppet in the upper left
    // corner of the given viewport.
    //
    // Inputs:
    //   vp  Viewport to be rendered in
    void draw_genie(const AugViewport & vp);
    // Draw the scene elements, sets up projection and modelview via push_scene
    // etc.
    //
    // Inputs:
    //   render_mode  rendering mode, either GL_SELECT or GL_RENDER
    void draw_scene();
  public:
    // Called whenever the window is reshaped (resized)
    //
    // Inputs:
    //   width  new window width in pixels
    //   height new window height in pixels
    void reshape(int width, int height);
    /////////////////////////////////////////////////////////////////////////
    // User-interface
    /////////////////////////////////////////////////////////////////////////
    // Called whenever a key is pressed (down)
    // Inputs:
    //   key  uchar id of key pressed
    //   mouse_x  x position of mouse
    //   mouse_y  y position of mouse
    bool key(unsigned char key, int mouse_x, int mouse_y);
    // Called whenever a mouse button is pressed/released
    // Inputs:
    //   glutButton  id of which button was pressed/released
    //   glutState  state of button (e.g. pressed or released)
    //   mouse_x  x position of mouse
    //   mouse_y  y position of mouse
    void mouse_glut(int glutButton, int glutState, int mouse_x, int mouse_y);
    // Called after passing to anttweakbar
    void mouse_down(int glutButton, int mouse_x, int mouse_y);
    void mouse_up(int glutButton, int mouse_x, int mouse_y);
    // Called whenever the mouse is moved with a button pressed
    // Inputs:
    //   mouse_x  x position of mouse
    //   mouse_y  y position of mouse
    void mouse_drag(int mouse_x, int mouse_y);
    // Called whenever the mouse is moved without a button pressed
    // Inputs:
    //   mouse_x  x position of mouse
    //   mouse_y  y position of mouse
    void mouse_move(int mouse_x, int mouse_y);
    // Called whenever the mouse wheel is changed
    // Inputs:
    //   wheel  number of wheel that was changed
    //   direction  +1/-1 for up/down
    //   mouse_x  x position of mouse
    //   mouse_y  y position of mouse
    void mouse_wheel(int wheel, int direction, int x, int y);
  private:
    // Called when trying to pick drawable objects from a given mouse click
    // (location).
    //
    // Inputs:
    //   mouse_x  x position of mouse
    //   mouse_y  y position of mouse
    // Outputs:
    //   pixel  color under mouse (shoud be preallocated)
    // Returns true only if pixel[4] != 0
    bool pick(int mouse_x, int mouse_y, float * pixel);
  public:
    void init_rig_animation_is_listening();
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
