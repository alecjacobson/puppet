//#define DRAW_PROFILING
#include "PupSkin.h"

// AntTweakbar callbacks
#include "Callbacks.h"

// Functions
#include "diffuse_material.h"
#include "process_hits.h"
#include "draw_string.h"
#include "draw_genie.h"
#include "shift_color.h"
#include "find_selected.h"
#include "draw_axis.h"
#include "set_node_scale.h"
#include "shadow_matrix.h"
#include "dim_lights.h"
#include "add_puppetreader_anttweakbar.h"
#include "place_lights.h"
#include "quattrans2affine.h"
#include "throw_at.h"
#include "list_affine_to_stack.h"
#include "match_skeletons.h"
#include "draw_skeleton_matching.h"
#include "filter_prefix.h"
#include "set_is_fullscreen.h"

// Classes
#include "Node.h"
#include "NodeCallbacks.h"
#include "TBT.h"
#include "Root.h"
#include "Splitter.h"
#include "PuppetReader.h"
#include "PuppetReaderCallbacks.h"

#include <igl/REDRUM.h>
#include <igl/file_exists.h>
#include <igl/deform_skeleton.h>

#include <igl/ONE.h>
#include <igl/ZERO.h>
#include <igl/EPS.h>
#include <igl/trackball.h>
#include <igl/quat_to_mat.h>
#include <igl/quat_mult.h>
#include <igl/quat_conjugate.h>
#include <igl/canonical_quaternions.h>
#include <igl/opengl/render_to_tga.h>
#include <igl/png/render_to_png.h>
#include <igl/png/texture_from_png.h>
#include <igl/png/render_to_png_async.h>
#include <igl/Camera.h>
#include <igl/PI.h>
#include <igl/ZERO.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/get_seconds.h>
#include <igl/get_seconds_hires.h>
#include <igl/mat_to_quat.h>
#include <igl/STR.h>
#include <igl/C_STR.h>
#include <igl/basename.h>
// Mesh related
#include <igl/read_triangle_mesh.h>
#include <igl/readOBJ.h>
#include <igl/pathinfo.h>
#include <igl/writeOFF.h>
#include <igl/writeDMAT.h>
#include <igl/readDMAT.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_corner_normals.h>
#include <igl/material_colors.h>
#include <igl/opengl2/draw_beach_ball.h>
#include <igl/opengl2/draw_floor.h>
#include <igl/opengl2/draw_point.h>
#include <igl/opengl2/draw_skeleton_vector_graphics.h>
#include <igl/opengl2/draw_skeleton_3d.h>
#include <igl/opengl2/model_proj_viewport.h>
//#define IGL_HEADER_ONLY
//#define EXTREME_VERBOSE
#include <igl/opengl2/project.h>
#include <igl/embree/unproject_in_mesh.h>
#ifdef WITH_MATLAB
#  include <igl/matlab/MatlabWorkspace.h>
#endif
#include <igl/matlab_format.h>
#include <igl/anttweakbar/ReAntTweakBar.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/lbs_matrix.h>
#include <igl/forward_kinematics.h>

#include "GLUT_include.h"

#include <boost/program_options.hpp>

#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <algorithm>

#include <deque>
#include <map>

PuppetReader * PupSkin::pr = NULL;


PupSkin::PupSkin():
  posing_test(),
  mouse(),
  rig_animation(),
  rotation_type(ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP),
  is_rotating(false),
  down_x(-1), down_y(-1), hover_x(-1), hover_y(-1),
  pick_s(1),
  hover_next(false),
  render_count(0),
  render_to_png_on_next(false),
  export_count(0),
  exporting_to_png(false),
  fps(0),
  min_zoom(0.01), max_zoom(10.0),
  zoom_lock(false),angle_lock(false),is_fullscreen(false),
  should_init_lbs(false),
  mirror_mode(false),
  skinning_method(SKINNING_METHOD_LBS),
  background_tex_id(0),
  prefixes(),
  prefix_in_use(0),
  flash_lights(4,1),
  scene_lights(4,2),
  mesh_is_visible(true),
  m(),
  m_def(),
  ei(),
  W(),
  M(),
  C(),BE(),P(),RP(),nodes_to_rig(),
  auto_scale(true),
  auto_fit_activated(false),
  tf_params(),
  tf_last_tf_params(),
  tf_last_O(),
  tf_last_bc(),
  cpu_lbs(false),
  was_cpu_lbs(false),
  pick_on_mesh(false),
  rebar(),
  min_angle(20),
  skeleton_visible(true),
  rig_skeleton_visible(true),
  rig_mapping_visible(true),
  drag_arrows_visible(true),
  ui_on_top(true),
  floor_visible(true),
  age(-1),
  //bbw_data(),
  bbw_max_iter(6),
  genie_width(200),
  genie_height(200),
  genie_is_visible(true),
  skel_style(SKELETON_STYLE_TYPE_VECTOR_GRAPHICS),
  control_type(CONTROL_TYPE_PUPPET),
  correct_relative_frames(true)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  flash_lights.col(0) = Vector4f(-0.1,-0.1,-1,0);
  scene_lights.col(0) = Vector4f(0.3,-0.4,-0.7,0);
  scene_lights.col(1) = Vector4f(-0.3,-0.4,-0.7,0);
  age = get_seconds();
}

PupSkin::~PupSkin()
{
  using namespace std;
  cout<<BLUERUM("begin ~PupSkin()")<<endl;
  terminate();
  cout<<BLUERUM("end ~PupSkin()")<<endl;
}

void PupSkin::terminate(void)
{
  using namespace std;
  static int terminate_once = 0;
  assert(terminate_once == 0);
  cout<<BLUERUM("terminate()")<<endl;
  save("autosave");
  rebar.TwTerminate();
  Node::bar = NULL;
  if(pr)
  {
    PupSkin::pr->close();
    delete PupSkin::pr;
    PupSkin::pr = NULL;
  }

  //exit(0);
  terminate_once++;
}

#define REBAR_NAME "-rebar"
#define NODES_NAME "-nodes"
#define MESH_NAME  "-model"
#define WEIGHTS_NAME "-weights"
#define SKELETON_NAME "-skeleton"
#define POSES_NAME "-poses"
#define TEXTURE_NAME  "-texture"
bool PupSkin::save(const std::string prefix)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  bool ret = true;
  // Save bar
  if(rebar.save((prefix+string(REBAR_NAME) + ".rbr").c_str()))
  {
    cout<<GREENGIN("Saved "<<prefix+REBAR_NAME<<".rbr successfully.")<<endl;
  }else
  {
    cout<<REDRUM("Saving "<<prefix+REBAR_NAME<<".rbr failed.")<<endl;
    ret = false;
  }
  
  // Save Nodes
  if(Node::writeXML(prefix+STR(NODES_NAME<<".xml"),get_const_root()))
  {
    cout<<GREENGIN("Saved "<<prefix+NODES_NAME<<".xml successfully.")<<endl;
  }else
  {
    cout<<REDRUM("Saving "<<prefix+NODES_NAME<<".xml failed.")<<endl;
    ret = false;
  }
  // Save mesh
  if(m.getF().size() > 0)
  {
    if(writeOFF(STR(prefix+MESH_NAME<<".off"),m.getV(),m.getF()))
    {
      cout<<GREENGIN("Saved "<<prefix+MESH_NAME<<".off successfully.")<<endl;
    }else
    {
      cout<<REDRUM("Saving "<<prefix+MESH_NAME<<".off failed.")<<endl;
      ret = false;
    }
  }
  // Save weights
  if(writeDMAT(STR(prefix+WEIGHTS_NAME<<".dmat"),W))
  {
    cout<<GREENGIN("Saved "<<prefix+WEIGHTS_NAME<<".dmat successfully.")<<endl;
  }else
  {
    cout<<REDRUM("Saving "<<prefix+WEIGHTS_NAME<<".dmat failed.")<<endl;
    ret = false;
  }
  // Save skeleton
#ifdef WITH_MATLAB
  {
    igl::matlab::MatlabWorkspace mw;
    MatrixXd C_rel = C;
    m.relative_to_mesh(C_rel);
    mw.save(C_rel,"C");
    mw.save_index(BE,"BE");
    mw.save_index(RP,"RP");
    mw.save_index(P,"P");
    if(mw.write(prefix + SKELETON_NAME + ".mat"))
    {
      cout<<GREENGIN("Saved "<<prefix+SKELETON_NAME<<".mat successfully.")<<
        endl;
      ret = false;
    }else
    {
      cout<<REDRUM("Saving "<<prefix+SKELETON_NAME<<".mat failed.")<<endl;
      ret = false;
    }
  }
#endif
  // Save poses
  {
    string filename = prefix+POSES_NAME+".dmat";
    if(posing_test.save_poses(filename))
    {
      cout<<GREENGIN("Saved "<<filename<<" successfully.")<<
        endl;
      ret = false;
    }else
    {
      cout<<REDRUM("Saving "<<filename<<" failed.")<<endl;
      ret = false;
    }
  }
  return ret;
}

bool PupSkin::init(int argc, char * argv[])
{
  using namespace std;
  using namespace igl;
  // Turn off key repeat for user study

  // Default params
  string prefixes_str = "autosave";
  // command line options
  namespace po = boost::program_options;
  po::options_description desc("General options");
  po::options_description hidden("Hidden options");

  string force_rebar_in_name;
  try
  {
    // Declare the supported options.
    desc.add_options()
      ("help,h", "produce help message")
      ("list-commands", "list serial commands and quit")
      ("testing,t",
        po::value<string>(&force_rebar_in_name)->implicit_value(""),
        "run in testing mode, followed by path to .rbr preferences file")
      ("log,l",
        po::value<string>(&posing_test.m_log_filename)->implicit_value(""),
        "path to log file")
      ("prefix,p",po::value<string>(),
         "project path prefix(es) (comma separated) {\"./autosave\"}")
    ;
    //hidden.add_options()
    //  ("psn", po::value<string>(),"ignored")
    //;
    po::options_description all("Allowed options");
    all.add(desc).add(hidden);
    
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(all).allow_unregistered().run(), vm);
    //po::store(po::command_line_parser(argc, argv).options(all).extra_parser(parse_psn).run(), vm);
    po::notify(vm);

    if(vm.count("help"))
    {
      cout << desc << "\n";
      return false;
    }
    if(vm.count("list-commands"))
    {
      cout << ifstream("data/doc/commands.txt").rdbuf() << endl;
      return false;
    }
    if(vm.count("prefix"))
    {
      prefixes_str = vm["prefix"].as<string>();
    }

  }
  catch( const std::exception& e)
  {
    std::cerr << e.what() << std::endl << std::endl;
    std::cout << desc << std::endl;
    return false;
  } 

  if(!init_puppet())
  {
    cout<<REDRUM("Puppet did not initialize properly...")<<endl;
#ifdef __APPLE__
    cout<<
      "Perhaps you ought to issue:"<<endl<<
      "    sudo kextunload /System/Library/Extensions/FTDIUSBSerialDriver.kext/"<<
      endl<<"or"<<endl<<
      "    sudo kextunload -b com.apple.driver.AppleUSBFTDI"<<endl;
#endif
    if(pr_is_attached())
    {
      pr->set_root(new Root());
    }
  }else{
    cout<<GREENGIN("Puppet initialized properly...")<<endl;
  }
  init_anttweakbar();
  // split on commas, http://stackoverflow.com/a/11719617/148668
  {
    std::istringstream ss(prefixes_str);
    std::string token;
    while(std::getline(ss, token, ','))
    {
       prefixes.push_back(token);
    }
  }

  assert(prefixes.size() >= 0);
  // Should come after init_anttweakbar()
  int loaded = load(prefixes[prefix_in_use],PUPSKIN_LOAD_ALL);
  if(force_rebar_in_name != "")
  {
    if(rebar.load(force_rebar_in_name.c_str()))
    {
      cout<<GREENGIN("Reloaded "<<force_rebar_in_name<<" successfully.")<<endl;
    }else
    {
      cout<<REDRUM("Reloading "<<force_rebar_in_name<<" failed.")<<endl;
    }
    posing_test.m_is_testing = true;
    mouse.root_enabled() = false;
  }
  // Try to load remaining from autosave
  loaded = loaded ^ PUPSKIN_LOAD_ALL;
  load("autosave",loaded);

  next_background();

  return true;
}

void PupSkin::init_anttweakbar()
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  using namespace igl::anttweakbar;
  // Initialize anttweakbar library
  TwInit(TW_OPENGL, NULL);
  rebar.TwNewBar("bar");
  // Hook up bar to Node elements
  Node::bar = rebar.bar;
#ifdef _RELEASE
  TwDefine("bar label='Release' size='200 550' text=light alpha='200' color='68 68 68'");
#else
  TwDefine("bar label='Debug' size='200 550' text=light alpha='200' color='250 0 0'");
#endif
  TwDefine(" GLOBAL help='This is a prototyping tool for the puppet "
    " interface.' ");
  // Connect up strings
  // must be called once (just after TwInit for instance)
  TwCopyStdStringToClientFunc(CopyStdStringToClient); 
  rebar.TwAddVarRO("fps",TW_TYPE_DOUBLE,&fps, "label='fps' ");
  add_display_group_to_anttweakbar();
  add_node_group_to_anttweakbar();

  if(PupSkin::pr_is_attached())
  {
    add_puppetreader_anttweakbar(pr,rebar);
    rebar.TwSetParam( "PuppetReader", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
  }

  add_mesh_group_to_anttweakbar();
  add_skinning_group_to_anttweakbar();

  // Picking
  rebar.TwAddVarRW("show_window",TW_TYPE_BOOLCPP,&Pickle::is_picking,
    "group='picking' "
    "help='Force renderer to show picking colors' "
    "key='P' "
    );
  rebar.TwAddVarRW("pick_s",TW_TYPE_INT32,&pick_s,
    "group='picking' "
    "label='size' "
    "help='Radius of picking window' "
    "min=1 max=1000 "
    );
  rebar.TwSetParam( "picking", "opened", TW_PARAM_INT32, 1, &INT_ZERO);

  // Viewports
  four_view.add_to_reanttweakbar(rebar);
  // Posing
  posing_test.add_to_reanttweakbar(rebar);
  {
    const auto & push_poseCB = [](void *clientData)
    {
      PupSkin & p = *static_cast<PupSkin*>(clientData);
      RotationList vdQ;
      if(p.rig_pose_from_control(vdQ))
      {
        p.posing_test.push_pose(vdQ);
      }
    };
    rebar.TwAddButton("posing_test_push_pose", push_poseCB, this,
      "label='push pose' group='Test'");
    const auto & pop_poseCB = [](void *clientData)
    {
      PupSkin & p = *static_cast<PupSkin*>(clientData);
      p.posing_test.pop_pose();
    };
    rebar.TwAddButton("posing_test_pop_pose", pop_poseCB, this,
      "label='pop pose' group='Test'");

  }

  // Light group
  for(int l = 0;l<flash_lights.cols();l++)
  {
    rebar.TwAddVarRW(C_STR("flash_light_"<<l),TW_TYPE_DIR3F,flash_lights.data()+4*l,
      C_STR("group='Lights' "
      "label='flash light "<<l<<"' "
      "open "
      "help='Orient light vector ' ")
      );
  }
  for(int l = 0;l<scene_lights.cols();l++)
  {
    rebar.TwAddVarRW(C_STR("scene_light_"<<l),TW_TYPE_DIR3F,scene_lights.data()+4*l,
      C_STR("group='Lights' "
      "label='scene light "<<l<<"' "
      "open "
      "help='Orient light vector ' ")
      );
  }
  rebar.TwSetParam( "Lights", "opened", TW_PARAM_INT32, 1, &INT_ZERO);

  TwType RotationTypeTW = igl::anttweakbar::ReTwDefineEnumFromString("RotationType","igl_trackball,two_axis_fixed_up");
  rebar.TwAddVarCB("rotation_type", RotationTypeTW,
    set_rotation_typeCB,get_rotation_typeCB,this,
    "group=Camera keyIncr=] keyDecr=[ help='type of rotation interface'");
  rebar.TwAddButton("xy", &xyCB, this,
    "group='Camera' "
    "label='xy' "
    "key='z' "
    "help='View xy plane' "
    );
  rebar.TwAddButton("xz", &xzCB, this,
    "group='Camera' "
    "label='xz' "
    "key='y' "
    "help='View xz plane' "
    );
  rebar.TwAddButton("yz", &yzCB, this,
    "group='Camera' "
    "label='yz' "
    "key='x' "
    "help='View yz plane' "
    );
  rebar.TwAddButton("ortho", &orthoCB, this,
    "group='Camera' "
    "label='orthographic' "
    "key='o' "
    "help='Toggle orthographic igl::opengl2::projection' "
    );
  rebar.TwAddButton("snap",&snapCB,this,
    "group='Camera' "
    "label='snap' "
    "key='Z' "
    "help='snap camera to canonical view' "
    );
  rebar.TwSetParam( "Camera", "opened", TW_PARAM_INT32, 1, &INT_ZERO);

  rig_animation.add_to_reanttweakbar(rebar);
  {
    const auto & push_poseCB = [](void *clientData)
    {
      PupSkin & p = *static_cast<PupSkin*>(clientData);
      RotationList vdQ;
      if(p.rig_pose_from_control(vdQ))
      {
        p.rig_animation.push_pose(vdQ);
      }
    };
    rebar.TwAddButton("rig_animation_push_pose",push_poseCB,this,
      "label='push keyframe' group=Animation key=A");
    const auto & start_playingCB = [](void * clientData)
    {
      PupSkin & p = *static_cast<PupSkin*>(clientData);
      p.start_playing();

    };
    rebar.TwAddButton("rig_animation_start_playing",start_playingCB,
      this,
      "label='play' group='Animation' key=a");
    rebar.TwAddButton("rig_animation_start_exporting",
      [](void *clientData)
      {
        PupSkin & p = *static_cast<PupSkin*>(clientData);
        p.exporting_to_png = true;
        p.start_playing();
      },
      this,
      "label='export' group='Animation' key=e");

    const auto & get_is_recordingCB = [](void * v, void * clientData)
    {
      PupSkin & p = *static_cast<PupSkin*>(clientData);
      *(bool*)v = p.rig_animation.is_recording();
    };
    const auto & set_is_recordingCB = [](const void * v, void * clientData)
    {
      PupSkin & p = *static_cast<PupSkin*>(clientData);
      const bool & b = *(const bool *)v;
      if(p.rig_animation.is_recording() == b)
      {
        return;
      }
      if(b)
      {
        RotationList vdQ;
        if(p.rig_pose_from_control(vdQ))
        {
          p.rig_animation.start_recording(vdQ);
        }
      }else
      {
        p.rig_animation.set_all_is_recording(false);
      }
    };
    rebar.TwAddVarCB("rig_animation_is_recording",TW_TYPE_BOOLCPP,
      set_is_recordingCB,
      get_is_recordingCB,this,
      "label='recording' group='Animation' key=@");
  }

  rebar.TwAddButton("living_will", &living_willCB, this,
    "label='living will' key='w' help='Save everything I know.' ");
}

void PupSkin::add_display_group_to_anttweakbar()
{
  using namespace igl;
  rebar.TwAddVarRW("floor",TW_TYPE_BOOLCPP,&this->floor_visible,
    "group='Display' "
    "label='floor' "
    "help='Show floor and shadows' ");
  rebar.TwAddVarRW("genie_is_visible",TW_TYPE_BOOLCPP,&genie_is_visible,
    "group='Display' "
    "label='show genie' "
    "key='g' "
    "help='Show \"genie\" device in corner.' ");
  rebar.TwAddVarRW("genie_width",TW_TYPE_INT32,&genie_width,
    "group='Display' "
    "label='genie box width' "
    "help='Viewport width for genie' ");
  rebar.TwAddVarRW("genie_height",TW_TYPE_INT32,&genie_height,
    "group='Display' "
    "label='genie box height' "
    "help='Viewport height for genie' ");
  rebar.TwAddVarRW("ui_on_top",TW_TYPE_BOOLCPP,&ui_on_top,
    "group='Display' "
    "label='UI on top' "
    "key=O "
    "help='display ui on top of mesh' ");
  rebar.TwAddVarRW("zoom_lock",TW_TYPE_BOOLCPP,&zoom_lock,
   "group=Display label='lock zoom' ");
  rebar.TwAddVarRW("angle_lock",TW_TYPE_BOOLCPP,&angle_lock,
    "group=Display label='lock angle'");
  rebar.TwAddVarRW("mirror_mode",TW_TYPE_BOOLCPP,&mirror_mode,
   "group=Display label='mirror'");
  rebar.TwAddVarCB("is_fullscreen",TW_TYPE_BOOLCPP,
    set_is_fullscreen,get_is_fullscreen,&is_fullscreen,
    "group=Display key=F");

  rebar.TwAddButton("next_background",
    [](void * client_data)
    {
      PupSkin & p = *static_cast<PupSkin*>(client_data);
      p.next_background();
    },this,
    "group=Display label='Next background' key=ret");

  rebar.TwAddButton("next_project",
    [](void * client_data)
    {
      PupSkin & p = *static_cast<PupSkin*>(client_data);
      p.next_project();
    },this,
    "group=Display label='Next project' key=space");

  rebar.TwSetParam( "Display", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
}

void PupSkin::add_node_group_to_anttweakbar()
{
  using namespace igl;
  rebar.TwAddVarRW("node_user_scale",TW_TYPE_DOUBLE,&Node::user_scale,
    "group='Node' "
    "label='scale' "
    "step=0.01 "
    "min=0 max=2 "
    );
  rebar.TwAddVarRW("auto_scale",TW_TYPE_BOOLCPP,&auto_scale,
    "group='Node' "
    "label='scale automatically' "
    "help='Automatically scale nodes according to arrangment' "
    );
  rebar.TwAddVarCB("auto_fit_activated",
    TW_TYPE_BOOLCPP,
    set_auto_fit_activatedCB,get_auto_fit_activatedCB,this,
    "group='Node' "
    "label='auto fit' "
    "key='f' "
    "help='Automatically fit node locations according to drag arrows' "
    );
  rebar.TwAddVarRW("node_drag_along_axis",TW_TYPE_BOOLCPP,
    &Node::drag_along_axis,
    "group='Node' "
    "label='drag_along_axis' "
    "help='Snap dragging to move node along incoming bone axis' ");
  rebar.TwAddVarRW("node_show_bind",TW_TYPE_BOOLCPP,&Node::show_bind,
    "group='Node' "
    "label='show bind' "
    "key='b' "
    "help='Show nodes as they were at bind time' ");
  rebar.TwAddVarRW("node_color_parts",TW_TYPE_BOOLCPP,&Node::color_parts,
    "group='Node' "
    "label='color parts' "
    "key='c' "
    "help='Color separate node parts' ");
  rebar.TwAddVarRW("Node_visible",TW_TYPE_BOOLCPP,&Node::visible,
    "group='Node' "
    "label='visible' "
    "key='n' "
    "help='Show nodes' ");
  rebar.TwAddVarRW("drag_arrows_visible",TW_TYPE_BOOLCPP,&drag_arrows_visible,
    "group='Node' "
    "label='show drag arrows' "
    "key='u' "
    "help='Show drag arrows ' ");
  rebar.TwAddButton("node_rig",&rigCB,this,
    "group='Node' "
    "label='rig' "
    "key='CTRL+B' "
    "help='Construct rig from current nodes' ");
  rebar.TwAddButton("node_bind",&bindCB,this,
    "group='Node' "
    "label='bind' "
    "key='B' "
    "help='Bind nodes to current rig' ");
  TwEnumVal RotationControlTypeEV[NUM_ROTATION_CONTROL_TYPES] = 
  {
    {ROTATION_CONTROL_TYPE_TRACKBALL,"trackball"},
    {ROTATION_CONTROL_TYPE_AXIS,"axis"},
  };
  TwType RotationControlTypeTW = 
    igl::anttweakbar::ReTwDefineEnum(
        "RotationControlType", 
        RotationControlTypeEV, 
        NUM_ROTATION_CONTROL_TYPES);
  rebar.TwAddVarRW(
      "node_rotation_control_type",
      RotationControlTypeTW,
      &Node::rotation_control_type,
      "label='Rot. Control' "
      "group='Node' "
      "help='choose type of rotation controls for nodes' ");
  TwEnumVal RotationAxisTypeEV[NUM_ROTATION_AXIS_TYPES] = 
  {
    {ROTATION_AXIS_TYPE_VIEW,"View"},
    {ROTATION_AXIS_TYPE_UP, "Up"},
    {ROTATION_AXIS_TYPE_RIGHT, "Right"}
  };
  TwType RotationAxisTypeTW = 
    igl::anttweakbar::ReTwDefineEnum("RotationAxisType", RotationAxisTypeEV, NUM_ROTATION_AXIS_TYPES);
  rebar.TwAddVarRW("node_rotation_axis_type",RotationAxisTypeTW,&Node::rotation_axis_type,
    " label='Rot. axis' group='Node' key=V"
    " help='Vector about which to rotate when using axis rot. control type'");
  rebar.TwAddVarRW("node_draw_numbes",TW_TYPE_BOOLCPP,&Node::draw_numbers,
    "group='Node' "
    "label='numbers' "
    "help='Number children outlets' ");
  rebar.TwAddButton("node_reset_all_angles",&reset_rotationsCB,this,
    "group='Node' "
    "label='reset rotations' "
    "key='t' "
    "help='Reset all node angles or bone transformations' "
    );
  rebar.TwAddButton("node_reset_all_offsets",&reset_all_offsetsCB,this,
    "group='Node' "
    "label='reset offsets' "
    "key='r' "
    "help='Reset all node offsets' "
    );
  rebar.TwAddButton("node_reset_drag_arrows",&reset_all_drag_arrowsCB,this,
    "group='Node' "
    "label='reset drag arrows' "
    "key='U' "
    "help='Reset all node drag arrows' "
    );
  rebar.TwAddVarRW("node_LEDs_visible",TW_TYPE_BOOLCPP,&TBT::LEDs_visible,
    "group='Node' "
    "label='LEDs visible' "
    "key='l' "
    "help='Show (blinking) LEDs on TBT nodes' "
    );
  rebar.TwSetParam( "Node", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
}

void PupSkin::add_mesh_group_to_anttweakbar()
{
  using namespace igl;
  using namespace std;
  rebar.TwAddVarCB("num_vertices",
    TW_TYPE_INT32,
    no_op,
    get_mesh_VrowsCB,
    this,
    "group='Mesh' "
    "label='#V' "
    "readonly=true "
    "help='Number of vertices' ");
  rebar.TwAddVarCB("num_facets",
    TW_TYPE_INT32,
    no_op,
    get_mesh_FrowsCB,
    this,
    "group='Mesh' "
    "label='#F' "
    "readonly=true "
    "help='Number of facets (triangles)' ");
  rebar.TwAddVarRW("m_color",TW_TYPE_COLOR4F,m.color,
    "group='Mesh' "
    "label='color' "
    "colormode='hls' "
    "help='Color of mesh including target alpha.' ");
  rebar.TwAddVarRW("m_alpha",TW_TYPE_FLOAT,&m.color[3],
    "group='Mesh' "
    "label='alpha' "
    "keyIncr=']' keyDecr='[' max=1 min=0 step=0.25"
    "help='Alpha channel.' ");
  //rebar.TwAddVarRW("pick_on_mesh",TW_TYPE_BOOLCPP,&pick_on_mesh,
  //  "group='Mesh' "
  //  "label='pickable?' "
  //  "help='Whether mesh is pickable on screen.' ");
  rebar.TwAddVarCB("mesh_invert_orientation",TW_TYPE_BOOLCPP,
    set_mesh_invert_orientationCB,get_mesh_invert_orientationCB,this,
    "group='Mesh' "
    "label='invert orientation' "
    "help='Invert orientation of mesh facets (normals)' ");
  const auto & save_deformedCB = [](void * clientData)
  {
    static int saved_mesh_count = 0;
    const Mesh & m_def = *static_cast<const Mesh *>(clientData);
    string name = 
      STR("puppet-"<<setw(4)<<setfill('0')<<saved_mesh_count<<".off");
    if(writeOFF( name, m_def.getV(), m_def.getF()))
    {
      cout<<GREENGIN("Saved "<<name<<" successfully.")<<endl;
    }else
    {
      cout<<REDRUM("Saving "<<name<<" failed.")<<endl;
    }
    saved_mesh_count++;
  };
  rebar.TwAddButton("mesh_save_deformed",
    save_deformedCB,&this->m_def,"label='save' group='Mesh' key=M");
  rebar.TwAddVarRW("mesh_is_visible",
    TW_TYPE_BOOLCPP,
    &mesh_is_visible,"label='visible?' group='Mesh' key=m");


  rebar.TwSetParam("Mesh", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
}

void PupSkin::add_skinning_group_to_anttweakbar()
{
  using namespace igl;
  rebar.TwAddVarCB("cpu_lbs",TW_TYPE_BOOLCPP,
    set_cpu_lbsCB,get_cpu_lbsCB,this,
    "group='Skinning' "
    "label='cpu lbs' "
    "key='C' "
    "help='Compute LBS deformation on CPU' ");
  rebar.TwAddVarRW("bbw_max_iter",TW_TYPE_INT32,&bbw_max_iter,
    "group='Skinning' "
    "min=1 max=40");
  TwEnumVal SkinningMethodEV[NUM_SKINNING_METHODS] = 
  {
    {SKINNING_METHOD_LBS,"lbs"},
    {SKINNING_METHOD_DQS,"dqs"},
  };
  TwType SkinningMethodTW = 
    igl::anttweakbar::ReTwDefineEnum(
        "SkinningMethodType", 
        SkinningMethodEV, 
        NUM_SKINNING_METHODS);
  rebar.TwAddVarRW(
      "skinning_method",
      SkinningMethodTW,
      &skinning_method,
      "group='Skinning' key='D' "
      "label='method' help='skinning method (linear blend, dual quaternions)' ");
  rebar.TwAddVarRW("rig_skeleton_visible",TW_TYPE_BOOLCPP,&rig_skeleton_visible,
    "group='Skinning' "
    "label='show rig skeleton?' "
    "key='S' "
    "help='Show rig skeleton' ");
  TwType SkeletonStyleTypeTW = igl::anttweakbar::ReTwDefineEnumFromString("SkeletonStyleType",
      "3d,vector-graphics");
  rebar.TwAddVarRW("skel_style",SkeletonStyleTypeTW,&skel_style,
    "group='Skinning' "
    "label='skeleton style' ");
  {
    TwType ControlTypeTW = igl::anttweakbar::ReTwDefineEnumFromString("ControlType",
      "puppet,mouse");
    const auto & get_control_typeCB = [](void * v, void * clientData)
    {
      *(ControlType*)v = static_cast<const PupSkin*>(clientData)->control_type;
    };
    const auto & set_control_typeCB = [](const void * v, void * clientData)
    {
      PupSkin & p = *static_cast<PupSkin*>(clientData);
      const ControlType & ct = *(const ControlType*)v;
      if(p.control_type == ct)
      {
        return;
      }
      p.control_type = ct;
      p.init_rig_animation_is_listening();
    };
    rebar.TwAddVarCB("control_type",ControlTypeTW,
      set_control_typeCB,get_control_typeCB,this,
      "group='Skinning' ");
  }
  rebar.TwAddVarRW("skeleton_visible",TW_TYPE_BOOLCPP,&skeleton_visible,
    "group='Skinning' "
    "label='show skeleton?' "
    "key='s' "
    "help='Show skeleton' ");
  rebar.TwAddVarRW("rig_mapping_visible",TW_TYPE_BOOLCPP,&rig_mapping_visible,
    "group='Skinning' "
    "label='rig mapping' "
    "help='Show mapping of rig skeleton to skeleton' ");
  //rebar.TwAddVarRW("correct_relative_frames",TW_TYPE_BOOLCPP,&correct_relative_frames,
  //  "group='Skinning' ",false);
  rebar.TwSetParam( "Skinning", "opened", TW_PARAM_INT32, 1, &INT_ZERO);
}

bool PupSkin::init_puppet()
{
  using namespace std;
  //pr = new PuppetReader("/dev/tty.mcat-DevB",115200);
#ifdef __APPLE__
  // pr = new PuppetReader("/dev/cu.usbserial-FTF3QCG9",115200);
  PupSkin::pr = new PuppetReader("ignored",115200);
#else
  PupSkin::pr = new PuppetReader("/dev/mcat2.0",115200);
  // pr = new PuppetReader("/dev/ttyUSB0",115200);
#endif
  return PupSkin::pr_is_attached();
}

unsigned int PupSkin::load(const std::string _prefix, const unsigned int types)
{
  using namespace std;
  using namespace igl;

  string prefix = filter_prefix(_prefix);
  if(prefix != _prefix)
  {
    cout<<"Filtered "<<_prefix<<" --> "<<prefix<<endl;
  }

  //cout<<"Loading: "<<pupskin_load_to_str(types)<<endl;
  int loaded = PUPSKIN_LOAD_NONE;
  if(types & PUPSKIN_LOAD_MODEL)
  {
    const char * exts[] = {"obj","off"};
    for(int i = 0;i<2;i++)
    {
      if(load_mesh(prefix+MESH_NAME +"."+ exts[i]) && m.getF().size() > 0)
      {
        loaded |= PUPSKIN_LOAD_MODEL;
        break;
      }
    }
  }

  if(types & PUPSKIN_LOAD_TEXTURE)
  {
    const string name = prefix+TEXTURE_NAME+".png";
    if(igl::png::texture_from_png(name,m.tex_id))
    {
      cout<<GREENGIN("Reloaded "<<name<<" successfully.")<<endl;
      loaded |= PUPSKIN_LOAD_TEXTURE;
    }else
    {
      cout<<REDRUM("Reloading "<<name<<" failed.")<<endl;
    }
  }
  if(m.is_textured())
  {
    m.color[0] = 1; m.color[1] = 1; m.color[2] = 1; m.color[3] = 1;
  }

  if(types & PUPSKIN_LOAD_NODES)
  {
    // Always read in last puppet even though device might replace it by
    // rebuild_topology()
    // Reload last nodes
    string name = prefix+NODES_NAME+".xml";
    Node * root = Node::readXML(name);
    if(root)
    {
      cout<<GREENGIN("Reloaded "<<name<<" successfully.")<<endl;
      loaded |= PUPSKIN_LOAD_NODES;
    }else
    {
      cout<<REDRUM("Reloading "<<name<<" failed.")<<endl;
    }
    //if(PupSkin::pr->get_root() == NULL)
    if(pr)
    {
      Node * old_pr_root = PupSkin::pr->set_root(root);
      delete old_pr_root;
      old_pr_root = NULL;
      root = NULL;
    }
  }

  // Try to load weights
  if(types & PUPSKIN_LOAD_WEIGHTS)
  {
    if(load_weights(prefix+WEIGHTS_NAME+".dmat"))
    {
      loaded |= PUPSKIN_LOAD_WEIGHTS;
    }
  }

  if(types & PUPSKIN_LOAD_REBAR)
  {
    if(rebar.load((prefix+REBAR_NAME + ".rbr").c_str()))
    {
      loaded |= PUPSKIN_LOAD_REBAR;
      cout<<GREENGIN("Reloaded "<<(prefix+REBAR_NAME + ".rbr")<<
          " successfully.")<<endl;
    }else
    {
      cout<<REDRUM("Reloading "<<(prefix+REBAR_NAME + ".rbr")<<
          " failed.")<<endl;
    }
  }

  if(types & PUPSKIN_LOAD_SKELETON)
  {
    if(load_skeleton(prefix+SKELETON_NAME+".mat"))
    {
      loaded |= PUPSKIN_LOAD_SKELETON;
    }
  }

  if(types & PUPSKIN_LOAD_POSES)
  {
    string filename = prefix+POSES_NAME+".dmat";
    if(posing_test.load_poses(filename))
    {
      loaded |= PUPSKIN_LOAD_POSES;
      cout<<GREENGIN("Reloaded "<<filename<<" successfully.")<<endl;
    }else
    {
      cout<<REDRUM("Reloading "<<filename<<" failed.")<<endl;
    }
  }

  return loaded;
}

bool PupSkin::load_mesh(const std::string filename)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  {
    {
      string d,b,e,f;
      pathinfo(filename,d,b,e,f);
      // Convert extension to lower case
      std::transform(e.begin(), e.end(), e.begin(), ::tolower);
      if(e == "obj")
      {
        MatrixXd CN;
        MatrixXi FTC,FN;
        m.dgetTC().resize(0,2);
        if(!readOBJ(filename,m.dgetV(),m.dgetTC(),CN,m.dgetF(),FTC,FN))
        {
          cout<<REDRUM("Reading "<<filename<<" failed.")<<endl;
          return false;
        }
      }else{
        if(!read_triangle_mesh(filename,m.dgetV(),m.dgetF()))
        {
          cout<<REDRUM("Reading "<<filename<<" failed.")<<endl;
          return false;
        }
      }
      if(m.getTC().rows()>0)
      {
        if(m.getTC().rows() != m.getV().rows())
        {
          cout<<REDRUM("#TC should equal #V")<<endl;
          m.dgetTC().resize(0,2);
        }
      }
    }
  }
  cout<<GREENGIN("Read "<<filename<<" successfully.")<<endl;
  per_face_normals(  m.dgetV(),m.dgetF(),m.dgetFN());
  per_vertex_normals(m.dgetV(),m.dgetF(),m.dgetFN(),m.dgetVN());
  per_corner_normals(m.dgetV(),m.dgetF(),m.dgetFN(),30.0,m.dgetCN());
  m.dget_normal_type() = igl::PER_FACE_NORMALS;
  m.compute_scale_and_shift();
  // Also try to initial with existing weights
  if(W.size() != 0)
  {
    should_init_lbs = true;
  }
  ei.init(m.getV().cast<float>(),m.getF().cast<int>());
  return true;
}

bool PupSkin::load_skeleton(const std::string filename)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  using namespace igl::matlab;
  // clear any selected bones
#ifndef WITH_MATLAB
  cerr<<REDRUM("Skeleton loading disabled without WITH_MATLAB defined")<<endl;
  return true;
#else
  MatlabWorkspace mw;
  if(!mw.read(filename))
  {
    cout<<REDRUM("Reading "<<filename<<" failed.")<<endl;
    return false;
  }
  if(
    !mw.find("C",C) ||
    !mw.find_index("BE",BE) ||
    !mw.find_index("RP",RP) ||
    !mw.find_index("P",P))
  {
    assert(BE.maxCoeff()<C.rows());
    cout<<REDRUM("Parsing "<<filename<<" failed.")<<endl;
    return false;
  }
  // scaling nightmare. I regret everything.
  m.unrelative_to_mesh(C);
  cout<<GREENGIN("Read "<<filename<<" successfully.")<<endl;
  mouse.set_size(BE.rows());
  if(cpu_lbs && get_const_root())
  {
    // Match to rig skeleton
    MatrixXd dC;
    MatrixXi dBE;
    VectorXi dP,dRP;
    Node::matlab(get_const_root(),true,dC,dBE,dP,dRP);
    match_skeletons(C,BE,dC,dBE,nodes_to_rig);
    if(BE.rows() != (int)rig_animation.bone_animations().size())
    {
      rig_animation.clear(BE.rows());
    }
    init_rig_animation_is_listening();
    //cout<<matlab_format(nodes_to_rig.transpose().eval(),"nodes_to_rig")<<endl;
  }
  return true;
#endif
}

bool PupSkin::load_weights(const std::string filename)
{
  using namespace std;
  using namespace igl;
  if(!readDMAT(filename,W))
  {
    cout<<REDRUM("Reading "<<filename<<" failed.")<<endl;
    return false;
  }
  cout<<GREENGIN("Read "<<filename<<" successfully.")<<endl;
  if(m.getV().size() != 0)
  {
    should_init_lbs = true;
  }
  return true;
}

bool PupSkin::init_lbs()
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  if(W.rows() < m.getV().rows())
  {
    cerr<<REDGIN("Error: W.rows() ("<<W.rows()<<
      ") < V.rows() ("<<m.getV().rows()<<")")<<endl;
     return false;
  }
  const int num_rps = RP.maxCoeff()+1;
  if(W.cols()!=num_rps)
  {
     cerr<<REDGIN("Error: W.cols() ("<<W.cols()<<
       ") != num_rps ("<<num_rps<<")")<<endl;
     return false;
  }
  lbs_matrix(m.as_drawn(),W,M);
  posing_test.set_rig(C, BE, P, RP, m, skinning_method, W, M);
  return true;
}


Node * PupSkin::get_root()
{
  if(!pr)
  {
    return NULL;
  }
  return PupSkin::pr->get_root();
}

const Node * PupSkin::get_const_root() const
{
  if(!pr)
  {
    return NULL;
  }
  return PupSkin::pr->get_const_root();
}

const Mesh & PupSkin::get_const_mesh() const
{
  return m;
}

Mesh & PupSkin::get_mesh()
{
  return m;
}

bool PupSkin::get_cpu_lbs() const
{
  return cpu_lbs;
}

#define NUM_CPU_LBS_READONLY 1
const char * const CPU_LBS_READONLY[NUM_CPU_LBS_READONLY] = 
{
  "node_reset_all_offsets"
};

void PupSkin::set_cpu_lbs(const bool v)
{
  using namespace std;
  using namespace igl;
  // reset rememberence
  was_cpu_lbs = false;
  for(int s = 0;s<NUM_CPU_LBS_READONLY;s++)
  {
    if(v)
    {
      rebar.TwSetParam(CPU_LBS_READONLY[s],"readonly",TW_PARAM_INT32,1,&igl::INT_ONE);
    }else
    {
      rebar.TwSetParam(CPU_LBS_READONLY[s],"readonly",TW_PARAM_INT32,1,&igl::INT_ZERO);
    }
  }
  cpu_lbs = v;
  if(cpu_lbs)
  {
    auto_fit_activated = false;
  }
}

static GLint old_vp[4];
void PupSkin::push_scene(const AugViewport & vp) const
{
  using namespace igl;
  using namespace std;
  glGetIntegerv(GL_VIEWPORT,old_vp);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();

  if(Pickle::is_picking)
  {
    // change viewport to be small window around mouse
    assert(pick_s > 0);
    int pick_w = pick_s;
    double pick_ratio = double(pick_w)/double(old_vp[2]);
    // ceil, cause might otherwise round down to 0
    int pick_h = ceil(double(old_vp[3])*pick_ratio);
    glViewport(
      hover_x-pick_w,
      four_view.height()-hover_y-pick_h,2*pick_w+1,2*pick_h+1);
    gluPickMatrix(
      hover_x,
      four_view.height()-hover_y, 
      pick_w*2+1,
      pick_h*2+1,
      old_vp);
  }
  auto & camera = vp.camera;
  glMultMatrixd(camera.projection().data());
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  if(mirror_mode)
  {
    glScaled(-1,1,1);
  }
  gluLookAt(
    camera.eye()(0), camera.eye()(1), camera.eye()(2),
    camera.at()(0), camera.at()(1), camera.at()(2),
    camera.up()(0), camera.up()(1), camera.up()(2));
}

double PupSkin::y_boost() const
{
  using namespace std;
  double y = 0;
  if(cpu_lbs)
  {
    y = m_def.y_boost;
    if(posing_test.cur_pose()!=NULL && posing_test.m_pose_is_visible)
    {
      y = max(y,posing_test.pose_mesh().y_boost);
    }
  }
  return y;
}

void PupSkin::pop_scene() const
{
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  if(Pickle::is_picking)
  {
    // reset viewport
    glViewport(old_vp[0],old_vp[1],old_vp[2], old_vp[3]);
  }
}

void PupSkin::draw_floor(const double floor_scale, const double floor_offset)
{
  using namespace Eigen;
  using namespace igl;
  using namespace igl::opengl2;
  glPushMatrix();
  glTranslated(0,floor_offset,0);
  glScaled(floor_scale,floor_scale,floor_scale);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(10.0,1);
  if(project(Vector3d(0,0,0))(2) -igl::opengl2::project(Vector3d(0,1,0))(2) > -FLOAT_EPS)
  {
    draw_floor_outline();
  }
  igl::opengl2::draw_floor();
  glDisable(GL_POLYGON_OFFSET_FILL);
  glPopMatrix();
}

bool PupSkin::deform_mesh()
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  // double check that skinning with these weights is possible
  if(W.rows()<m.getV().rows())
  {
    cerr<<REDRUM("Error: W.rows() ("<<W.rows()<<
        ") < V.rows() ("<<m.getV().rows()<<")")<<endl;
    set_cpu_lbs(false);
    was_cpu_lbs = false;
    return false;
  }
  RotationList vQ;
  vector<Vector3d> vT;
  if(!rig_transformations(vQ,vT))
  {
    set_cpu_lbs(false);
    was_cpu_lbs = false;
    return false;
  }
  vQ = throw_at(vQ,RP);
  vT = throw_at(vT,RP);
  if(should_init_lbs)
  {
    init_lbs();
    should_init_lbs = false;
  }
  if(M.rows() != m.getV().rows())
  {
    if(cpu_lbs)
    {
      cerr<<REDRUM("Error: M.rows() ("<<M.rows()<<
          ") != V.rows() ("<<m.getV().rows()<<")")<<endl;
      set_cpu_lbs(false);
      was_cpu_lbs = true;
    }
    // No matter what, we cannot deform
    return false;
  }
  if(M.cols() != (int)vT.size()*(3+1))
  {
    if(cpu_lbs)
    {
      cerr<<REDRUM("Error: M.cols() ("<<M.cols()<<
          ") != vT.size()*(3+1) ("<<vT.size()*(3+1)<<")")<<endl;
      set_cpu_lbs(false);
      was_cpu_lbs = true;
    }
    // No matter what, we cannot deform
    return false;
  }
  if(was_cpu_lbs)
  {
    set_cpu_lbs(true);
    was_cpu_lbs = false;
  }
  return m.deformed_copy(skinning_method,W,vQ,vT,M,m_def);
}

void PupSkin::deform_rig_skeleton(
  Eigen::MatrixXd & CT,
  Eigen::MatrixXi & BET)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  vector<Affine3d,aligned_allocator<Affine3d> > vA;
  if(!cpu_lbs || Node::show_bind)
  {
    vA.resize(BE.rows(),Affine3d::Identity());
  }else
  {
    RotationList vQ;
    vector<Vector3d> vT;
#ifndef NDEBUG
    bool ret = 
#endif
      rig_transformations(vQ,vT);
    assert(ret);
    quattrans2affine(vQ,vT,vA);
  }
  deform_skeleton(C,BE,vA,CT,BET);
}

bool PupSkin::rig_transformations(
  std::vector<
    Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > & vQ,
  std::vector<Eigen::Vector3d> & vT)
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;
  RotationList rdQ;
  if(!rig_pose(rdQ))
  {
    return false;
  }
  forward_kinematics(C,BE,P,rdQ,vQ,vT);
  return true;
}

bool PupSkin::rig_pose_from_control(RotationList & rdQ)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  // Relative rotations
  rdQ = RotationList(BE.rows(),Quaterniond::Identity());
  switch(control_type)
  {
    case CONTROL_TYPE_PUPPET:
    {
      RotationList dQ;
      Node::relative_rotations(get_const_root(),dQ);
      // Device skeleton
      MatrixXd nC;
      MatrixXi nBE;
      {
        VectorXi nP,nRP;
        Node::matlab(get_const_root(),true,nC,nBE,nP,nRP);
      }
      if(nodes_to_rig.rows() != nBE.rows())
      {
        cerr<<REDRUM("Error: nodes_to_rig.rows() ("<<nodes_to_rig.rows()<<
            ") != nBE.rows() ("<<nBE.rows()<<")")<<endl;
        return false;
      }
      if(nodes_to_rig.maxCoeff() >= BE.rows())
      {
        cerr<<REDRUM("Error: max(nodes_to_rig) ("<<
          nodes_to_rig.maxCoeff()<<" >= "<<BE.rows());
        return false;
      }
      // Loop over device bones
      for(int e = 0;e<nodes_to_rig.rows();e++)
      {
        const int re = nodes_to_rig(e);
        if(re >= 0)
        {
          if(correct_relative_frames)
          {
            // rig bone vector
            RowVector3d v = C.row(BE(re,1)) - C.row(BE(re,0));
            // Device bone vector
            RowVector3d nv = nC.row(nBE(e,1)) - nC.row(nBE(e,0));
            Quaterniond q;
            q.setFromTwoVectors(v,nv);
            // Throw at rig bone relative rotation
            rdQ[re] = q.conjugate() * dQ[e] * q;
          }else
          {
            rdQ[re] = dQ[e];
          }
        }
      }
      break;
    }
    case CONTROL_TYPE_MOUSE:
    {
      assert((int)mouse.rotations().size() == BE.rows());
      rdQ = mouse.rotations();
      break;
    }
    default:
      assert(false);
      break;
  }
  return true;
}

bool PupSkin::rig_pose(RotationList & rdQ)
{
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  if(!rig_pose_from_control(rdQ))
  {
    return false;
  }
  if(rig_animation.is_playing())
  {
    const double export_framerate = 30.;
    rig_animation.play(
      exporting_to_png?
      ((double)export_count)/export_framerate+rig_animation.t_start():
        get_seconds(),
      rdQ);
  }
  return true;
}

bool PupSkin::rig_transformations(
  Eigen::MatrixXd & T)
{
  using namespace Eigen;
  using namespace std;
  RotationList vQ;
  vector<Vector3d> vT;
  if(!rig_transformations(vQ,vT))
  {
    return false;
  }
  vector<Affine3d,aligned_allocator<Affine3d> > vA;
  quattrans2affine(vQ,vT,vA);
  list_affine_to_stack(vA,T);
  return true;
}


void PupSkin::draw_objects(const int vp)
{
  using namespace igl;
  using namespace igl::opengl2;
  using namespace Eigen;
  using namespace std;

  if(cpu_lbs || was_cpu_lbs)
  {
    deform_mesh();
  }

  // y-boost fucks up mouse controls
  if(control_type != CONTROL_TYPE_MOUSE)
  {
    glPushMatrix();
    glTranslated(0,y_boost(),0);
  }

  if(this->floor_visible && !Pickle::is_picking && m.getV().size() > 0)
  {
    const double floor_scale = 1;//m.camera.zoom;
    const double floor_offset = 
      m.scale*(m.getV().col(1).minCoeff()+m.shift(1))-y_boost();
    this->draw_floor(floor_scale,floor_offset);

    //Shadow
    //// Non-z-fighting, but layered
    //glDepthMask(GL_FALSE);
    // Non-z-fighting and not layered
    glEnable(GL_STENCIL_TEST);
    //glDisable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    for(int l = 0;l<scene_lights.cols();l++)
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
      if(mirror_mode)
      {
        glFrontFace(GL_CW);
      }else
      {
        glFrontFace(GL_CCW);
      }
      this->draw_floor(floor_scale,floor_offset);
      glEnable(GL_CULL_FACE);
      glCullFace(GL_BACK);
      glStencilFunc(GL_EQUAL, 1, 0xff);
      //glEnable(GL_POLYGON_OFFSET_FILL);
      //glPolygonOffset(-l,10);
      const Vector4d pos = -1.*scene_lights.col(l).cast<double>();
      const Vector4d plane(0,-1,0,floor_offset*floor_scale);
      Matrix4d shadow_floor;
      shadow_matrix(plane,pos,shadow_floor);
      glPushMatrix();
      glMultMatrixd(shadow_floor.data());

      glDisable(GL_LIGHTING);
      glEnable(GL_COLOR_MATERIAL);
      glColor4f(0,0,0,0.5*m.color[3]);
      posing_test.draw_raw_pose_mesh();
      draw_mesh();
      glEnable(GL_LIGHTING);
      glPopMatrix();
    }
    //glDisable(GL_POLYGON_OFFSET_FILL);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
  }



  // if not on top then draw before so behind transparent mesh
  if(!ui_on_top)
  {
    // Draw puppet
    draw_puppet(vp == four_view.down_vp());
    draw_ui();
  }

  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(10.0,1);
  posing_test.draw();
  glDisable(GL_POLYGON_OFFSET_FILL);
  // Turn off specular lighting
  if(m.is_hover)
  {
    dim_lights(0.5);
  }else if(m.is_selected)
  {
    dim_lights(0.65);
  }
  m.activate_pickle_or_diffuse_material(m.color,m.color[3]);
  draw_mesh();
  if(m.is_hover)
  {
    dim_lights(1./0.5);
  }else if(m.is_selected)
  {
    dim_lights(1./0.65);
  }

  if(ui_on_top)
  {
    glClear(GL_DEPTH_BUFFER_BIT);
    // Draw puppet
    draw_puppet(vp == four_view.down_vp());
    draw_ui();
  }


  // y-boost fucks up mouse controls
  if(control_type != CONTROL_TYPE_MOUSE)
  {
    glPopMatrix();
  }
}

void PupSkin::draw_mesh()
{
  if(!mesh_is_visible)
  {
    return;
  }
  glEnable(GL_NORMALIZE);
  if(!Pickle::is_picking || pick_on_mesh)
  {
    if(cpu_lbs && !Node::show_bind)
    {
      m_def.draw_and_cache();
    }else
    {
      m.draw_and_cache();
    }
  }
}

void PupSkin::draw_puppet(const bool show_controls)
{
  if(get_root() && Node::visible && control_type == CONTROL_TYPE_PUPPET)
  {
    // Only show "controls" (guides etc.) for down viewport
    const bool old_show_controls = Node::show_controls;
    Node::show_controls = show_controls;
    get_root()->draw();
    Node::show_controls = old_show_controls;
  }
}

void PupSkin::draw_ui()
{  
  using namespace igl;
  using namespace igl::opengl2;
  using namespace std;
  using namespace Eigen;
  // Draw skeleton beneath puppet
  {
    MatrixXd dC,dC_bind;
    MatrixXi dBE;
    VectorXi dP,dRP;
    Node::matlab(get_const_root(),true,dC_bind,dBE,dP,dRP);
    Node::matlab(get_const_root(),Node::show_bind,dC,dBE,dP,dRP);
    const float * point_color = BBW_POINT_COLOR;
    const float * line_color =
      (Node::show_bind?
        FAST_GREEN_DIFFUSE:
        BBW_LINE_COLOR);
    if(skeleton_visible && control_type == CONTROL_TYPE_PUPPET)
    {
      if(!(rig_skeleton_visible && 
        (dC_bind.size() > 0 &&
         dC_bind.rows() == this->C.rows() &&
         dC_bind.cols() == this->C.cols() &&
        dC_bind.isApprox(this->C))))
      {
        switch(skel_style)
        {
          case SKELETON_STYLE_TYPE_3D:
            if(!cpu_lbs || Node::show_bind)
            {
              draw_skeleton_3d(dC,dBE,MatrixXd(),MAYA_VIOLET);
            }else
            {
              MatrixXd T;
              Node::transformations(get_const_root(),false,T);
              draw_skeleton_3d(dC_bind,dBE,T,MAYA_SEA_GREEN);
            }
            break;
          case SKELETON_STYLE_TYPE_VECTOR_GRAPHICS:
            draw_skeleton_vector_graphics(dC,dBE,point_color,line_color);
            break;
          default:
            break;
        }
      }
    }

    MatrixXd * rC = &C;
    MatrixXi * rBE = &BE;
    MatrixXd CT;
    MatrixXi BET;
    if(cpu_lbs && !Node::show_bind)
    {
      deform_rig_skeleton(CT,BET);
      rC = &CT;
      rBE = &BET;
    }

    if(rig_skeleton_visible)
    {
      float eff_pcolor[4], eff_lcolor[4];
      copy(point_color,point_color+4,eff_pcolor);
      //copy(line_color,line_color+4,eff_lcolor);
      copy(FAST_RED_DIFFUSE,FAST_RED_DIFFUSE+4,eff_lcolor);
      // reduce saturation by 50%
      auto desaturate = [](float * color)
      {
        const float f = 0.5;
        const float L = 0.3*color[0] + 0.6*color[1] + 0.1*color[2];
        color[0] += f*(L-color[0]);
        color[1] += f*(L-color[1]);
        color[2] += f*(L-color[2]);
      };
      desaturate(eff_lcolor);
      desaturate(eff_pcolor);
      MatrixXd T = MatrixXd::Identity(3+1,3).replicate(BE.rows(),1);
      switch(skel_style)
      {
        case SKELETON_STYLE_TYPE_3D:
        {
          if(cpu_lbs && !Node::show_bind)
          {
#ifndef NDEBUG
            bool ret =
#endif
              rig_transformations(T);
            assert(ret);
          }
          MatrixXf color;
          switch(control_type)
          {
            case CONTROL_TYPE_MOUSE:
            {
              MouseController::VectorXb S;
              MouseController::propogate_to_descendants_if(mouse.selection(),P,S);
              MouseController::color_if(S,MAYA_SEA_GREEN,MAYA_VIOLET,color);
              break;
            }
            case CONTROL_TYPE_PUPPET:
              color = MAYA_VIOLET;
              break;
            default:
              assert(false);
              break;
          }
          draw_skeleton_3d(C,BE,T,color);
          break;
        }
        case SKELETON_STYLE_TYPE_VECTOR_GRAPHICS:
        {
          draw_skeleton_vector_graphics(*rC,*rBE,eff_pcolor,eff_lcolor);
          break;
        }
        default:
          assert(false);
      }
    }

    if(rig_skeleton_visible && 
      // skeleton_visible &&
      rig_mapping_visible && control_type == CONTROL_TYPE_PUPPET)
    {
      if(cpu_lbs)
      {
        draw_skeleton_matching(*rC,*rBE,dC,dBE,nodes_to_rig);
      }else
      {
        VectorXi I;
        match_skeletons(C,BE,dC_bind,dBE,I);
        draw_skeleton_matching(*rC,*rBE,dC,dBE,I);
      }
    }
  }

  if(control_type == CONTROL_TYPE_MOUSE)
  {
    // either -1 or valid
    assert(mouse.selection().size()==BE.rows());
    mouse.draw();
  }

  if(drag_arrows_visible && !cpu_lbs && control_type == CONTROL_TYPE_PUPPET)
  {
    if(get_root())
    {
      // Should be const
      deque<Node*> Q = deque<Node*>(1,get_root());
      while(Q.size()>0)
      {
        Node * n = Q.front(); 
        Q.pop_front();
        // Reset "from" to origin
        n->drag_arrow.from = n->origin();
        n->drag_arrow.draw();
        // add children
        Node::add_children(n,Q,Q.end());
      }
    }
  }
  
}


bool PupSkin::hover(const int hover_x, const int hover_y)
{
  using namespace std;
  bool any = false;
  float pixel[4] = {0,0,0,0};
  pick(hover_x,hover_y,pixel);
  m.is_hover = false;
  if(pick_on_mesh)
  {
    m.is_hover = m.diff(pixel) < 0.005;
  }
  any |= m.is_hover;
  if(get_root())
  {
    deque<Node*> Q = deque<Node*>(1,get_root());
    while(Q.size()>0)
    {
      Node * n = Q.front(); 
      Q.pop_front();
      bool any = false;
      for(int i = 0;i<(int)n->pickles.size();i++)
      {
        any |= 
          n->pickle_hit[i] = 
            n->pickles[i].diff(pixel) < 0.005;
      }
      n->set_is_hover(any);
      any |= n->get_is_hover();
      // add children
      Node::add_children(n,Q,Q.end());
    }
  }
  return any;
}

void PupSkin::display()
{
  using namespace igl;
  using namespace igl::opengl2;
  using namespace igl::opengl;
  using namespace igl::png;
  using namespace std;
  using namespace Eigen;

  // Handle hovering for picking on mouse passive move
  if(hover_next)
  {
    hover(hover_x,hover_y);
    hover_next = false;
  }

  // Sync with puppet
  if(PupSkin::pr_is_attached())
  {
    PupSkin::pr->sync();
  }

  const bool rig_animation_was_playing = rig_animation.is_playing();
  {
    RotationList rdQ;
    if(cpu_lbs && !should_init_lbs && rig_pose_from_control(rdQ))
    {
      posing_test.update(rdQ);
      rig_animation.update(rdQ);
    }
  }

  // Set scale automatically
  if(get_root() && auto_scale)
  {
    set_node_scale(get_root());
  }

  if(!cpu_lbs && auto_fit_activated)
  {
    // Set fit automatically
    if(get_root())
    {
      auto_fit();
    }
  }
  
  {
    // Grey that matches apple ambient space
    //const float GREY[3] = {190.0/255.0,190.0/255.0,190.0/255.0};
    const float WHITE[3] = {1.,1.,1.};
    glClearColor(WHITE[0],WHITE[1],WHITE[2],0);
  }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Draw scene elements
  draw_scene();

  four_view.draw();

  if(render_to_png_on_next)
  {
    stringstream padnum; 
    padnum << getenv("HOME")<<"/Desktop/"<< "puppet-" << setw(4) << setfill('0') << render_count++ << ".png";
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    render_to_png_async(padnum.str(),viewport[2],viewport[3],true,false);
    glClearColor(1,1,1,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glFlush();
    usleep(1000*1);
    render_to_png_on_next = false;
  }
  if(exporting_to_png && rig_animation_was_playing)
  {
    stringstream padnum; 
    padnum << getenv("HOME")<<"/Desktop/"<< "puppet-export-" << setw(5) << setfill('0') << export_count++ << ".png";
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    render_to_png_async(padnum.str(),viewport[2],viewport[3],true,false);
    exporting_to_png = rig_animation.is_playing();
    if(!exporting_to_png)
    {
      cout<<R"(Convert to video with:

/usr/local/bin/ffmpeg -f image2 -r 30 -i ~/Desktop/puppet-export-%05d.png -r 30 -vcodec libx264 -pix_fmt yuv420p -q:vscale 0 ~/Desktop/puppet-animation.mp4

)";
    }
  }

  // Draw anttweakbar
  TwDraw();

  {
    static double last_display_t = get_seconds_hires();
    static int display_count = 0;
    const int FPS_REFRESH_RATE = 100;
    if(!(display_count%FPS_REFRESH_RATE))
    {
      fps = double(FPS_REFRESH_RATE)/(get_seconds_hires()-last_display_t);
      last_display_t = get_seconds_hires();
    }
    display_count++;
  }
  glutSwapBuffers();
  // Only refresh if using device or animating
  {
    if(pr_is_attached() || 
        four_view.any_rotation_animating() || 
        rig_animation_was_playing)
    {
      glutPostRedisplay();
    }
  }
}

bool PupSkin::auto_fit()
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;
  bool skip_tree_fit = false;
  // Initial positions
  vector<Vec3> vO;
  // Indices of fixed
  vector<int> vb;
  // Positions of fixed
  vector<Vec3> vbc;
  vector<Node*> i2n;
  map<const Node*,int> n2i;
  {
    deque<Node*> Q = deque<Node*>(1,get_root());
    int k = 0;
    while(Q.size()>0)
    {
      Node * n = Q.front(); 
      Q.pop_front();
      vO.push_back(n->origin());
      i2n.push_back(n);
      n2i[n] = k;
      if(n->drag_arrow.activated)
      {
        vb.push_back(k);
        vbc.push_back(n->drag_arrow.to);
      }
      k++;
      // add children
      Node::add_children(n,Q,Q.end());
    }
    assert(k == (int)vO.size());
  }
  // Number of nodes
  const int n = vO.size();
  MatrixXd O(n,3);
  VectorXi P(n);
  for(int v = 0;v<n;v++)
  {
    O.row(v) = vO[v];
    if(i2n[v]->get_parent())
    {
      // look up parent's index
      P(v) = n2i[i2n[v]->get_parent()];
    }else
    {
      // root
      P(v) = -1;
    }
  }
  MatrixXd bc(vb.size(),3);
  VectorXi b(vb.size());
  for(int v = 0;v<(int)vb.size();v++)
  {
    b(v) = vb[v];
    bc.row(v) = vbc[v];
  }
  // Check if we should just skip it
  const double SKIP_EPS = 5e-3;
  // skip if no draw arrows activated or if parameters haven't really changed.
  skip_tree_fit = b.size() == 0 ||
    ((O.rows() == tf_last_O.rows() && 
     (O-tf_last_O).array().abs().maxCoeff()<SKIP_EPS) &&
    (bc.rows() == tf_last_bc.rows() && 
     (bc-tf_last_bc).array().abs().maxCoeff()<SKIP_EPS) &&
    tf_params == tf_last_tf_params);
  if(skip_tree_fit)
  {
    return false;
  }
  MatrixXd N = O;
  VectorXd S;
  MatrixXd R;
  // Cache input
  tf_last_O = O;
  tf_last_bc = bc;
  tf_last_tf_params = tf_params;
  tree_fit(O,P,b,bc,tf_params,N,S,R);
  // Set all root positions according to N...
  {
    int s = 0;
    for(int v = 0;v<n;v++)
    {
      if(P(v) == -1)
      {
        // Set root directly
        i2n[v]->t_off = N.row(v);
        // There're probably too many tranposes around
        Quat q;
        mat3_to_quat<double>(R.data(),q.coeffs().data());
        q.normalize();
        i2n[v]->r_off = q.conjugate() * i2n[v]->r_off;
      }else
      {
        Vec3 prtc,vt,pct;
        vt = i2n[v]->t_off;
        pct = i2n[P(v)]->ct_off[i2n[v]->get_parent_id()]; 
        Quat prqc;
        i2n[P(v)]->rigid_to_child(i2n[v]->get_parent_id(),prqc,prtc); 
        // ASSUMES THAT prtc is of the form (0,0,z) (see also
        // set_node_scale)
        prtc = (prqc.conjugate() * prtc);
        // v->origin() - p->origin() ~ prtc + vt + pct
        // Want that:
        // ‖prtc + vt + pct‖ = s
        // Let:
        //   len := ‖prtc + vt + pct‖
        // s/len*prtc + s/len*vt + s/len*pct = prtc + vt + pct
        // s/len*prtc + s/len*vt + s/len*pct - prtc - vt = pct
        const double len = (prtc + vt + pct).norm();
        const double scale = S(s)/len;
        Vec3 new_ct_off = scale*prtc + scale*vt + scale*pct - prtc - vt;
        // Only change in z-axis
        i2n[P(v)]->ct_off[i2n[v]->get_parent_id()](2) = new_ct_off(2);
        s++;
      }
    }
  }
  return true;
}

void PupSkin::draw_genie(const AugViewport & vp)
{
  using namespace Eigen;
  using namespace std;
  glClear(GL_DEPTH_BUFFER_BIT);
  // Don't draw if viewport is small.
  if( genie_height>vp.height || genie_width>vp.width)
  {
    return;
  }
  // Draw puppet
  if(get_root() && control_type == CONTROL_TYPE_PUPPET)
  {
    auto cvp = vp;
    cvp.x = vp.x+vp.width-genie_width;
    cvp.y = vp.y+vp.height-genie_height;
    cvp.width = genie_width;
    cvp.height = genie_height;
    cvp.camera.m_aspect = (double)genie_width/(double)genie_height;
    cvp.camera.m_angle = 15;
    int num_nodes = 0;
    {
      // Should be const
      deque<const Node*> Q = deque<const Node*>(1,get_const_root());
      while(Q.size()>0)
      {
        const Node * n = Q.front(); 
        num_nodes++;
        Q.pop_front();
        // add children
        vector<const Node*> c(n->get_const_children()); Q.insert(Q.end(), c.begin(), c.end());
      }
    }
    cvp.camera.look_at(
      Vector3d(0,0,1. + 0.5*num_nodes),
      // Should be "centroid" of skeleton to center genie in box
      get_const_root()->t_off,
      Vector3d(0,1,0));
    // rotate to match this viewport
    cvp.camera.orbit(vp.camera.m_rotation_conj.conjugate());
    const bool old_show_controls = Node::show_controls;
    const bool old_visible = Node::visible;
    const double old_scale = Node::scale;
    Node::show_controls = false;
    Node::visible = true;
    Node::scale = 1.0;
    //Node::scale *= 1.5;
    //Node::scale = std::min(Node::scale,1.0);
    Vector4f bg_color(1.0,0.6,1.0,1);
    if(pr_is_attached())
    {
      bg_color = Vector4f(0.6,0.6,0.6,1);
    }
    if(pr)
    {
      ::draw_genie(cvp,pr->get_const_root(),bg_color);
    }
    Node::scale = old_scale;
    Node::visible = old_visible;
    Node::show_controls = old_show_controls;
  }
}

void PupSkin::draw_scene()
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;

  // Draw background
  glPushMatrix();
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glLoadIdentity();
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  glStencilMask(0x00);
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, background_tex_id);
  glColor3f(1,1,1);
  glBegin(GL_QUADS);
  glTexCoord2d( 0, 1);glVertex2d(-1,-1);
  glTexCoord2d( 1, 1);glVertex2d( 1,-1);
  glTexCoord2d( 1, 0);glVertex2d( 1, 1);
  glTexCoord2d( 0, 0);glVertex2d(-1, 1);
  glEnd();
  glPopAttrib();
  glPopMatrix();

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);



  // All lights share white
  Vector4f l_color(0,0,0,1);
  l_color(0) = l_color(1) = l_color(2) = 
    1./(double)(flash_lights.cols() + scene_lights.cols());

  place_lights(flash_lights,0,l_color,true);
  if(!floor_visible)
  {
    place_lights(scene_lights,flash_lights.cols(),l_color,true);
  }

  // Draw in each viewport
  for(int v = 0;v<four_view.NUM_VIEWPORTS;v++)
  {
    auto & vp = four_view.m_viewports[v];
    // Continue rotating animation
    if(vp.rotation_is_animating)
    {
      const double ANIMATION_DURATION = 0.5;
      double t = (get_seconds() - vp.animation_start_time)/ANIMATION_DURATION;
      if(t > 1)
      {
        t = 1;
        vp.rotation_is_animating = false;
      }
      Quaterniond q = vp.animation_from_quat.slerp(
        t,vp.animation_to_quat).normalized();
      auto & camera = vp.camera;
      camera.orbit(q.conjugate());
    }
    // Minimal size for drawing
    const int MIN_S = 4;
    if(
      vp.width <= MIN_S ||
      vp.height <= MIN_S)
    {
      continue;
    }
    if(Pickle::is_picking && v != four_view.hover_vp())
    {
      continue;
    }
    glViewport(vp.x, vp.y, vp.width, vp.height);
    push_scene(vp);
    if(floor_visible)
    {
      place_lights(scene_lights,flash_lights.cols(),l_color,true);
    }
    draw_objects(v);
    pop_scene();

    // Genie view
    if(genie_is_visible && !Pickle::is_picking)
    {
      draw_genie(vp);
    }

  }

}

void PupSkin::reshape(int width, int height)
{
  using namespace std;
  four_view.reshape(width,height);
  mouse.reshape(width,height);

  // Tell AntTweakBar about new size
  TwWindowSize(width, height);
  // Keep AntTweakBar on right side of screen and height == opengl height
  // get the current position of a bar
  int size[2];
  rebar.TwGetParam(NULL, "size", TW_PARAM_INT32, 2, size);
  int pos[2];
  // Place bar on left side of opengl rect (padded by 10 pixels)
  pos[0] = 10;//max(10,(int)width - size[0] - 10);
  // place bar at top (padded by 10 pixels)
  pos[1] = 10;
  // Set height to new height of window (padded by 10 pixels on bottom)
  size[1] = height-pos[1]-10;
  rebar.TwSetParam( NULL, "position", TW_PARAM_INT32, 2, pos);
  rebar.TwSetParam( NULL, "size", TW_PARAM_INT32, 2,size);
}

static double last_space = 0;
bool PupSkin::key(unsigned char key, int mouse_x, int mouse_y)
{
  using namespace igl;
  using namespace std;

  switch(key)
  {
    //// Uncomment this for testing
    //case ' ':
    //  if(posing_test.finish_pose())
    //  {
    //    if((posing_test.cur_pose_id()+1)==(int)posing_test.poses().size())
    //    {
    //      // Suicide!
    //      // http://www.parashift.com/c++-faq-lite/delete-this.html
    //      delete this;
    //      exit(0);
    //    }
    //    if((get_seconds() - last_space)>2)
    //    {
    //      posing_test.next_pose();
    //      last_space = get_seconds();
    //    }
    //  }else
    //  {
    //    render_to_png_on_next = true;
    //  }
    //  break;
    default:
      if(!TwEventKeyboardGLUT(key,mouse_x,mouse_y))
      {
        return false;
      }
  }
  return true;

}

void PupSkin::mouse_glut(int glutButton, int glutState, int mouse_x, int mouse_y)
{
  using namespace std;
  // Be sure this is called on up and down
  mouse_move(mouse_x,mouse_y);
  bool tw_using = TwEventMouseButtonGLUT(glutButton,glutState,mouse_x,mouse_y);
  switch(glutButton)
  {
    case GLUT_RIGHT_BUTTON:
    case GLUT_LEFT_BUTTON:
    {
      if(glutState==1)
      {
        mouse_up(glutButton,mouse_x,mouse_y);
      }
      else if(glutState==0 && !tw_using)
      {
        mouse_down(glutButton,mouse_x,mouse_y);
      }
      break;
    }
    // Scroll down
    case 3:
    {
      mouse_wheel(0,-1,mouse_x,mouse_y);
      break;
    }
    // Scroll up
    case 4:
    {
      mouse_wheel(0,1,mouse_x,mouse_y);
      break;
    }
    // Scroll left
    case 5:
    {
      mouse_wheel(1,-1,mouse_x,mouse_y);
      break;
    }
    // Scroll right
    case 6:
    {
      mouse_wheel(1,1,mouse_x,mouse_y);
      break;
    }

    default:
    {
      cout<<"Unknown mouse button: "<<glutButton<<endl;
    }
  }
}

void PupSkin::mouse_down(int glutButton, int mouse_x, int mouse_y)
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;
  // modifiers
  const int mod = glutGetModifiers();
  const int height = four_view.height();
  if(!four_view.down(mouse_x,mouse_y))
  {
    four_view.update_anttweakbar_visibility(rebar);
    // down
    if(const AugViewport * down_viewport = four_view.down_viewport())
    {
      glViewport(
          down_viewport->x,
          down_viewport->y,
          down_viewport->width,
          down_viewport->height);
      push_scene(*down_viewport);

      switch(control_type)
      {
        case CONTROL_TYPE_PUPPET:
        {
          // clear mouse input 
          mouse.clear_selection();
          mouse.set_size(BE.rows());
          float pixel[4] = {0,0,0,0};
          bool down_used = false;

          // Down on drag arrows
          if(get_root() && !cpu_lbs && drag_arrows_visible)
          {
            deque<Node*> Q = deque<Node*>(1,get_root());
            while(!down_used && Q.size()>0)
            {
              Node * n = Q.front(); 
              Q.pop_front();
              down_used |= n->drag_arrow.down(mouse_x,height-mouse_y,mod);
              // add children
              Node::add_children(n,Q,Q.end());
            }
          }

          // Down on pickable objects (nodes and mesh)
          if(!down_used && pick(hover_x,hover_y,pixel))
          {
            if(pick_on_mesh)
            {
              m.is_down = m.diff(pixel) < 0.005;
              down_used = true;
            }
            m.is_selected = m.is_down;

            if(get_root())
            {
              deque<Node*> Q = deque<Node*>(1,get_root());
              while(Q.size()>0)
              {
                Node * n = Q.front(); 
                Q.pop_front();
                bool any = false;
                for(int i = 0;i<(int)n->pickles.size();i++)
                {
                  any |= 
                    n->pickle_hit[i] = 
                    n->pickles[i].diff(pixel) < 0.005;
                }
                down_used |= any;
                n->set_is_down(any);
                n->set_is_selected(n->get_is_down());
                // add children
                Node::add_children(n,Q,Q.end());
              }
            }
          }

          if(!down_used)
          {
            // Reset selected
            m.is_selected = m.is_down;
            // Down again on nodes (based on proximity of click to joint)
            if(get_root())
            {
              deque<Node*> Q = deque<Node*>(1,get_root());
              while(Q.size()>0)
              {
                Node * n = Q.front(); 
                Q.pop_front();
                n->set_is_down(false);
                if(!down_used)
                {
                  Vec3 po(0,0,0);
                  Vec3 o = n->origin();
                  igl::opengl2::project(o,po);
                  bool hit = 
                    (po(0)-mouse_x)*
                    (po(0)-mouse_x)+
                    (po(1)-(height-mouse_y))*
                    (po(1)-(height-mouse_y))<2.0*7.0*7.0;
                  down_used |= hit;
                  n->set_is_down(hit);
                }
                n->set_is_selected(n->get_is_down());
                // add children
                Node::add_children(n,Q,Q.end());
              }
            }

            // Otherwise rotating
            if(!down_used)
            {
              glutSetCursor(GLUT_CURSOR_CYCLE);
              is_rotating = true;
              down_used = true;
            }
          }

          // Finally, for pickables, down() functions must be called
          m.down(mouse_x,height-mouse_y);
          if(get_root())
          {
            deque<Node*> Q = deque<Node*>(1,get_root());
            while(Q.size()>0)
            {
              Node * n = Q.front(); 
              Q.pop_front();
              if(glutButton==GLUT_LEFT_BUTTON)
              {
                n->down(mouse_x,height-mouse_y);
              }else if(glutButton==GLUT_RIGHT_BUTTON)
              {
                n->right_down(mouse_x,height-mouse_y);
              }
              // add children
              Node::add_children(n,Q,Q.end());
            }
          }
          break;
        }
        case CONTROL_TYPE_MOUSE:
        {
          if(mod & GLUT_ACTIVE_ALT || glutButton==GLUT_RIGHT_BUTTON)
          {
            glutSetCursor(GLUT_CURSOR_CYCLE);
            is_rotating = true;
          }else
          {
            mouse.down(mouse_x,mouse_y);
          }
          break;
        }
        default:
          assert(false);
          break;
      }
      down_x = mouse_x;
      down_y = mouse_y;
      pop_scene();
    }
  }
}

void PupSkin::mouse_up(int /*glutButton*/, int mouse_x, int mouse_y)
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;
  // modifiers
  const int mod = glutGetModifiers();
  // up
  four_view.up(mouse_x,mouse_y);
  const int height = four_view.height();
  const bool mouse_was_selecting = mouse.is_selecting();
  mouse.up(mouse_x,mouse_y);
  if(const AugViewport * down_viewport = four_view.down_viewport())
  {
    glViewport(
      down_viewport->x,
      down_viewport->y,
      down_viewport->width,
      down_viewport->height);
    push_scene(*down_viewport);

    if(mouse_was_selecting)
    {
      mouse.set_selection_from_last_drag(C,BE,P,RP);
      if(mouse.selection().rows() == (int)rig_animation.bone_animations().size())
      {
        rig_animation.set_is_listening(mouse.selection());
      }
    }

    // Always call up
    if(get_root())
    {
      deque<Node*> Q = deque<Node*>(1,get_root());
      while(Q.size()>0)
      {
        Node * n = Q.front(); 
        Q.pop_front();
        n->drag_arrow.up(mouse_x,height-mouse_y,mod);
        // add children
        Node::add_children(n,Q,Q.end());
      }
    }
    if(is_rotating)
    {
      // Sort for new depth ordering
      m.sort();
      // Log that view change took place
      posing_test.log_view_change(
          four_view.down_camera().m_rotation_conj,
          down_viewport->camera.m_rotation_conj);
    }
    //glutSetCursor(GLUT_CURSOR_LEFT_ARROW);
    glutSetCursor(GLUT_CURSOR_INHERIT);
    m.is_down = false;
    if(get_root())
    {
      deque<Node*> Q = deque<Node*>(1,get_root());
      while(Q.size()>0)
      {
        Node * n = Q.front(); 
        Q.pop_front();
        n->set_is_down(false);
        // add children
        Node::add_children(n,Q,Q.end());
      }
    }
    m.up(mouse_x,height-mouse_y);
    if(get_root())
    {
      deque<Node*> Q = deque<Node*>(1,get_root());
      while(Q.size()>0)
      {
        Node * n = Q.front(); 
        Q.pop_front();
        n->up(mouse_x,height-mouse_y);
        // add children
        Node::add_children(n,Q,Q.end());
      }
    }
    pop_scene();
  }
  // Always clear these on up
  is_rotating = false;
}

void PupSkin::mouse_drag(int mouse_x, int mouse_y)
{
  using namespace igl;
  using namespace igl::embree;
  using namespace std;
  using namespace Eigen;
  bool used = false;
  // modifiers
  const int mod = 0;
  const int height = four_view.height();

  four_view.drag(mouse_x,mouse_y);

  if(is_rotating)
  {
    glutSetCursor(GLUT_CURSOR_CYCLE);
    Quaterniond q;
    auto & vp = *(four_view.down_viewport());
    auto & camera = vp.camera;
    int mod_down_x = down_x;
    int mod_mouse_x = mouse_x;
    if(mirror_mode)
    {
      mod_down_x = four_view.width()-mod_down_x;
      mod_mouse_x = four_view.width()-mod_mouse_x;
    }
    switch(rotation_type)
    {
      case ROTATION_TYPE_IGL_TRACKBALL:
      {
        // Rotate according to trackball
        igl::trackball(
          vp.width,
          vp.height,
          2.0,
          four_view.down_camera().m_rotation_conj,
          vp.mouse_x(mod_down_x),
          vp.mouse_y(down_y,height),
          vp.mouse_x(mod_mouse_x),
          vp.mouse_y(mouse_y,height),
          q);
          break;
      }
      case ROTATION_TYPE_TWO_AXIS_VALUATOR_FIXED_UP:
      {
        // Rotate according to two axis valuator with fixed up vector 
        two_axis_valuator_fixed_up(
          vp.width, vp.height,
          2.0,
          four_view.down_camera().m_rotation_conj,
          vp.mouse_x(mod_down_x),
          vp.mouse_y(down_y,height),
          vp.mouse_x(mod_mouse_x),
          vp.mouse_y(mouse_y,height),
          q);
        break;
      }
      default:
        break;
    }
    camera.orbit(q.conjugate());
    used = true;
  }else
  {
    if(const AugViewport * down_viewport = four_view.down_viewport())
    {
      glViewport(
          down_viewport->x,
          down_viewport->y,
          down_viewport->width,
          down_viewport->height);
      push_scene(*down_viewport);

      if(mouse.drag(mouse_x,mouse_y))
      {
      }else
      {
        if(m.getF().size() > 0)
        {
          m.push_matrix();
          double z = 0;
          Vec3 obj,win;
          Eigen::Matrix4f model,proj;
          Eigen::Vector4f viewport;
          igl::opengl2::model_proj_viewport(model,proj,viewport);
          Eigen::Vector2f pos(mouse_x,height-mouse_y);

          int nhits = igl::embree::unproject_in_mesh(
              pos,model,proj,viewport,ei,obj);

          igl::opengl2::project(obj,win);
          m.pop_matrix();
          z = win(2);
          if(get_root() && !cpu_lbs)
          {
            deque<Node*> Q = deque<Node*>(1,get_root());
            while(Q.size()>0)
            {
              Node * n = Q.front(); 
              Q.pop_front();
              if(nhits>0)
              {
                used |= n->drag_arrow.drag(mouse_x,height-mouse_y,z,mod);
              }else
              {
                used |= n->drag_arrow.drag(mouse_x,height-mouse_y,mod);
              }
              // add children
              Node::add_children(n,Q,Q.end());
            }
          }
          used |= m.drag(mouse_x,height-mouse_y);
          if(get_root())
          {
            deque<Node*> Q = deque<Node*>(1,get_root());
            while(Q.size()>0)
            {
              Node * n = Q.front(); 
              Q.pop_front();
              if(n->get_translate_on() && cpu_lbs)
              {
                cerr<<REDGIN("Can't apply offsets while skinnin'")<<endl;
              }else{
                used |= n->drag(mouse_x,height-mouse_y);
              }
              // add children
              Node::add_children(n,Q,Q.end());
            }
          }
        }
      }
      pop_scene();
    }
  }
  if(!used)
  {
    TwEventMouseMotionGLUT(mouse_x, mouse_y);
  }
}

void PupSkin::mouse_move(int mouse_x, int mouse_y)
{
  using namespace std;
  TwEventMouseMotionGLUT(mouse_x, mouse_y);
  hover_next = true;
  hover_x = mouse_x;
  hover_y = mouse_y;
  four_view.hover(mouse_x,mouse_y);
}

void PupSkin::mouse_wheel(int wheel, int direction, int mouse_x, int mouse_y)
{
  using namespace std;
  if(wheel == 0)
  {
    static double mouse_scroll_y = 0;
    const double delta_y = 0.125*direction;
    mouse_scroll_y += delta_y;
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);
    if(TwMouseMotion(mouse_x, mouse_y))
    {
      TwMouseWheel(mouse_scroll_y);
    }else if(m.is_selected)
    {
      if(!cpu_lbs)
      {
        //m.camera.zoom *= (1.0+double(direction)*z_diff);
      }
    }else if(AugViewport * in_vp = four_view.in_viewport(mouse_x,mouse_y))
    {
      if(!zoom_lock)
      {
        auto & camera = in_vp->camera;
        // factor of zoom change
        double s = (1.-0.01*direction);
        camera.push_away(s);
      }
    }
  }else if(AugViewport * in_vp = four_view.in_viewport(mouse_x,mouse_y))
  {
    if(!angle_lock)
    {
      auto & camera = in_vp->camera;
      if(!is_rotating)
      {
        // Dolly zoom:
        camera.dolly_zoom((double)direction*1.0);
      }
    }
  }
}

bool PupSkin::next_background()
{
  using namespace igl;
  using namespace std;
  static int id = 0;
  const auto id_to_path = [](const int id)->std::string
  {
    return STR("data/cinekid/background-"<<id<<".png");
  };
  int max_image_id = -1;
  while(true)
  {
    if(file_exists(id_to_path(max_image_id+1)))
    {
      max_image_id++;
    }else
    {
      break;
    }
  }
  if(max_image_id<0)
  {
    cout<<REDRUM("Failed to load background; none available.")<<endl;
    return false;
  }
  const std::string path = id_to_path(id%(max_image_id+1));

  if(glIsTexture(background_tex_id))
  {
    glDeleteTextures(1,&background_tex_id);
  }
  if(igl::png::texture_from_png(path,background_tex_id))
  {
    cout<<GREENGIN("Loaded "<<path<<" successfully.")<<endl;
    id++;
  }else
  {
    cout<<REDRUM("Loading "<<path<<" failed.")<<endl;
    return false;
  }
  return true;
}

bool PupSkin::pick(int mouse_x, int mouse_y, float * pixel)
{
  using namespace igl;
  glClearColor(0,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  bool old_is_pickle = Pickle::is_picking;
  Pickle::is_picking = true;

  draw_scene();

  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT,viewport);
  glReadPixels(mouse_x,four_view.height()-mouse_y,1,1,GL_RGBA,GL_FLOAT,(void *)pixel);

  Pickle::is_picking = old_is_pickle;
  return ( pixel[0] != 0 || pixel[1] != 0 || pixel[2] != 0 );
}

bool PupSkin::pr_is_attached()
{
  using namespace std;
  if(pr)
  {
    return pr->is_attached();
  }else
  {
    cout<<"NULLLLLL"<<endl;
  }
  return false;
}

void PupSkin::init_rig_animation_is_listening()
{
  using namespace std;
  switch(control_type)
  {
    case CONTROL_TYPE_MOUSE:
      if(mouse.selection().rows() == 
        (int)rig_animation.bone_animations().size())
      {
        rig_animation.set_is_listening(mouse.selection());
      }
      break;
    default:
    case CONTROL_TYPE_PUPPET:
      if(BE.rows() == (int)rig_animation.bone_animations().size())
      {
        // only select those that are bound
        RigAnimation::VectorXb S = 
          RigAnimation::VectorXb::Constant(BE.rows(),1,false);
        // Always skip root!
        for(int n = 1;n<nodes_to_rig.rows();n++)
        {
          if(nodes_to_rig(n)>=0)
          {
            S(nodes_to_rig(n)) = true;
          }
        }
        rig_animation.set_is_listening(S);
      }
      break;
  }
}

bool PupSkin::next_project()
{
  prefix_in_use++;
  int loaded = load(prefixes[prefix_in_use%prefixes.size()],PUPSKIN_LOAD_ALL);
  return loaded;
}

void PupSkin::start_playing()
{
  RotationList vdQ;
  if(rig_pose_from_control(vdQ))
  {
    if(rig_animation.keyframes_size()==0)
    {
      rig_animation.start_to_from_identity(vdQ);
    }else
    {
      rig_animation.start_playing(vdQ);
    }
  }
}
