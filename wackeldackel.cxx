#include <PuppetReader.h>
#include <TBT.h>

#include <igl/get_seconds.h>
#include <igl/colon.h>
#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/PI.h>
#include <igl/cat.h>
#include <igl/lbs_matrix.h>
#include <igl/deform_skeleton.h>
#include <igl/per_face_normals.h>
#include <igl/dqs.h>
#include <igl/readDMAT.h>
#include <igl/readMESH.h>
#include <igl/svd3x3/arap.h>
#include <igl/viewer/Viewer.h>
#include <igl/partition.h>
#include <igl/massmatrix.h>
#include <igl/REDRUM.h>
#include <igl/PI.h>
#include <igl/is_dir.h>
#include <igl/dirname.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>
#include <unistd.h>

PuppetReader * pr;

typedef 
  std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> >
  RotationList;

const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
Eigen::MatrixXd V,U,bc;
Eigen::MatrixXi T,F;
Eigen::VectorXi S,b,b1;
double anim_t = 0.0;
double anim_t_dir = 0.03;
igl::ARAPData arap_data;
bool is_animating = true;
bool not_yet_warned = true;

bool init_puppet()
{
  using namespace Eigen;
  using namespace std;
  pr = new PuppetReader("ignored",115200);
  return pr->is_attached();
}

const TBT * get_const_tbt(const PuppetReader & pr)
{
  if(const Node * r = pr.get_const_root())
  {
    return dynamic_cast<const TBT *>(r->get_child(0));
  }
  return NULL;
}

bool pre_draw(igl::Viewer & viewer)
{
  using namespace Eigen;
  using namespace std;
  using namespace igl;
  for(int i = 0;i<b1.size();i++)
  {
    const Vector3d p = V.row(b1(i));
    const Vector3d o(0,0.28,0.125);
    Quaterniond rotation;
    rotation.setIdentity();
    if(pr->is_attached())
    {
      pr->sync();
      const TBT * t = get_const_tbt(*pr);
      if(t != NULL)
      {
        not_yet_warned = true;
        vector<Quaterniond,aligned_allocator<Quaterniond> > dQ;
        Node::relative_rotations(pr->get_const_root(),dQ);
        assert(dQ.size() >= 2);
        Quaterniond q = 
          Quaterniond(AngleAxisd(PI/2,Vector3d(1,0,0)))*
          Quaterniond(AngleAxisd(PI/2,Vector3d(0,1,0)));
        rotation = q* dQ[1]* q.conjugate();
      }else if(not_yet_warned)
      {
        cout<<REDRUM("ERROR: Plug in just a single TBT.")<<endl;
        not_yet_warned = false;
      }
    }else
    {
      if(S(b1(i))==1)
      {
        rotation = 
          AngleAxisd(cos(1.1*anim_t*2.*igl::PI),Vector3d(0,1,1))*
          AngleAxisd(1.4*anim_t*2.*igl::PI,Vector3d(0,0,1));
      }
    }
    bc.row(i) = rotation* (p-o)+o;
  }
  igl::arap_solve(bc,arap_data,U);
  viewer.data.set_vertices(U);
  viewer.data.compute_normals();
  if(is_animating)
  {
    anim_t += anim_t_dir;
  }
  return false;
}

bool key_down(igl::Viewer &viewer, unsigned char key, int mods)
{
  switch(key)
  {
    case ' ':
      //viewer.core.is_animating = !viewer.core.is_animating;
      is_animating = !is_animating;
      return true;
  }
  return false;
}

int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;
  //igl::readOBJ("data/wackeldackel/wackeldackel.obj",V,F);
  string data_dir = "./data/wackeldackel/";
  string wd = getcwd(NULL,0);
  {
    string wd = getcwd(NULL,0);
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
    string dir = igl::dirname(exe);
    chdir(dir.c_str());
  }
  if(!igl::is_dir(data_dir.c_str()))
  {
    data_dir = "wackeldackel.app/Contents/Resources/wackeldackel/";
    if(!igl::is_dir(data_dir.c_str()))
    {
      return -1;
    }
  }
  {
    string wd = getcwd(NULL,0);
    cout<<"wackeldackel.cxx: Current working directory: "<<wd<<endl;
  }
  igl::readMESH(data_dir+"wackeldackel.mesh",V,T,F);
  igl::readDMAT(data_dir+"wackeldackel-selection.dmat",S);
  igl::readDMAT(data_dir+"wackeldackel-partition.dmat",arap_data.G);

  U=V;

  // vertices in selection
  VectorXi b0;
  igl::colon<int>(0,V.rows()-1,b0);
  b0.conservativeResize(stable_partition( b0.data(), b0.data()+b0.size(), 
   [](int i)->bool{return S(i)==0;})-b0.data());
  igl::colon<int>(0,V.rows()-1,b1);
  b1.conservativeResize(stable_partition( b1.data(), b1.data()+b1.size(), 
   [](int i)->bool{return S(i)==1;})-b1.data());
  b = igl::cat(1,b1,b0);
  // Precomputation
  arap_data.max_iter = 5;
  arap_data.with_dynamics = true;
  arap_data.h = 0.05;
  //MatrixXd W,E;
  //E.resize(V.rows(),V.cols()+W.cols());
  //E<<W,V;
  //VectorXi _S;
  //VectorXd _D;
  //igl::partition(V,100,arap_data.G,_S,_D);
  igl::arap_precomputation(V,T,V.cols(),b,arap_data);
  SparseMatrix<double> M;
  igl::massmatrix(V,T,igl::MASSMATRIX_TYPE_DEFAULT,M);
  const double grav_mag = 0.5;
  arap_data.f_ext = grav_mag*M*RowVector3d(0,-1,0).replicate(V.rows(),1);

  bc.resize(b.size(),V.cols());
  for(int i = 0;i<b.size();i++)
  {
    bc.row(i) = V.row(b(i));
  }

  init_puppet();

  // Plot the mesh with pseudocolors
  igl::Viewer viewer;
  viewer.data.set_mesh(U, F);
  //viewer.data.set_colors(C);
  // Compute per-face normals
  MatrixXd N;
  igl::per_face_normals(V,F,N);
  viewer.data.set_normals(N);
  viewer.callback_pre_draw = &pre_draw;
  viewer.callback_key_down = &key_down;
  viewer.core.is_animating = true;
  viewer.core.animation_max_fps = 30.;
  viewer.core.show_lines = false;
  viewer.core.trackball_angle = Vector4f(0.03,-0.27,-0.01,0.96);
  viewer.core.camera_zoom = 2.5;
  cout<< "Press [space] to toggle animation"<<endl;
  viewer.launch();
}

