#include "Node.h"
#include "Mesh.h"
#include "bind.h"
#include <igl/read.h>
#include <igl/writeDMAT.h>
#include <igl/bbw/bbw.h>
#include <string>
#include <iostream>
int main(int argc, char * argv[])
{
  // ./bind_test mesh.obj nodes.xml
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  Mesh m;
  read(argv[1],m.dgetV(),m.dgetF());
  m.compute_scale_and_shift();
  Node * root = Node::readXML(argv[2]);
  BBWData bbw_data;
  bbw_data.active_set_params.max_iter = 4;
  bbw_data.active_set_params.Auu_pd = true;
  bbw_data.verbosity = 1;
  MatrixXd W,M,WV;
  ::bind(m,bbw_data,root,W,M);
  WV = W.block(0,0,m.getV().rows(),W.cols());
  writeDMAT("W.dmat",WV);
  return 0;
}
