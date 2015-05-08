#include "Node.h"
#include "Mesh.h"
#include "bind.h"
#include "forward_kinematics.h"
#include "throw_at.h"
#include "quattrans2affine.h"
#include "list_affine_to_stack.h"
#include <igl/matlab_format.h>
#include <string>
#include <iostream>

int main(int argc, char * argv[])
{
  // ./bind_test nodes.xml
  using namespace std;
  using namespace Eigen;
  using namespace igl;
  Node * root = Node::readXML(argv[1]);
  MatrixXd T,TRP,Tfk,TfkRP;
  Node::transformations(root,false,T);
  Node::transformations(root,true,TRP);
  cout<<matlab_format(T,"T")<<endl;
  cout<<matlab_format(TRP,"TRP")<<endl;
  MatrixXd C;
  MatrixXi BE;
  VectorXi P,RP;
  Node::matlab(root,true,C,BE,P,RP);
  vector<Quaterniond,aligned_allocator<Quaterniond> > dQ,vQ;
  {
    vector<Vector3d> _;
    Node::transformations(root,false,dQ,_);
  }
  vector<Vector3d> vT;
  vector<Affine3d,aligned_allocator<Affine3d> > vA,vARP;
  forward_kinematics(C,BE,P,dQ,vQ,vT);
  ::quattrans2affine(vQ,vT,vA);
  list_affine_to_stack(vA,Tfk);
  throw_at(vA,RP,vARP);
  list_affine_to_stack(vARP,TfkRP);
  cout<<matlab_format(Tfk,"Tfk")<<endl;
  cout<<matlab_format(TfkRP,"TfkRP")<<endl;
  return 0;
}
