#include "bind.h"
#include "match_skeletons.h"
#include "Mesh.h"
#include "Node.h"
#include <igl/tetgen/mesh_with_skeleton.h>
#include <igl/boundary_conditions.h>
#include <igl/bbw/bbw.h>
#include <igl/normalize_row_sums.h>
#include <igl/REDRUM.h>
#include <igl/lbs_matrix.h>
#include <igl/matlab_format.h>
#include <igl/writeDMAT.h>
#include <igl/writeMESH.h>
#include <igl/writeTGF.h>
#include <igl/sparse.h>
#include <igl/colon.h>
#include <deque>
#include <algorithm>
#include <iostream>

void bind(
  const Eigen::MatrixXd & rC,
  const Eigen::MatrixXi & rBE,
  Node * root,
  Eigen::VectorXi & I)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;

  // Bind all nodes
  deque<Node*> Q = deque<Node*>(1,root);
  while(Q.size()>0)
  {
    Node * n = Q.front(); 
    Q.pop_front();
    n->bind();
    vector<Node*> c(n->get_children()); Q.insert(Q.end(), c.begin(), c.end());
  }

  // Match to rig skeleton
  MatrixXd C;
  MatrixXi BE;
  VectorXi P,RP;
  Node::matlab(root,true,C,BE,P,RP);
  match_skeletons(rC,rBE,C,BE,I);

  cout<<matlab_format(I.transpose().eval(),"I")<<endl;
}
