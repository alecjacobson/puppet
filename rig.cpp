#include "rig.h"
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

bool rig(
  const Mesh & m,
  igl::BBWData & bbw_data,
  const Node * root,
  Eigen::MatrixXd & W,
  Eigen::MatrixXd & M,
  Eigen::MatrixXd & TV,
  Eigen::MatrixXi & TT,
  Eigen::MatrixXi & TF)
{
  using namespace std;
  using namespace igl;
  using namespace Eigen;

  // Get skeleton in (C,BE) format
  MatrixXd C;
  MatrixXi BE;
  VectorXi P,RP;
  Node::matlab(root,false,C,BE,P,RP);
  // Transform relative to mesh
  m.relative_to_mesh(C);

  // Convert mesh to tetmesh sampled at bones
  // Mesh with samples on skeleton
  MatrixXi CE;
  // Phony list of point handles, cage edges
  VectorXi _P;
  if(!mesh_with_skeleton(m.getV(),m.getF(),C,_P,BE,CE,10,TV,TT,TF))
  {
    return false;
  }

  // Compute boundary conditions (aka fixed value constraints)
  // List of boundary indices (aka fixed value indices into TV)
  VectorXi b;
  // List of boundary conditions of each weight function
  MatrixXd bc;
  if(!boundary_conditions(TV,TT,C,_P,BE,CE,b,bc))
  {
    cout<<REDRUM("boundary conditions failed (bone entirely outside?).")<<endl;
    return false;
  }
  assert(BE.rows() == bc.cols());

  // Combine boundary conditions for rigid parts.
  const bool combine_rigid_parts = true;
  if(combine_rigid_parts)
  {
    SparseMatrix<double> A;
    sparse(
      colon<int>(0,RP.size()-1),
      RP,
      VectorXd::Ones(RP.size()),
      bc.cols(),RP.maxCoeff()+1,A);
    bc = (bc * A).eval();
  }

  // Weights matrix
  if(!bbw(TV,TT,b,bc,bbw_data,W))
  {
    return false;
  }
  // Clamp to [0,1]
  for(int v = 0;v<W.rows();v++)
  {
    for(int b = 0;b<W.cols();b++)
    {
      W(v,b) = (W(v,b)>0?(W(v,b)<1?W(v,b):1):0);
    }
  }
  // Normalize to sum to 1
  normalize_row_sums(W,W);
  // Immediately compute lbs matrix
  lbs_matrix(m.as_drawn(),W,M);
  //writeMESH("rig.mesh",TV,TT,TF);
  //writeDMAT("rig.dmat",W);
  //writeTGF("rig.tgf",C,BE);
  //writeDMAT("rig-RP.dmat",RP);
  

  return true;
}

bool rig(
  const Mesh & m,
  igl::BBWData & bbw_data,
  const Node * root,
  Eigen::MatrixXd & W,
  Eigen::MatrixXd & M)
{
  Eigen::MatrixXd TV;
  Eigen::MatrixXi TT,TF;
  return rig(m,bbw_data,root,W,M,TV,TT,TF);
}

