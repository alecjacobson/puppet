#include "tree_fit.h"
#include "fit_rotation.h"

#include <igl/repdiag.h>
#include <igl/sparse.h>
#include <igl/find.h>
#include <igl/colon.h>
#include <igl/repmat.h>
#include <igl/slice.h>
#include <igl/active_set.h>
#include <igl/mat_max.h>
#include <igl/cat.h>
#include <igl/matlab_format.h>
#include <igl/mosek/mosek_quadprog.h>

#include <cassert>
#include <algorithm>
#include <cmath>

//// Debugin'
//template <typename Mat>
//int rows(const Mat & M)
//{
//  return M.rows();
//}
//template <typename Mat>
//int cols(const Mat & M)
//{
//  return M.cols();
//}
//template <typename Mat>
//typename Mat::Scalar at(const Mat & M,const int i, const int j)
//{
//  return M(i,j);
//}

// http://stackoverflow.com/a/4609795/148668
template <typename T> static int sgn(T val) {
      return (T(0) < val) - (val < T(0));
}

// DRAG_ENERGY Construct a set of linear equality constraints to be treated
// as a least squares energy of the form:
//     E_user(N) = || A*[S;N(:)] - B ||^2
// such that new positions of nodes (N) are attracted toward their
// old/initial positions (O).
//
// Inputs:
//   r  number of root nodes (aka n-#S)
//   O  #O by dim list of initial vertex positions
// Outputs:
//   A  #O*dim by #S+#O*dim sparse matrix
//   B  #O*dim vector
//
template <typename DerivedO,typename SparseMatrixXS, typename VectorXS>
void drag_energy(
  const int r,
  const Eigen::PlainObjectBase<DerivedO> & O,
  SparseMatrixXS & A,
  VectorXS & B)
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;
  const int dim = O.cols();
  const int n = O.rows();
  SparseMatrixXS I(n*dim,n*dim);
  I.setIdentity();
  A = cat(2,SparseMatrixXS(n*dim,n-r),I);
  B.resize(O.size());
  copy(O.data(),O.data()+O.size(),B.data());
}
// DAMPENING_ENERGY Construct a set of linear equality constraints to be treated
// as a least squares energy of the form:
//     E_user(N) = || A*[S;N(:)] - B ||^2
// such that new lengths of edges are attracted toward their old/initial
// lengths (S).
//
// Inputs:
//   n  number of nodes
//   r  number of root nodes
//   dim  number of dimensions
//   S_original  #S list of original edge lengths
// Outputs:
//   A  #S by #S+#O*dim sparse matrix
//   B  #S vector
//
template <typename VectorXS,typename SparseMatrixXS>
void dampening_energy(
  const int n,
  const int r,
  const int dim,
  const VectorXS & S_original,
  SparseMatrixXS & A,
  VectorXS & B)
{
  using namespace igl;
  SparseMatrixXS I(n-r,n-r);
  I.setIdentity();
  SparseMatrixXS Z(n-r,n*dim);
  cat(2,I,Z,A);
  B = S_original;
}

// GRAPH_SMOOTHNESS_ENERGY Construct a set of linear equality constraints to
// be treated as a least squares energy of the form:
//     E_lapl(S) = || A*[S;N(:) - B ||^2
// such that new lengths S are punished if not similar to their neighbors'
// lengths in the tree.
// 
// Inputs:
//   dim  #dimensions
//   P  #O list of parent indices (0 means root)
// Outputs:
//   A  #S by #S+#O*dim sparse matrix
//   B  #S vector
template <typename SparseMatrixXS, typename VectorXS>
void graph_smoothness_energy(
  const int n,
  const int dim,
  Eigen::VectorXi & INR,
  Eigen::VectorXi & PNR,
  SparseMatrixXS & A,
  VectorXS & B)
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;
  // number of non-root nodes
  const int nnr = INR.size();
  // edge to node adjacency
  VectorXi S2NI(nnr*2);
  S2NI << colon<int>(0,nnr-1), colon<int>(0,nnr-1);
  SparseMatrixXS S2S,S2N;
  // Edge to node adjacency
  sparse(S2NI,cat(1,INR,PNR),VectorXS::Ones(nnr*2),nnr,n,S2N);
  // edge to edge adjacency
  {
    SparseMatrixXS S2NT,S2Sd,S2Stemp,I(nnr,nnr);
    S2NT = S2N.transpose().eval();
    S2Stemp = S2N * S2NT;
    I.setIdentity();
    S2S = S2Stemp - I * (S2Stemp.diagonal()).asDiagonal();
  }
  {
    VectorXi I,J;
    VectorXS _;
    find(S2S,I,J,_);
    VectorXi AI(I.size()*2),AJ(I.size() + J.size());
    AI <<I,I;
    AJ <<I,J;
    VectorXS AV(I.size() + J.size());
    AV<<
      VectorXS::Ones(I.size()),
      VectorXS::Constant(J.size(),-1.);
    sparse(AI,AJ,AV,nnr,nnr+n*dim,A);
  }
  B = VectorXS::Zero(nnr);
}


// USER_DESIRED_POSITIONS_ENERGY Construct a set of linear equality
// constraints to be treated as a least squares energy of the form:
//     E_user(N) = || A*[S;N(:)] - B ||^2
// such that new positions of selected nodes (N(b,:)) are attracted toward
// user specified locations (bc).
//
// Inputs:
//   n  number of nodes
//   r  number of root nodes (aka n-#S)
//   b  #b list of soft-constrained indices into O
//   bc  #b by dim list of soft-constraint positions
// Outputs:
//   A  #O by #S+#O*dim sparse matrix
//   B  #O vector
//
template <typename Derivedb, typename Derivedbc, typename SparseMatrixXS, typename VectorXS>
void user_desired_positions_energy(
    const int n,
    const int r,
    const Eigen::PlainObjectBase<Derivedb> & b,
    const Eigen::PlainObjectBase<Derivedbc> & bc,
    SparseMatrixXS & A,
    VectorXS & B)
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;
  const int dim = bc.cols();
  const int nnr = n-r;
  const int nb = b.size();
  VectorXi AI = colon<int>(0,nb*dim-1);
  VectorXi AJ(nb*dim);
  for(int d = 0;d<dim;d++)
  {
    for(int i = 0;i<nb;i++)
    {
      AJ(d*nb + i) = nnr + d*n + b(i);
    }
  }
  sparse(AI,AJ,VectorXS::Ones(nb*dim),nb*dim,nnr+n*dim,A);
  B.resize(bc.size());
  copy(bc.data(),bc.data()+bc.size(),B.data());
}


// RELATIVE_ANGLE_CONSTRAINTS Construct a set of linear equality constraints
// of the form:
//     B = A * [S;N(:)]
// such that relative angles between nodes are maintained w.r.t. their
// original positions (O).
//
// Inputs:
//   V  #S by dim list of original edge vector directions
//   II  #S by #O matrix taking nodes to edge destinations
//   PP  #S by #O matrix taking nodes to edge parents
//   R  dim by dim global rotation
// Outputs:
//   A  #S*dim by #S+#O*dim sparse matrix
//   B  #S*dim vector
//
template <typename MatrixXS, typename SparseMatrixXS, typename DerivedR, typename VectorXS>
void relative_angle_constraints(
    const MatrixXS & V,
    const SparseMatrixXS & II,
    const SparseMatrixXS & PP,
    const Eigen::PlainObjectBase<DerivedR> & R,
    SparseMatrixXS & A,
    VectorXS & B)
{
  using namespace igl;
  using namespace Eigen;
  using namespace std;


  // number of nodes
  const int dim = V.cols();
  // number of non-root nodes
  const int nnr = PP.rows();
  // rhs
  B = VectorXS::Zero(nnr*dim);
  VectorXi AI = colon<int>(0,nnr*dim-1);
  VectorXi AJ(nnr*dim);
  VectorXS AV(nnr*dim);
  MatrixXS VR = V*R;
  for(int d = 0;d<dim;d++)
  {
    AJ.block(nnr*d,0,nnr,1) = colon<int>(0,nnr-1);
    AV.block(nnr*d,0,nnr,1) = VR.col(d);
  }
  SparseMatrixXS AS,AN;
  sparse(AI,AJ,AV,nnr*dim,nnr,AS);
  repdiag((PP-II).eval(),dim,AN);
  A = cat(2,AS,AN);
}

template <
  typename DerivedO,
  typename DerivedP,
  typename Derivedb,
  typename Derivedbc,
  typename DerivedN,
  typename DerivedS,
  typename DerivedR>
igl::SolverStatus tree_fit(
  const Eigen::PlainObjectBase<DerivedO> & O,
  const Eigen::PlainObjectBase<DerivedP> & P,
  const Eigen::PlainObjectBase<Derivedb> & b,
  const Eigen::PlainObjectBase<Derivedbc> & bc,
  const TreeFitParams & params,
  Eigen::PlainObjectBase<DerivedN> & N,
  Eigen::PlainObjectBase<DerivedS> & S,
  Eigen::PlainObjectBase<DerivedR> & R)
{
  using namespace Eigen;
  using namespace igl;
  using namespace std;
  // Computation types
  typedef typename DerivedS::Scalar Scalar;
  typedef Matrix<Scalar,Dynamic,1> VectorXS;
  typedef Matrix<Scalar,Dynamic,Dynamic> MatrixXS;
  typedef SparseMatrix<Scalar> SparseMatrixXS;

  // Return value
  SolverStatus ret = SOLVER_STATUS_ERROR;

  // Number of vertices
  const int n = O.rows();
  // Number of dimensions
  const int dim = O.cols();
  assert(n == P.size());
  assert(b.size() == bc.rows());
  assert(b.size() == 0 || b.maxCoeff() < n);
  assert(b.size() == 0 || b.minCoeff() >= 0);
  assert(bc.size() == 0 || bc.cols() == dim);
  // Number of roots
  const int r = count(P.data(),P.data()+P.size(),-1);
  const int nnr = n-r;

  // Ensure vector of minimum lengths
  VectorXS Smin;
  if(params.Smin.size() == nnr)
  {
    Smin = params.Smin;
  }else if(params.Smin.size() == 1)
  {
    Smin = VectorXS::Constant(nnr,1,params.Smin(0,0));
  }else
  {
    assert(false);
  }

  // Indices of non-roots, parents of non-roots
  VectorXi I = colon<int>(0,n-1);
  VectorXi INR = I;
  if(I.size() == 1)
  {
    // single root
    INR.resize(0,1);
  }else
  {
    INR.conservativeResize(stable_partition(
        INR.data(), 
        INR.data()+INR.size(), 
        [&P](int i){return P(i)!=-1;})-INR.data());
  }
  VectorXi PNR(INR.size());
  for(int i = 0;i<INR.size();i++)
  {
    PNR(i) = P(INR(i));
  }
  SparseMatrixXS II,PP;
  sparse(colon<int>(0,nnr-1),INR,VectorXS::Ones(nnr),nnr,n,II);
  sparse(colon<int>(0,nnr-1),PNR,VectorXS::Ones(nnr),nnr,n,PP);
  // Edge vectors
  MatrixXS V = II*O - PP *O;
  VectorXS S_original = V.rowwise().norm();
  // normalize
  V.rowwise().normalize();

  R.setIdentity(dim,dim);

  int iter = 0;
  N = O;
  S = S_original;
  VectorXS Z_prev(nnr+n*dim,1);
  // Z = [S;N(:)];
  Z_prev.topLeftCorner(nnr,1) = S;
  copy(N.data(),N.data()+N.size(),Z_prev.data()+nnr);

  VectorXS meanbc = bc.colwise().mean().transpose();
  VectorXS meanO = O.colwise().mean().transpose();

  while(true)
  {
    VectorXS meanNb = VectorXS::Zero(dim,1);
    for_each(b.data(),b.data()+b.size(),[&meanNb,&N](int i){meanNb += N.row(i);});
    meanNb /= b.size();
    VectorXS meanN = N.colwise().mean().transpose();
    MatrixXS C = MatrixXS::Zero(dim,dim);
    for(int i = 0;i<b.size();i++)
    {
      VectorXS bcm = bc.row(i).transpose() - meanbc;
      VectorXS Nbm = N.row(b(i)).transpose() - meanNb;
      C += params.w_user/(params.w_user + params.w_drag) *
        (bcm * Nbm.transpose());
    }
    for(int i = 0;i<n;i++)
    {
      VectorXS Om = O.row(i).transpose() - meanO;
      VectorXS Nm = N.row(i).transpose() - meanN;
      C += params.w_drag/(params.w_user + params.w_drag) *
        (Om * Nm.transpose());
    }
    MatrixXS dR;
    fit_rotation(C,dR);
    // Apply rotations to current solution
    N = (N * dR.transpose()).eval();
    // Accumulate rotations
    R = (R * dR.transpose()).eval();

    SparseMatrixXS A_drag,A_damp,A_user,A_smooth,Aeq,A;
    VectorXS B_drag,B_damp,B_user,B_smooth,Beq,B;
    drag_energy(r,O,A_drag,B_drag);
    dampening_energy(n,r,dim,S_original,A_damp,B_damp);
    graph_smoothness_energy(n,dim,INR,PNR,A_smooth,B_smooth);
    user_desired_positions_energy(n,r,b,bc,A_user,B_user);
    relative_angle_constraints(V,II,PP,R,Aeq,Beq);

    A_drag *= params.w_drag;
    A_damp *= params.w_damp;
    A_smooth *= params.w_smooth;
    A_user *= params.w_user;
    B_drag *= params.w_drag;
    B_damp *= params.w_damp;
    B_smooth *= params.w_smooth;
    B_user *= params.w_user;

    A = cat(1, A_drag, cat(1, A_user, cat(1, A_smooth, A_damp)));
    B = cat(1, B_drag, cat(1, B_user, cat(1, B_smooth, B_damp)));
    const double INF = 1e26;
    VectorXS lx(nnr + n*dim),ux(nnr + n*dim);
    // Constant bound constraints
    lx << Smin,VectorXS::Constant(n*dim,-INF);
    ux << VectorXS::Constant(nnr+n*dim,INF);
    // No inequality constraints
    SparseMatrixXS Aieq;
    VectorXS Bieq;

    // Solution vector
    VectorXS Z;
    const bool use_as = false;
    if(use_as)
    {
      // Build quadratic term and linear term
      SparseMatrixXS AT = A.transpose().eval();
      SparseMatrixXS Q = 2.*AT * A;
      VectorXS lin = -2.*AT*B;
      VectorXi known;
      VectorXS Y;
      active_set_params as_params;
      as_params.Auu_pd = true;
      SolverStatus as_ret = 
        active_set(Q,lin,known,Y,Aeq,Beq,Aieq,Bieq,lx,ux,as_params,Z);
      if(as_ret == SOLVER_STATUS_ERROR)
      {
        ret = SOLVER_STATUS_ERROR;
        break;
      }
    }else
    {
      SparseMatrixXS AT = A.transpose().eval();
      SparseMatrixXS Q = 2.*AT*A;
      VectorXS lin = -2.*AT*B;
      VectorXS uc(Beq.size()+Bieq.size());
      uc<< Beq,Bieq;
      VectorXS lc(Beq.size()+Bieq.size());
      lc<< Beq,VectorXS::Constant(Bieq.size(),1,-1e26);
      SparseMatrixXS AA = cat(1,Aeq,Aieq);
      MosekData mosek_data;
      mosek_quadprog(Q,lin,0,AA,lc,uc,lx,ux,mosek_data,Z);
    }
    S = Z.block(0,0,nnr,1);
    for(int d = 0;d<dim;d++)
    {
      N.col(d) = Z.block(nnr+d*n,0,n,1);
    }

    Scalar dZ = (Z-Z_prev).array().abs().maxCoeff();
    Z_prev = Z;

    if(dZ < params.min_dZ)
    {
      ret = SOLVER_STATUS_CONVERGED;
      break;
    }

    if(iter < params.max_iter)
    {
      ret = SOLVER_STATUS_MAX_ITER;
      break;
    }
    iter = iter + 1;
  }

  return ret;
}

// Explicit tempalte specialization 
template igl::SolverStatus tree_fit<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, 1, 0, -1, 1>, Eigen::Matrix<int, -1, 1, 0, -1, 1>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, 1, 0, -1, 1>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 1, 0, -1, 1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, 1, 0, -1, 1> > const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, TreeFitParams const&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, 1, 0, -1, 1> >&, Eigen::PlainObjectBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> >&);
