#include "tree_fit.h"

#include <igl/matlab/MatlabWorkspace.h>
#include <igl/matlab_format.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/active_set.h>
#include <igl/get_seconds.h>

#include <iostream>

int main(int argc, char * argv[])
{
  using namespace igl;
  using namespace std;
  using namespace Eigen;

  MatlabWorkspace mw;
  if(argc<=2)
  {
    cerr<<"Usage:"<<endl<<"  ./tree_fit_test input.mat out.mat"<<endl;
  }
  mw.read(argv[1]);
  MatrixXd O,bc,N,R;
  VectorXd S;
  VectorXi P,b;
  TreeFitParams params;
  params.max_iter = 1;

  // Load from .mat
  if(!mw.find("O",O))
  {
    cerr<<" 'O' not found in "<<argv[1]<<endl;
    return 1;
  }
  if(!mw.find("bc",bc))
  {
    cerr<<" 'bc' not found in "<<argv[1]<<endl;
    return 1;
  }
  if(!mw.find_index("P",P))
  {
    cerr<<" 'P' not found in "<<argv[1]<<endl;
    return 1;
  }
  if(!mw.find_index("b",b))
  {
    cerr<<" 'b' not found in "<<argv[1]<<endl;
    return 1;
  }
  if(!mw.find("max_iter",params.max_iter))
  {
    cerr<<" 'max_iter' not found in "<<argv[1]<<". Using default..."<<endl;
  }

  tree_fit(O,P,b,bc,params,N,S,R);
  mw.clear();
  mw.save(N,"N");
  mw.save(S,"S");
  mw.save(R,"R");
  mw.write(argv[2]);
  return 0;
}

