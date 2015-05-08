#include "set_node_scale.h"
#include "Node.h"
#include "eigen_typedef.h"
#include <deque>
#include <iostream>

void set_node_scale(const Node * root)
{
  if(!root)
  {
    return;
  }
  using namespace std;
  using namespace Eigen;
  // Absolute values (besides 0) don't really make sense
  double desired_h = 0.0;
  deque<const Node*> Q = deque<const Node*>(1,root);
  bool some_children = false;
  double min_s = 1e26;
  double max_s = -1e26;
  while(Q.size()>0)
  {
    const Node * n = Q.front(); 
    Q.pop_front();
    for(int c = 0;c<n->get_max_children();c++)
    {
      if(!n->get_child(c))
      {
        continue;
      }
      Quat qq;
      Vec3 vv;
      n->rigid_to_child(c,qq,vv);
      // Assume vv is of the form (0,0,z)
      vv = (qq.conjugate() * vv);
      // Assumes that ct_off is only interesting in z coordinate
      double ct = -n->ct_off[c](2);
      //ct = (ct<0?0:ct);
      const double v = vv.norm();
      const double t = n->get_child(c)->t_off.norm();
      //desired_h = 0.5*(v+c);
      const double s = (ct-desired_h)/(v+t) + 1;
      min_s = (s<min_s?s:min_s);
      max_s = (s>max_s?s:max_s);
      some_children = true;
    }
    // add children
    vector<const Node*> c(n->get_const_children()); Q.insert(Q.end(), c.begin(), c.end());
  }
  // only set if there were some children
  if(some_children)
  {
    Node::scale = min_s*Node::user_scale;
  }
  //cout<<"min_s: "<<min_s<<endl;
  //cout<<"max_s: "<<max_s<<endl;
}
