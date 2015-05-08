#include "NodeCallbacks.h"
#include "TBT.h"
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/REDRUM.h>
#include <iostream>

void TW_CALL NodeCallbacks::deleteCB(void *clientData)
{
  Node * n = static_cast<Node*>(clientData);
  delete n;
  n=NULL;
}

void TW_CALL NodeCallbacks::snapCB(void *clientData)
{
  Node * n = static_cast<Node*>(clientData);
  igl::snap_to_canonical_view_quat<double>(
    n->r_off.coeffs().data(),
    1.0,
    n->r_off.coeffs().data());
}

void TW_CALL NodeCallbacks::zeroCB(void *clientData)
{
  TBT * n = static_cast<TBT*>(clientData);
  n->angle[0] = 0;
  n->angle[1] = 0;
  n->angle[2] = 0;
}

void TW_CALL NodeCallbacks::new_childCB(void *clientData)
{
  using namespace std;
  Node * n = static_cast<Node*>(clientData);
  switch(n->new_node_type)
  {
    case NODE_TYPE_ENDCAP:
    {
      if(n->new_node_max_children != 0)
      {
        cerr<<MAGENTAGIN("Warning: setting new_node_max_children to 0")<<endl;
        n->new_node_max_children = 0;
      }
      break;
    }
    case NODE_TYPE_TBT:
    {
      if(n->new_node_max_children != 1)
      {
        cerr<<MAGENTAGIN("Warning: setting new_node_max_children to 1")<<endl;
        n->new_node_max_children = 1;
      }
      break;
    }
    case NODE_TYPE_SPLITTER:
    {
      if(n->new_node_max_children <= 1)
      {
        cerr<<MAGENTAGIN("Warning: setting new_node_max_children to 2")<<endl;
        n->new_node_max_children = 2;
      }
      break;
    }
    default:
    {
      // Bad type (defer error handling)
    }
  }
  Node * new_child = Node::new_Node(n->new_node_type,n->new_node_max_children);
  assert(n->new_node_cid>=0);
  assert(n->new_node_cid<n->get_max_children());
  Node * old_child = n->set_child(new_child,n->new_node_cid);
  delete old_child;
  old_child = NULL;
  if(n->get_is_selected() && !n->get_is_down())
  {
    n->set_is_selected(false);
    new_child->set_is_selected(true);
  }
}

void TW_CALL NodeCallbacks::reset_offsetsCB(void *clientData)
{
  using namespace std;
  Node * n = static_cast<Node*>(clientData);
  n->reset_offsets();
}
