#include "find_selected.h"
#include <deque>

std::vector<Node *> find_selected(Node * root)
{
  using namespace std;
  vector<Node*> selected;
  deque<Node*> Q = deque<Node*>(1,root);
  while(Q.size()>0)
  {
    Node * n = Q.front(); 
    Q.pop_front();
    if(n->get_is_selected())
    {
      selected.push_back(n);
    }
    // add children
    vector<Node*> c(n->get_children()); Q.insert(Q.end(), c.begin(), c.end());
  }
  return selected;
}

