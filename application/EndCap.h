#ifndef ENDCAP_H
#define ENDCAP_H
#include "Node.h"

// EndCap "Node" contains no electronics and does not annouce itself
//
//
class EndCap : public Node
{
  // Public fields
  public:
  // public functions
  public: 
    // Instanciate an EndCap object. The first time this is called an
    // initialization is performed where mesh data is read from files and
    // stored into memory.
    EndCap();
    virtual void draw_self() const;
    virtual void rigid_to_child(const int i, Quat & q, Vec3 & v) const;
    virtual void set_is_selected(const bool v);
    virtual tinyxml2::XMLElement * appendXML( 
      tinyxml2::XMLDocument & doc, 
      tinyxml2::XMLElement * parent) const;
};
#endif

