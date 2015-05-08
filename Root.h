#ifndef ROOT_H
#define ROOT_H
#include "Node.h"

// Root joint 
//
//
// Let's define a "rest state", lies along the x-axis like:
//
//                         ___      
//                   -----/   |     
// z<---.  wire ---> |___|    | --> child
//     /|                 \___|    
//    / |               
//   y  x                            
//
//
class Root : public Node 
{
  // Public fields
  public:
  // public functions
  public: 
    // Instanciate a Root object. The first time this is called an
    // initialization is performed where mesh data is read from files and
    // stored into memory.
    Root();
    virtual void draw_self() const;
    virtual void rigid_to_child(const int i, Quat & q, Vec3 & v) const;
    virtual bool right_down(const int x, const int y, const int modifier=0);
    virtual void set_is_selected(const bool v);
    virtual tinyxml2::XMLElement * appendXML( 
      tinyxml2::XMLDocument & doc, 
      tinyxml2::XMLElement * parent) const;
    virtual void reset_offsets();
};

#endif

