#ifndef SPLITTER_H
#define SPLITTER_H
#include "Node.h"




#include "eigen_typedef.h"


// Callback prototype for announcing angles change
class Splitter;
typedef void (*AnnounceAnglesChangeCB) (void *, const Splitter &);

class Splitter : public Node
{
  // Public fields
  public:
    void * param_announce_angles_change;
    AnnounceAnglesChangeCB announce_angles_change;
  //// Protected fields
  //protected:
    // 3 Angles for each child
    std::vector<std::vector<double> > angles;
    // selection for "child" (really plug for child)
    std::vector<bool > child_is_hover;
    std::vector<bool > child_is_down;
    std::vector<bool > child_is_selected;
    std::vector<bool > child_drag;
    std::vector<std::vector<double> > down_angles;
  private:
    bool is_hand;
  // Public functions
  public:
    Splitter(const int max_children);
  private:
    // Copy constructor: not allowed
    Splitter(const Splitter & that);
    // Assignment operator: not allowd
    Splitter & operator=(const Splitter& that);
  public:
    void draw_self() const;
    void rigid_to_child(const int i, Quat & q, Vec3 & v) const;
    virtual bool down(const int x, const int y, const int modifier=0);
    virtual bool right_down(const int x, const int y, const int modifier=0);
    virtual bool drag(const int x, const int y, const int modifier=0);
    virtual void set_is_hover(const bool v);
    virtual void set_is_selected(const bool v);
    virtual void set_is_down(const bool v);
    void set_is_hand(const bool v);
    bool get_is_hand() const;
    virtual tinyxml2::XMLElement * appendXML( 
      tinyxml2::XMLDocument & doc, 
      tinyxml2::XMLElement * parent) const;
    virtual void parseXML(const tinyxml2::XMLElement * root);
    // Set angles of all children (expects to have TBT joints in each)
    void set_angles(bool only_if_selected=false);
    // Returns const reference to angles
    const std::vector<std::vector<double> > & get_angles() const;
};

#endif
