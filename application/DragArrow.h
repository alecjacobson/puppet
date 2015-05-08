#ifndef DRAG_ARROW_H
#define DRAG_ARROW_H
#include "eigen_typedef.h"
class DragArrow
{
  public:
    Vec3 from;
    Vec3 to;
    bool activated;
  private:
    bool is_down,snapping,static_from,delay_snap;
    double down_x,down_y;
    Vec3 down_to,down_from;
  public:
    DragArrow();
    void draw();
    // Inputs:
    //  x  mouse x-coordinate
    //  y  mouse y-coordinate
    //  mod  modifiers
    // Returns whether down was accepted
    bool down(const int x, const int y, const int mod);
    // z  depth
    bool drag(const int x, const int y, const int mod);
    bool drag(const int x, const int y, const double z, const int mod);
    bool up(const int x, const int y, const int mod);
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
