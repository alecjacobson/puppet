#ifndef FOURVIEW_H
#define FOURVIEW_H
#include "AugViewport.h"

namespace igl
{
  class ReTwBar;
}

// Implements a multi-viewport display like Maya's "Four View"
class FourView
{
  // NUM_VIEWPORTS  number of viewports (always 4)
  // m_width  width of containing window
  // m_height height 〃 
  // m_down_vp  last viewport that accepted a mouse down event
  // m_hover_vp                                    〃 move event
  // m_horiz_on  whether mouse is currently dragging horizontal bar
  // m_vert_on                                    〃  vertical bar
  // m_horiz  ratio of height at which horizontal bar is drawn
  // m_vert                          〃 vertical bar is drawn
  public:
    static constexpr int NUM_VIEWPORTS = 4;
  private:
    static constexpr double BAR_THICKNESS = 3.0;
    static constexpr double SNAP_DIST = 10;
    int m_width, m_height;
    int m_down_vp, m_hover_vp;
    bool m_horiz_on, m_vert_on;
    double m_horiz, m_vert;
    igl::Camera m_down_camera;
  public:
    AugViewport m_viewports[NUM_VIEWPORTS];
  public:
    FourView();
    // Return const reference to m_width
    const int & width() const{ return m_width;};
    //                        〃  m_height
    const int & height() const{ return m_height;};
    //                        〃  m_down_vp
    const int & down_vp() const{ return m_down_vp;};
    //                        〃  m_hover_vp
    const int & hover_vp() const{ return m_hover_vp;};
    // Return clamped `horiz` value so that it places the bar just in view.
    double clamped_horiz() const;
    // Return clamped `vert` 〃
    double clamped_vert() const;
    // Reshapes viewports according to new container width's and heights
    //
    // Inputs:
    //   w  new container width
    //   h  new container height
    void reshape(const int w, const int h);
    // Viewports are arranged like planar quadrants (CCW) according to m_horiz
    // and m_vert
    // /-----.-----\ .
    // |  1  |  0  | .
    // ------------- .
    // |  2  |  3  | .
    // \-----.-----/ .
    void reshape();
    // Draws viewport bars
    void draw() const;
    // Process mouse down, drag, hover and up. Catch down_vp, hover_vp
    // 
    // Inputs:
    //   x  x-coordinate of mouse click with respect to container
    //   y  y-coordinate 〃 
    // Returns true if accepted.
    bool down(const int x, const int y);
    bool drag(const int x, const int y);
    bool hover(const int x, const int y);
    bool up(const int x, const int y);
    // Add items for each viewport to a reanttweakbar instance
    //
    // Input:
    //   rebar  reanttweakbar object 
    void add_to_reanttweakbar(igl::ReTwBar & rebar);
    // Sets visibility of AntTweakbar items according to whether each vp is the
    // current down_vp
    // 
    // Input:
    //   rebar  reanttweakbar object used during add_to_reanttweakbar
    void update_anttweakbar_visibility(igl::ReTwBar & rebar) const;
    // Return pointer to camera of last down viewport cached at down.
    igl::Camera & down_camera();
    // Returns pointer to viewport of last down. Returns null if m_down_vp == -1.
    AugViewport * down_viewport();
    // Returns true if any viewports rotation is animating
    bool any_rotation_animating()const;
    // Find which viewport (x,y) is inside
    // 
    // Inputs:
    //   x  x-coordinate
    //   y  y-coordinate
    // Returns index of viewport inside or -1 iff not in any
    int in_viewport_index(const int x, const int y) const;
    // Returns pointer to viewport inside or NULL iff not in any
    AugViewport * in_viewport(const int x, const int y);
    // Snap all viewport camera angles to fixed up
    void snap_to_fixed_up();

  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
