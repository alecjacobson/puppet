#ifndef TBT_H
#define TBT_H
#include "Node.h"

#include "GL_include.h"

#include <igl/lens_flare.h>
#include <vector>

// Callback prototype for announcing angles change
class TBT;
typedef void (*AnnounceTareCB) (void *, const TBT &);

// Twist-Bend-Twist (TBT) joint 
//
//
// Let's define a "rest state" if all the puppet angles are PI then our joint
// lies along the z-axis like:
//
//                       θ₀   θ₁   θ₂
//                     H|^| /```\ |^|
// z<---.  parent --->  | ||  .  || | ---> child
//     /|              H|_| \__L/ |_|
//    / |               
//   y  x               |---- d --- |
//
// Where increasing θ₀ rotates around z-axis (right-hand rule)
//       increasing θ₁ rotates around y-axis (right-hand rule)
//       increasing θ₂ rotates around z-axis (right-hand rule)
//       L means the little LED
//       H are the little hooks coming out of the male socket
//       d is the length of the joint at rest minus the length of the hooks so
//         that if one was to connect k joints the total length would be k*d
//
//
//
class TBT : public Node 
{
  // Public static fields
  public:
    static const double axis[][3];
    // vectors orthogonal to axis (using left-hand rule)
    static const double biaxis[][3];
    // Impose virtual limits on joint angles
    static bool angle_limits;
    static bool LEDs_visible;
    static std::vector<GLuint> shine_ids;
    static std::vector<GLuint> flare_ids;
    static int shine_tic;
    static bool textures_initialized;
  // Public fields
  public:
    // Angle values as described above, measured in degrees
    double angle[3];
    // Cached angle values at bind time
    double angle_at_bind[3];
    // Hover for each angle
    bool angle_is_hover[3];
    bool angle_is_selected[3];
    bool angle_is_down[3];
    bool angle_drag[3];
    void * param_announce_tare;
    AnnounceTareCB announce_tare;
    // Frequencies of RGB LEDs (should match device)
    int LED_periods[3];
  // public functions
  public: 
    // Instanciate a TBT object. The first time this is called an
    // initialization is performed where mesh data is read from files and
    // stored into memory.
    TBT();
    // Copy constructor: not allowed
    TBT(const TBT & that);
    // Assignment operator: not allowd
    TBT & operator=(const TBT& that);
  public:
    // "Tare" current angles to be the new zero angle of the device
    // Returns true only on success from device
    bool tare();
    // Draw the joint parts according to angles
    virtual void draw_self() const;
    void draw_LEDs() const;
    virtual void rigid_to_child(const int i, Quat & q, Vec3 & v) const;
    virtual void rigid_to_child_at_bind(const int i, Quat & q, Vec3 & v) const;
    void bind();
    virtual bool down(const int x, const int y, const int modifier=0);
    virtual bool right_down(const int x, const int y, const int modifier=0);
    virtual bool drag(const int x, const int y, const int modifier=0);
    virtual void set_is_hover(const bool v);
    virtual void set_is_selected(const bool v);
    virtual void set_is_down(const bool v);
    virtual tinyxml2::XMLElement * appendXML( 
      tinyxml2::XMLDocument & doc, 
      tinyxml2::XMLElement * parent) const;
    virtual void parseXML(const tinyxml2::XMLElement * root);
  public:
    // Convert a unit quaternion representation of a rotation in a sequence of
    // TBT rotations around the TBT axes
    //
    // Inputs:
    //   q  unit quaternion
    // Returns 3 "Euler" angles in degrees
    static Vec3 quat_to_TBT_angles(const Quat & q);
    // Convert a sequence of TBT angles into a quaternion
    //
    // Inputs:
    //   theta_0  TBT angle in degrees
    //   theta_1  TBT angle in degrees
    //   theta_2  TBT angle in degrees
    // Returns quaternion representation of rotation
    static Quat TBT_angles_to_quat(
      const double theta_0,
      const double theta_1,
      const double theta_2);
    static void initialize_textures();
};

#endif
