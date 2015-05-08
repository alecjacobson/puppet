#ifndef PICKLE_H
#define PICKLE_H

#if __APPLE__
#  include <OpenGL/gl.h>
#else
#  ifdef _WIN32
#    define NOMINMAX
#    include <Windows.h>
#    undef NOMINMAX
#  endif
#  include <GL/gl.h>
#endif

// A class for things that are pickable
class Pickle
{
  public:
    // Are we currently in pick mode?
    static bool is_picking;
  private:
    // Unique pick color
    float pick_color[4];
  public:
    Pickle();
    // Note a true copy construct, need unique colors for vectors of pickles
    Pickle(const Pickle & /*that*/);
    virtual ~Pickle(){}
    // returns const pointer to unique pick color
    const float* get_pick_color() const;
  public:
    // Activate pick color using opengl calls. 
    // this then it will ruin it
    //
    void activate_pickle() const;
    // Depending on is_picking, either activates pickle or passes call to
    // diffuse material
    void activate_pickle_or_diffuse_material(
      const float * diffuse, 
      const float alpha) const;
    // Compute difference (norm) between two colors, this and that
    //
    // Inputs:
    //   that  another color
    // Returns norm of difference in pick_color values
    float diff(const float * that);
};
#endif
