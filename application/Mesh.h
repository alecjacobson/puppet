#ifndef MESH_H
#define MESH_H
#include "Pickle.h"
#include "SkinningMethod.h"
#include <igl/NormalType.h>

// Lightweight mesh class
//#include <Eigen/Dense>
#include "EigenConvenience.h"
#include "eigen_typedef.h"

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

class Mesh : public Pickle
{
  private:
    Eigen::MatrixXd V;
    Eigen::MatrixXi F,NF,TCF;
    // Per vertex normals
    Eigen::MatrixXd VN;
    // Per face normals
    Eigen::MatrixXd FN;
    // Per corner normals
    Eigen::MatrixXd CN;
    // Texture coords
    Eigen::MatrixXd TC;
    // Name for opengl name stack
    GLuint name;
    // Normal type
    igl::NormalType normal_type;
    // Invert orientation
    bool invert_orientation;
  public:
    // TODO: remove this, it is redundant with dl_id
    bool display_list_compiled;
    // Display list id
    GLuint dl_id;
    // Texture id
    GLuint tex_id;
    double scale;
    Vec3 shift;
    double y_boost;
    float color[4];
    Eigen::Quaterniond rotation;
    bool is_hover;
    bool is_selected;
    bool is_down;
  public:
    Mesh();
    void compute_scale_and_shift();
    // Push rendering matrix
    void push_matrix() const;
    // Transform an input set of vertices to be relative to this mesh
    //
    // Input:
    //   C  #C by dim list of input points
    // Output:
    //   C  #C by dim list of transformed points
    void relative_to_mesh(Eigen::MatrixXd & C) const;
    void unrelative_to_mesh(Eigen::MatrixXd & C) const;
    // Returns mesh as its drawn
    Eigen::MatrixXd as_drawn() const;
    // Pop rendering matrix
    void pop_matrix() const;
    // Const getters
    const Eigen::MatrixXd & getV() const{ return V;}
    const Eigen::MatrixXi & getF() const{ return F;}
    const Eigen::MatrixXi & getTCF() const{ return TCF;}
    const Eigen::MatrixXi & getNF() const{ return NF;}
    const Eigen::MatrixXd & getTC() const{ return TC;}
    const Eigen::MatrixXd & getVN() const{ return VN;}
    const Eigen::MatrixXd & getFN() const{ return FN;}
    const Eigen::MatrixXd & getCN() const{ return CN;}
    GLuint get_name() const {return name;}
    const igl::NormalType & get_normal_type() const { return normal_type;}
    // "Dirty" getters which trigger that mesh (may be) is dirty, and display list
    // should be recompiled
    Eigen::MatrixXd & dgetV() { display_list_compiled = false; return  V;}
    Eigen::MatrixXi & dgetF() { display_list_compiled = false; return  F;}
    Eigen::MatrixXi & dgetTCF() { display_list_compiled = false; return  TCF;}
    Eigen::MatrixXi & dgetNF() { display_list_compiled = false; return  NF;}
    Eigen::MatrixXd & dgetTC() { display_list_compiled = false; return  TC;}
    Eigen::MatrixXd & dgetVN(){ display_list_compiled = false; return VN;}
    Eigen::MatrixXd & dgetFN(){ display_list_compiled = false; return FN;}
    Eigen::MatrixXd & dgetCN(){ display_list_compiled = false; return CN;}
    igl::NormalType & dget_normal_type(){ display_list_compiled = false; return normal_type;}
    // Draw *without* pushing and poping matrices
    void draw();
    // Draw and cache in a display list
    // This method *does* call push_matrix and pop_matrix respectively
    void draw_and_cache();
    // Sort triangles
    void sort();
    // Called when is_down has just been set 
    //
    // Inputs:
    //   mouse_x  x location of mouse
    //   mouse_y  y location of mouse
    // Returns is_down
    bool down(int mouse_x, int mouse_y);
    // Called when mouse is dragging 
    //
    // Inputs:
    //   mouse_x  x location of mouse
    //   mouse_y  y location of mouse
    // Returns is_down
    bool drag(int mouse_x, int mouse_y);
    // Called when is_up has just been set
    //
    // Inputs:
    //   mouse_x  x location of mouse
    //   mouse_y  y location of mouse
    // Returns is_down
    bool up(int mouse_x, int mouse_y);
    bool set_invert_orientation(const bool v);
    bool get_invert_orientation() const;
    // Create a copy of this mesh deformed by the given parameters
    //
    // Inputs:
    //   skinning_meshod  lbs or dqs
    //   W  #V by #W list of weights
    //   vQ  #W list of rotations as quaternions
    //   vT  #W list of translations
    //   M  #V by #W*(3+1) lbs matrix
    bool deformed_copy(
      const SkinningMethod & skinning_method,
      const Eigen::MatrixXd & W,
      const std::vector<
        Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond> > & vQ,
      const std::vector<Eigen::Vector3d> & vT,
      const Eigen::MatrixXd & M,
      Mesh & m_def
        ) const;
    bool is_textured();
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
