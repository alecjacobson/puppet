#ifndef NODE_H
#define NODE_H

#include "Pickle.h"
#include "DragArrow.h"
#include "GL_include.h"

#include <vector>
#include <deque>
#include "EigenConvenience.h"
#include "eigen_typedef.h"

#include <AntTweakBar.h>

//enum NodeType
//{
//  NT_TBT,
//  NT_SPLITTER_ONE_TO_FIVE_REGULAR,
//  NT_SPLITTER_ONE_TO_TWO_Y,
//  NT_BONE,
//  NT_ROOT
//};
//#define NUM_NODE_TYPES 5

// Matrix of scalars, e.g. #V by dim list of vertex positions
typedef Eigen::MatrixXd ScalarMat;
// Matrix of indices, e.g. #F by |simplex| list of face indices
typedef Eigen::MatrixXi IndexMat;

class Mesh;
namespace tinyxml2
{
  class XMLDocument;
  class XMLElement;
}


#define NULL_PARENT_ID -1

enum NodeParts
{
  TBT_PLUG_PART = 0,
  TBT_BASE_PART = 1,
  TBT_FOLLOWER_PART = 2,
  TBT_SOCKET_PART = 3,
  SPLITTER_C_PART = 4,
  SPLITTER_Y_PART = 5,
  BONE_PART = 6 ,
  WIRE_SOCKET_PART = 7,
  TERMINATOR_PART = 8,
  NUM_NODE_PARTS = 9 // This is not a real part, but the number of parts
};

class Node;
typedef void (*AnnounceIsSelectedChangeCB) (void *, const Node &);

enum NodeType
{
  NODE_TYPE_ROOT = 0,
  NODE_TYPE_TBT = 1,
  NODE_TYPE_SPLITTER = 2,
  NODE_TYPE_ENDCAP = 3,
  NODE_TYPE_UNKNOWN = 4,
  NUM_NODE_TYPES = 5 // Not a real type
};

enum RotationControlType
{
  ROTATION_CONTROL_TYPE_TRACKBALL = 0,
  ROTATION_CONTROL_TYPE_AXIS = 1,
  NUM_ROTATION_CONTROL_TYPES = 2
};


enum RotationAxisType
{
  ROTATION_AXIS_TYPE_VIEW = 0,
  ROTATION_AXIS_TYPE_UP = 1,
  ROTATION_AXIS_TYPE_RIGHT = 2,
  NUM_ROTATION_AXIS_TYPES = 3
};

class Node
{
  // Public static fields
  public:
    static Mesh part[NUM_NODE_PARTS];
    static const float * part_color[NUM_NODE_PARTS];
    // Shifts and scale node parts for drawing. scale can be set automatically
    // using set_node_scale and user_scale is applied on top of that. These
    // scales do not affect distances between joints.
    static double shift[3], scale, user_scale;
    // Global scale affects not just drawings but also default distances
    // between new joints.
    static const double GLOBAL_SCALE;
    static bool visible;
    static TwBar * bar;
    static bool beach_balls;
    static bool drag_along_axis;
    static bool show_bind;
    static RotationControlType rotation_control_type;
    static RotationAxisType rotation_axis_type;
    // Color individual parts (regardless of node's color)
    static bool color_parts;
    // Whether twbar nodetype has been initialized
    static bool node_type_tw_initialized;
    static TwType NodeTypeTW;
    // Whether to draw numbers
    static bool draw_numbers;
    static const double hover_dim;
    static const double selected_dim;
    // show rotation and translation controls
    static bool show_controls;
  // Public fields
  public:
    // Display opacity (assumes triple pass OpenGL transparency)
    double alpha;
    // Distance from origin to input/output (along z-axis)
    double radius;
    // Rigid transformation controlling translation and rotation offsets
    Quat r_off;
    // displacement from input to origin
    Vec3 t_off;
    // displacement from input to origin of each child
    Vec3 * ct_off;
    // Color (read from device)
    Eigen::Vector4f color;
    // Pickable pickles
    std::vector<Pickle> pickles;
    std::vector<bool> pickle_hit;
    void * param_announce_is_selected_change;
    AnnounceIsSelectedChangeCB announce_is_selected_change;
    DragArrow drag_arrow;
  // Protected fields
  protected:
    Node * parent;
    // id in parent's child array, such that this == parent->child[parent_id]
    // NULL_PARENT_ID if parent is NULL
    int parent_id;
    // child[i] is NULL if ith slot is empty
    Node ** child;
    const int max_children;
    bool is_hover;
    bool is_selected;
    bool is_down;
    // List of current anttweakbar items by name
    std::vector<std::string> tw_vars;
    // Unique name for opengl name stack
    GLuint name;
  // TODO: put these in a "dragable" class
    int down_x;
    int down_y;
    int down_z;
    int last_x;
    int last_y;
    Quat down_r_off;
    Vec3 down_t_off;
    bool rotate_on;
    bool translate_on;
    // projection and model view matrix on last draw
  public:
    // variables for UI for adding nodes dynamically
    NodeType new_node_type;
    int new_node_cid;
    int new_node_max_children;

  // Public functions
  public: 
    // Inputs:
    //   max_children  maximum number of children allowed at this node
    Node(const int max_children);
    // Copy constructor: This does a rather shallow copy. It copies any
    // non-pointer values and sets pointer values (children and parents) to
    // default values. It does *not* copy all children (only endcaps)
    //
    // Inputs:
    //   that  node to copy from
    Node(const Node & that);
    // Assignment. Rather shallow assignment. See copy constructor
    //
    // Inputs:
    //   that  node to copy from
    Node & operator=(const Node& that);
  public:
    // Removes self from parent's child list and deletes any children
    virtual ~Node();
    // Push and pop transformations called before drawing self, but after
    // calling pushmv()
    void push() const;
    void pop() const;
    // Push and pop last seen modelview matrix (mv)
    void pushmv() const;
    void popmv() const;
    // Return the current origin used at last draw
    Vec3 origin() const;
    Vec3 origin_at_bind() const;
    // unproject mouse click to 3d position at depth of down origin
    // Inputs:
    //   x  x position of mouse click
    //   y  y position of mouse click
    // Outputs:
    //   u  3d position 
    void unproject( const int x, const int y, double * u) const;
    // Project z-axis onto screen, then project input (x,y) point on to that
    // line, finally unproject that point back to space
    //
    // Inputs:
    //   x  x position of mouse click
    //   y  y position of mouse click
    // Outputs:
    //   u  3d position 
    void unproject_to_screen_z( const int x, const int y, double * u) const;
    // Draw the node, which may consist of many parts. 
    //
    // TODO: Expectation of GL state before and explanation of GL state after
    void draw() const;
    // Draw parts of this node (should be overloaded by super class)
    //
    // Expects that before called the origin has been moved to the "center" of
    // this node, with the frame aligned with the incoming direction.
    //
    // Leaves state of origin and frame in tact
    virtual void draw_self() const{};
    // Helper to rotation control widgets
    virtual void draw_rotation_controls() const;
    // Helper to translation control widgets
    virtual void draw_translation_controls() const;
    // Compute rigid transformation that moves origin and from center (where
    // this joint is drawn (draw_self()) to where the child i should be drawn
    // (child[i]->draw())
    //
    // Inputs:
    //   i  index of child in question
    //   q  rotation of rigid transformation
    //   v  translation of rigid transformation
    virtual void rigid_to_child(const int /*i*/,Quat& q, Vec3& v) const
    {
      q = Quat(1,0,0,0);
      v = Vec3(0,0,0);
    };
    virtual void rigid_to_child_at_bind(const int i,Quat&q,Vec3&v) const 
    {
      return rigid_to_child(i,q,v);
    }
    // Compute rigid transformation to self from identity (root). Self being
    // the frame in which the parts are drawn (before this Node's angles have
    // been applied)
    //
    // Outputs:
    //   q  rotation of rigid transformation
    //   t  translation of rigid transformation
    void rigid_to_self(Quat & q,Vec3 & t) const;
    void rigid_to_self_at_bind(Quat & q,Vec3 & t) const;
    // Cache enough values to remember bind state
    virtual void bind() {};
    // Sets child at location cid. 
    //
    // Inputs:
    //   c  pointer to new child c
    //   cid  location of new child
    // Returns pointer to child node previously at cid (NULL if empty)
    Node * set_child(Node * c, const int cid);
    // Removes child at location cid. Same as:
    //   set_child(NULL,cid)
    //
    // Inputs:
    //   cid  location of child to be removed
    // Returns pointer to child node previously at cid (NULL if empty)
    Node * remove_child(const int cid);
    // Return point to cidth child
    Node * get_child(const int cid) const;
    // Return const pointer to parent or NULL if root
    const Node * get_parent() const;
    // Return parent id
    int get_parent_id() const;
    // Return vector of non-null children
    std::vector<Node*> get_children() const;
    std::vector<const Node*> get_const_children() const;
    // Return max_children
    int get_max_children() const;
    // Return unique name
    GLuint get_name() const;
    //// Return a list which includes this node and all ancestors
    //const std::vector<Node*> me_and_everybody_i_know() const;
    // Called when is_down has just been set 
    //
    // Inputs:
    //   x  x location of mouse
    //   y  y location of mouse
    // Returns is_down
    virtual bool down(const int x, const int y, const int modifier = 0);
    // " but for right mouse
    virtual bool right_down(const int x, const int y, const int modifier = 0);
    // Called when mouse is dragging 
    //
    // Inputs:
    //   x  x location of mouse
    //   y  y location of mouse
    // Returns is_down
    virtual bool drag(const int x, const int y, const int modifier = 0);
    // Called when is_down has just been unset 
    //
    // Inputs:
    //   x  x location of mouse
    //   y  y location of mouse
    // Returns is_down 
    virtual bool up(const int x, const int y, const int modifier = 0);
    // Sets is_* and appropriately adds or removes UI elements to
    // anttweakbar
    //
    // Inputs
    //   v  new is_* value
    virtual void set_is_hover(const bool v);
    virtual void set_is_selected(const bool v);
    virtual void set_is_down(const bool v);
    // Retrieve is_* values
    bool get_is_hover() const;
    bool get_is_selected() const;
    bool get_is_down() const;
    bool get_translate_on() const;
    // Helper function for writing XML
    //
    // Inputs:
    //   doc  pointer to xml doc
    //   parent  pointer to parent node
    // Returns pointer to node element corresponding to this, null on error
    virtual tinyxml2::XMLElement * appendXML( 
      tinyxml2::XMLDocument & doc, 
      tinyxml2::XMLElement * parent) const;
    // Helper function for read XML. Reads in local information about node
    // (does *not* recurse on children)
    //
    // Inputs:
    //  root  root of XML corresponding to this node 
    virtual void parseXML(const tinyxml2::XMLElement * root);
    // Clear all "state" stored in a Node: offsets like t_off, r_off, ct_off(:)
    void clear_state();
    // Reset effective offsets of this Node  (including parents ct_off)
    virtual void reset_offsets();
  // Public static functions
  public:
    // Export Node heirarchy starting at given root to TGF/matlab style format.
    // 
    // Inputs:
    //   root  root node of tree
    //   at_bind  get positions from bind locations 
    // Outputs:
    //   C  #Nodes by dim list of node positions
    //   BE  #Nodes-1 by 2 list of edges indices into C
    //   P  #Nodes-1 list of parent indices into BE
    //   RP  #Nodes-1 list of rigid piece IDs
    // Optional inputs:
    static void matlab(
      const Node * root,
      const bool at_bind,
      Eigen::MatrixXd & C,
      Eigen::MatrixXi & BE,
      Eigen::VectorXi & P,
      Eigen::VectorXi & RP);
    //// Slightly different format
    //// Inputs:
    ////   root  root node of tree
    //// Outputs:
    ////   C  #Nodes by dim list of node positions
    ////   P  #Nodes list of parent indices (-1 means root)
    //static void origins_and_parents(
    //  const Node * root,
    //  Eigen::MatrixXd & C,
    //  Eigen::MatrixXi & P);
    // Slightly different format
    // Export transformations taking bind nodes to current nodes
    //
    // Inputs:
    //   root  root node of tree
    //   select_rigid  whether to consider only rigid pieces
    // Outputs:
    //   T  #BE*4 by 3  stacked matrix of transposed transformations, order is
    //     same as BE produced by matlab(...)
    template <typename DerivedT>
    static void transformations(
      const Node * root,
      const bool select_rigid,
      Eigen::PlainObjectBase<DerivedT>& T);
    // rigid version
    template <typename Q, typename QAlloc, typename T>
    static void transformations(
      const Node * root,
      const bool select_rigid,
      std::vector<Q,QAlloc> & vQ,
      std::vector<T> & vT);
    // Relative rotations for each node
    template <typename Q_type, typename QAlloc>
    static void relative_rotations(
      const Node * root,
      std::vector<Q_type,QAlloc> & vdQ);
    // Load a node (tree) from XML file
    //
    // Inputs:
    //   filename  path to .xml file
    // Returns new root node, null on error
    static Node * readXML( const std::string filename);
    // Write a node (tree) to an XML file
    //
    // Inputs:
    //   filename  path to .xml file
    //   n  pointer to node 
    // Returns true on success, false on failure
    static bool writeXML(const std::string filename, const Node * n);
    // Return type from given string
    //
    // Inputs:
    //   str  should be a string name from node_type_strings
    // Returns matching node type or NODE_TYPE_UNKNOWN
    static NodeType type_from_string(const std::string str);
    // Create a new node (by calling new !) based on the given string
    //
    // Inputs:
    //   type  type of Node implementation 
    //   max_children  max_children of new node (should be valid for given
    //     type)
    // Returns pointer to new node (NULL on error)
    static Node * new_Node(const NodeType type, const int max_children);
    // Append n's non-null children to Q before Qit
    //
    // Inputs:
    //   n  node whose children we're appending to
    //   Q  queue of nodes
    //   Qit  iterator in Q before which we'll insert
    static void add_children(
        const Node * n,
        std::deque<Node*> & Q,
        const std::deque<Node*>::iterator & Qit);
    // Const version
    static void add_children(
        const Node * n,
        std::deque<const Node*> & Q,
        const std::deque<const Node*>::iterator & Qit);
    //template <typename return_type,typename func_type>
    //static return_type for_all(
    //  Node * root,
    //  const func_type & func);
  public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
