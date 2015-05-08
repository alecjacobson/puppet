#ifndef PUPPETREADER_H
#define PUPPETREADER_H

// Classes
#include "Node.h"
class Splitter;
class TBT;

// Puppet api
//#include <PuppetParserRQ.h>
//#include <SerialLineHandler.h>
#include "PuppetConvenience.h"

#include <Eigen/Core>
//#include <boost/thread/thread.hpp>
#include <thread>
#include <mutex>

#include <string>
#ifdef __APPLE__
#  define USE_D2XX
// NOTE: You should not call serial.update() when defining USE_D2XX
#endif


// Class for buffering data parsed by PuppetParserRQ
class PuppetReader
{
  static const std::string HAND_ID_1;
  public:
    // numbfer of times looped
    int loop_count;
    // number of lines read since instanciation or reset
    int lines_read;
    // total number of lines read since instanciation
    int total_lines_read;
    // Next command to send via UI
    std::string next_command;
    // Print lines as they're parsed
    bool print_lines;
    bool log_lines;
    //// List of commands the computer may send to the device
    //static const char * SERIAL_COMMANDS = 
    //  "b  Stop (or restart?) the serial output\n"
    //  "l,[id],[rate_r],[rate_g],[rate_b]  Set the each RGB LED to a certain "
    //    " rate in ms\n"
    //  "g,[id],[output],[tbt_0],[tbt_1],[tbt_2]  Define the rotation from "
    //    "splitter [id] to its child [output], angles are the theta values of "
    //    "connected TBT joint need to restore identity in degrees.\n";

  private:
    // Awkwardly handles incoming serial data via function handle
#ifdef USE_D2XX
    SerialLineHandler<D2XXSerial> serial;
    //SerialLineHandler<OpenD2XXSerial> serial;
#else
    SerialLineHandler<Serial> serial;
#endif
    // Parses raw serial data into NodeDB and MeasurementMap
    Puppet::PuppetParserRQ pp;
    // Buffered copy of node database
    Puppet::NodeDB buf_nodedb;
    // Buffered copy of node measurements
    Puppet::MeasurementMap buf_measurements;
    // Whether topology has changed since last sync
    bool topology_changed_since_last_sync;
    // Whether measurements have been ready since last sync
    bool measurements_ready_since_last_sync;
    // Thread running main loop
    std::thread loop_thread;
    // Whether looping or should be looping 
    bool looping;
    //bool done_looping;
    // Mutex protecting PuppetParserRQ instance (and serial)
    std::mutex pp_mutex;
    bool got_topology;
    // Drawing objects
    Node * root;
    // Log file object
    FILE * log_file;
    // Map that keeps track of node (attributes) for last seen unique ids
    std::map<std::string,Node*> id_to_live_node;
    std::map<std::string,Node*> id_to_last_node;
    // Attached to device
    bool attached;
    bool is_partying, is_no_LEDs;
  public:
    std::vector< std::string > wrong_topology_measurement;
    std::vector< std::string > correct_topology_measurement;
  // Public Functions
  public:
    // Instanciate interface with a path to the puppet device location
    // Inputs:
    //   devname path to device location (e.g. /dev/tty.Puppet)
    //   speed  serial device speed
    PuppetReader(const std::string devname="", const unsigned int speed=0);
  private:
    PuppetReader(const PuppetReader & that);
    // http://cpp-next.com/archive/2009/08/want-speed-pass-by-value/
    PuppetReader & operator=(PuppetReader that);
  public:
    // Destroy and cleanup
    ~PuppetReader();
    void close();
    // Read a raw line
    //
    // Inputs:
    //   line  raw line string read from device
    void read_line(const std::string &line);
    // Main loop, continously calls update on serial device
    void loop();
    // Callback that is fired (via leapfrog) whenever a new measurement is
    // received
    void new_measurement();
    // Callback that is fired (via leapfrog) whenever a topology has identified
    // (and potentially changed)
    void topology_changed();
    // Return const reference to PuppetParserRQ
    const Puppet::PuppetParserRQ& get_pp();
    // Return const reference to Root 
    const Node * get_const_root() const;
    // Return *non*-const reference to Root. BE CAREFUL. It's expected that
    // pointer values (parents,chidlren) are not changed
    Node * get_root();
    // Sets root node. This will probably be verfied and then possibly replaced
    // by rebuild_topology
    //
    // Inputs:
    //   r  pointer to root drawing node
    // Returns pointer to old root
    Node * set_root(Node * r);
    // Send a command to the serial device
    //
    // Inputs:
    //   s  command to send. The following commands are recognized (no quotes):
    //     "i\r\n"  identify
    //     "l,[id],[r],[g],[b]\r\n"  Set LED color where [id] is the id of the
    //       node, [r],[g],[b] are 1 or 0 specifying on or off for red, green,
    //       blue LEDs respectively
    //     "b\r\n"  toggle stop serial output
    //     "d\r\n"  toggle debug mode
    //     "k\r\n"  kill and restart slaves (power on/off)
    //     see page 19 of "Collected information and instructions"
    //     assembly_plan*.pdf
    void send(const std::string s);
    // Called infrequently with respect to the updates in loop(), this function
    // syncronizes the drawing objects and others connected to the puppet.
    void sync();
    // Syncronize measurements
    void sync_topology();
    // Syncronize topology
    void sync_measurements();
    // Updates measures in tree recursively
    //
    // Inputs:
    //   nodedb  node database (see Puppet code)
    //   measurements  measurement map (see Puppet code)
    //   nid   unique node id (1 is always root collector)
    ////   nit iterator in node database (see Puppet code)
    //   d  display node which should match nit
    // Returns the (subtree) root, which should always be d
    // TODO: meaningful return value
    Node * update_measurements(
      const Puppet::NodeDB & nodedb,
      const Puppet::MeasurementMap & measurements,
      const uint8_t nid,
      Node * d);
    // Update node measurements for a given node and a given record
    //
    // Inputs:
    //  m  measurement corresponding to node linked to d
    //  d  receiving node, should match type as m.type
    // Throws exception if d does not match type of m
    void update_measurement(const Puppet::NodeMeasurement & n, Node * d);
    // Determine whether topology in node database (i.e. from the last parsed
    // identify response) matches the current display topology. "Matching"
    // means that the trees match and each node is the same "type".
    // 
    // Inputs:
    //   nodedb  node database (see Puppet code)
    //   nid   unique node id (1 is always root collector)
    //   d  pointer to drawing node
    // returns true only if topologies are a perfect match
    //
    // Known bug: If you replace a splitter with a different physical splitter
    // then its children might be in a difference order (despite having the
    // same number of children, remember the wires are just a spaghetti mess
    // inside). This will cause verify_topology to return false. It should try
    // all permutations (efficiently with dynamic programming), but instead
    // just change you node tree's xml file to match the new splitter :0(
    //
    // Back up your old (no longer matching) nodes.xml --> old.xml
    // Plug in the splitter save to new.xml
    // Label compare the angles, rearrange the old ones to match the new ones.
    // Repeat with the old.xml file using parent_ids, but leave ct_off's in
    // place.
    bool verify_topology(
      const Puppet::NodeDB & nodedb, 
      const uint8_t nid,
      const Node * d);
    bool verify_topology_robust_to_splitter_spaghetti(
      const Puppet::NodeDB & nodedb, 
      const uint8_t nid,
      Node * d);
    // Determine whether a given node record matches a given drawing node
    // "locally". Where "locally" means we do not check that children (or
    // parents) also match.
    //
    // Inputs:
    //   nid  id of node
    //   d  pointer to drawing node
    // Outputs;
    //   n  node record
    // return true only there is a match
    bool find_if_local_match(
      const Puppet::NodeDB & nodedb, 
      const uint8_t nid,
      const Node * d, 
      Puppet::NodeRecord & n) const;
    bool local_match(
      const Puppet::NodeRecord & n,
      const Node * d) const;
    // Rebuilds non-matching portion of topology 
    //
    // Inputs:
    //   nodedb  node database (see Puppet code)
    //   nid   unique node id (1 is always root collector)
    ////   nit iterator in node database (see Puppet code)
    //   d  display node which should match nit
    // Returns the (subtree) root, potentially new. Returns d on error.
    Node * rebuild_topology(
      const Puppet::NodeDB & nodedb,
      const uint8_t nid,
      //const Puppet::NodeDB::const_iterator & nit, 
      Node * d);
    // Find (first) node id given drawing node
    //
    // Inputs:
    //   q  Node to be found 
    // Outputs:
    //   qid  dynamic id of node matching q
    // Returns whether n was found (and thus whether qid was set)
    bool find(const Node * q, uint8_t & qid);
    // Recursive helper
    //
    // Inputs:
    //   nodedb  node database (see Puppet code)
    //   nid  unique id of d (root is always 1)
    //   d  Root of subtree
    //   q  Node to be found 
    // Outputs:
    //   qid  dynamic id of node matching q
    // Returns whether n was found (and thus whether qid was set)
    bool find(
      const Puppet::NodeDB & nodedb, 
      const uint8_t nid, 
      const Node * d, 
      const Node * q, 
      uint8_t & qid);
    // Sets the splitter angles for a given drawing node splitter
    //
    // Inputs:
    //   s  Splitter drawing node
    // Returns true on success, false on error
    bool set_splitter_angles(const Splitter & s);
    // Set splitter angle of outlet c according to angle stored in drawing node
    // Splitter s.
    //
    // Inputs:
    //   s  Splitter drawing node.
    //   c  index of child (outlet) whose angle should be set
    // Returns true on success, false on error
    bool set_splitter_angle(const Splitter & s,const int c);
    // Helper function to form the actual command and send it to the device.
    //
    // Inputs:
    //   nid  unique id of Splitter
    //   s  Splitter drawing node.
    //   c  index of child (outlet) whose angle should be set
    void send_set_splitter_angle(const uint8_t nid, const Splitter & s, const int c);
    // Sets LEDs according to whether node has been selected or deselected on
    // screen
    //
    // Inputs:
    //   n  drawing node
    // Returns true on success, false on error
    bool set_is_selected(const Node & n);
    void toggle_is_partying();
    void toggle_is_no_LEDs();
    // Set all the LEDs of the device recursively using a function that chooses
    // the color based on the node id.
    //
    // Inputs:
    //   nodedb  node database
    //   nid  id of node
    //   LED_rates_from_nid  function defining LED frequencies from node id.
    void set_LEDs(
      const Puppet::NodeDB & nodedb, 
      const uint8_t nid,
      Eigen::Vector3i (*LED_rates_from_nid)(const uint8_t));
    // Sends command to change light and also tells attached drawing node its
    // new LED colors.
    //
    // Inputs:
    //   nid  id of node
    //   r  red frequency
    //   g  green frequency
    //   b  blue frequency
    // Returns true if and only if drawing node is found and set appropriately.
    bool set_LED(const uint8_t nid, const int r, const int g, const int b);
  private:
    bool set_LED(const Puppet::NodeDB & nodedb, const uint8_t nid, const int r, const int g, const int b);
  public:
    // Sets current "zero" angles to current angles for a given node
    //
    // Inputs:
    //   n  drawing node
    // Returns true on success, false on error
    bool tare(const TBT & t);
    // Returns whether attached
    bool is_attached() const;
    void clear_id_to_last_node();
};

#endif
