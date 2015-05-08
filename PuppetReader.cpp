#include "PuppetReader.h"

// Functions
#include <igl/REDRUM.h>
#include <igl/STR.h>
#include <igl/EPS.h>
#include <igl/C_STR.h>

// Classes
#include "Root.h"
#include "TBT.h"
#include "Splitter.h"
#include "EndCap.h"

// Constants
#include <igl/PI.h>
// Functions
#include <igl/get_seconds.h>
#include <igl/Timer.h>

#include <Eigen/Geometry>

#include <string>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <chrono>


#define LOG_PATH "./PuppetReaderLog.txt"
const std::string PuppetReader::HAND_ID_1 = "35006b063146383325160143";

// SerialLineHandler uses plain old C function pointers so we need to bounce to
// get to a member function
static void read_line(void * arg, const std::string &s)
{
  using namespace std;
  PuppetReader * pr = (PuppetReader*)arg;
  pr->read_line(s);
}

static void new_measurement(void * arg, const Puppet::MeasurementMap&)
{
  PuppetReader * pr = (PuppetReader*)arg;
  pr->new_measurement();
}

static void topology_changed(void * arg, const Puppet::NodeDB&)
{
  PuppetReader * pr = (PuppetReader*)arg;
  pr->topology_changed();
}

static void splitter_angles_changed(void * arg, const Splitter & s)
{
  PuppetReader * pr = (PuppetReader*)arg;
  pr->set_splitter_angles(s);
}

static void node_is_selected_changed(void * arg, const Node & n)
{
  PuppetReader * pr = (PuppetReader*)arg;
  pr->set_is_selected(n);
}

static void tare(void * arg, const TBT & tbt)
{
  PuppetReader * pr = (PuppetReader*)arg;
  pr->tare(tbt);
}

PuppetReader::PuppetReader(
  const std::string devname, 
  const unsigned int speed):
  loop_count(0),
  lines_read(0),
  total_lines_read(0),
  next_command(""),
  print_lines(false),
  log_lines(false),
  // Set up default values and pass on devname to initialize serial
#ifdef __APPLE__
  serial(0,&::read_line,this),
#else
  serial(devname.c_str(),&::read_line,this),
#endif
  pp(),
  buf_nodedb(),
  buf_measurements(),
  topology_changed_since_last_sync(false),
  measurements_ready_since_last_sync(false),
  loop_thread(),
  looping(false),
  //done_looping(false),
  pp_mutex(),
  got_topology(false),
  root(NULL),
  // open a log file
  log_file(fopen(LOG_PATH,"w")),
  id_to_live_node(),
  id_to_last_node(),
  attached(false),
  is_partying(false),
  is_no_LEDs(false),
  wrong_topology_measurement(),
  correct_topology_measurement()
{
  using namespace std;
#ifdef __APPLE__
  if(devname != "" && devname != "ignored")
  {
    cout<<MAGENTAGIN("Warning: ignoring devname ("<<devname<<")")<<endl;
  }
#endif
  // Depending on the local set up serial.Open(...) will fail because
  // permissions on the device are not set correctly. To test issue something
  // like:
  //   stty -echo -F /dev/ttyUSB0
  // If you get:
  //   stty: /dev/ttyUSB0: Permission denied
  // Then try:
  //   sudo stty -echo -F /dev/ttyUSB0
  // If this produces no error then you simply need to add rw privileges to the
  // device file:
  //   sudo chmod o+rw /dev/ttyUSB0
  // Now you should see no error issuing again:
  //   stty -echo -F /dev/ttyUSB0
  // 
  // https://groups.google.com/forum/?fromgroups#!topic/fhem-users/2nUjXv16vlc
  //
  // Another option is to map the device to a new name:
  //   http://aeturnalus.com/robotics/mapping-ftdi-to-files-with-udev/
  // Then just append MODE="666" and the permissions should be permanently
  // changed
  //
  // On ubunutu, also you need to restart udev for the rules to take affect:
  //   sudo /etc/init.d/udev restart
  //
  // Sometimes (usually) when you first plug in the device, the Serial.cpp will
  // only read junk. It seems the following fixes it:
  //   screen /dev/ttyUSB0 115200
  //   <CTRL>+a [backslash]
  //   ./puppet
  // This seems to be a problem only with the original Serial implementation
  //
  // Depending on what's been used for the ATTRS{serial} number, the device may
  // not seem to close properly. Try to issue:
  //
  // sudo lsof /dev/mcat
  //
  // If you see something like:
  //
  // COMMAND    PID USER   FD   TYPE DEVICE SIZE/OFF    NODE NAME
  // devkit-po 2580 root    9u   CHR  188,0      0t0 3260938 /dev/ttyUSB0
  //
  // Then probably your device is being recognized incorrectly as some other
  // device. To temporarily fix this you can comment out the line in
  // /lib/udev/rules.d/95-devkit-power-wup.rules
  // (Notice that the line probably has an ATTRS{serial} value that starts with
  // the same prefix as your device's.
  //
  // Sometimes it happens that the device connects (using D2XX or
  // USB-to-serial), but the green light stays solid (rather than blinking) and
  // no information is read from the device. Commands seem to be ignored.  
  //
  // This was fixed by connecting *only* the collector to the computer and
  // sending the "t\r\n", which (Oliver knows, and Alec is hesitant to agree
  // with but trusts Oliver) sets the type to collector (tantamount to sending
  // "t,00\r\n"). Then we sent the "b\r\n" to unbreak it.
  //
  // SOLID RED (Recognized as joint "Good")
  // SOLID/BLINKING GREEN ("Bad", maybe send "t\r\n" like above)
  attached = serial.Open(speed);
  if(!attached)
  {
    cout<<
      REDRUM("PuppetReader: serial.Open() on '"<<devname<<"' failed.")<<endl;
    assert(root == NULL);
    return;
  }

  if(!log_file)
  {
    cout<<REDRUM("PuppetReader: unable to open log at "<<LOG_PATH)<<endl;
  }


  // Register new measure callback
  pp.registerMeasurementCB(::new_measurement,this);
  pp.registerTopologyChangeCB(::topology_changed,this);

  // Set max line size
  serial.setMaxSize(256);

  // Begin loop in new thread
  loop_thread = std::thread(std::mem_fun(&PuppetReader::loop),this);

  // Not neccessary since loop will call this on its own when bad/unknown
  // measurements are received
  //// Tell device to identify itself
  //send("i\r\n");
}

PuppetReader::PuppetReader(const PuppetReader & that):
  loop_count(that.loop_count),
  lines_read(that.lines_read),
  total_lines_read(that.total_lines_read),
  next_command(""),
  print_lines(that.print_lines),
  log_lines(that.log_lines),
  // Set up phony serial
#ifdef __APPLE__
  serial(that.serial),
#else
  serial(that.serial),
#endif
  pp(that.pp),
  buf_nodedb(),
  buf_measurements(),
  topology_changed_since_last_sync(false),
  measurements_ready_since_last_sync(false),
  loop_thread(),
  looping(),
  pp_mutex(),
  got_topology(that.got_topology),
  root(that.root), 
  log_file(that.log_file),
  id_to_live_node(),
  id_to_last_node(),
  attached(that.attached),
  wrong_topology_measurement(),
  correct_topology_measurement()
{
  assert(false);
  throw std::runtime_error("PuppetReader(const PuppetReader &) not allowed");
  // call assignment operator (designed to fail)
  root = that.root;
}

PuppetReader & PuppetReader::operator=(PuppetReader /*that*/)
{
  assert(false);
  throw std::runtime_error("PuppetReader::operator=(PuppetReader that) not allowed");
  return *this;
}

PuppetReader::~PuppetReader()
{
  using namespace std;
  cout<<BLUEGIN("~PuppetReader")<<endl;
  clear_id_to_last_node();
}

void PuppetReader::close()
{
  using namespace std;
  // Stop if looping and try to join thread
  looping = false;
  //// Hack to try to let thread die on its own
  //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  if(loop_thread.joinable())
  {
    loop_thread.join();
  }

  // Close device
  bool success = serial.Close();
  if(success)
  {
    cout<<GREENGIN("serial.Close() succeeded")<<endl;
  }else
  {
    cout<<REDRUM("serial.Close() failed")<<endl;
  }
  if(log_file)
  {
    if(fclose(log_file)==EOF)
    {
      cout<<REDRUM("fclose(log_file) failed")<<endl;
    }
  }
  delete root;
}

void PuppetReader::read_line(const std::string &line)
{
  using namespace std;
  using namespace igl;

  //if(lines_read%100 == 0)
  //{
  //  printf("read_line: %d %s\n",lines_read,line.c_str());
  //}

  if(print_lines)
  {
    cout<<MAGENTAGIN(line)<<endl;
  }
  if(log_lines&=(log_file!=NULL))
  {
    static double t_start = get_seconds();
    fprintf(log_file,C_STR("# "<<(get_seconds()-t_start)<<" "));
    string posix(line);
    replace(posix.begin(),posix.end(),'\r','\n');
    fprintf(log_file,posix.c_str());
  }


  // Try to parse line using PuppetParserRQ
  if (!pp.parseLine(line))
  {
    //cout<<REDRUM("PuppetReader: failed to parse line:"<<endl<<line<<endl)<<endl;
    goto count_and_return;
  }

  // Remember that topology has changed since last sync
  // This is not trustworthy since pp.measurementsAreReady() just returns true
  // if the last message was "Z"
  //if(pp.measurementsAreReady())
  //{
  //if(measurements_ready_since_last_sync)
  //{
  //  measurements_ready_since_last_sync = false;
  //  //cout<<BLUEGIN("readline: measurements_ready_since_last_sync")<<endl;
  //  if(pp.topologyHasChanged())
  //  {
  //    cout<<BLUEGIN("readline: pp.topologyHasChanged()")<<endl;
  //    topology_changed_since_last_sync = true;
  //    got_topology = true;
  //  }else
  //  {
  //  }
  //}

count_and_return:
  lines_read++;
  total_lines_read++;
    return;
}

static double wait_time = 0.01;
void PuppetReader::loop()
{
  using namespace std;
  using namespace igl;
  assert(!looping);
  looping = true;
  double secs = igl::get_seconds();

#ifdef USE_D2XX
  //// Wait 3 seconds
  //sleep(3);
#else
  // Read at least 100 lines
  while(lines_read<100 && looping)
  {
    serial.update();
  }
#endif
  lines_read = 0;
  Timer timer;
  timer.start();
  //int fps_count = 0;
  while(looping)
  {
    {
      std::unique_lock<std::mutex> l(pp_mutex);
      serial.update();
      // Now serial.line is updated

      // If topology has not been read correctly, then messages will be
      // misinterpreted. We need to re-idenitfy in this case.
      //
      // This is such a hack. Can't we do better?
      if (((lines_read>100) && !got_topology) || pp.initIsRequired())
      {
        //cout<<BLUEGIN("pp.initIsRequired()")<<endl;
        // Only send init at most once per second
        if((igl::get_seconds()-secs)>wait_time)
        {
          pp.resetInitFlag();
          lines_read = 0;
          got_topology = false;
          cout<<YELLOWRUM("Sending \"i\\r\\n\"")<<endl;
          serial.Send("i\r\n");
          secs = igl::get_seconds();
          wait_time*=2;
          //printf("new wait time: %g\n",wait_time);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    timer.stop();
    //printf("loop: %g microseconds\n",timer.getElapsedTimeInMicroSec());
    // TODO: sched_yield
    loop_count++;
    //fps_count++;
    //if(fps_count % 1000)
    //{
    //  timer.stop();
    //  printf("loop: %g frames per second\n",double(fps_count)/timer.getElapsedTimeInSec());
    //  timer.start();
    //  fps_count = 0;
    //}
  }
  cout<<"done looping."<<endl;
}

void PuppetReader::new_measurement()
{
  using namespace std;
  //cout<<BLUEGIN("new_measurement()")<<endl;
  buf_measurements = pp.getMeasurements();
  measurements_ready_since_last_sync = true;
}

void PuppetReader::topology_changed()
{
  using namespace std;
  //cout<<BLUEGIN("topology_changed()")<<endl;
  buf_nodedb = pp.getTopology();
  got_topology = true;
  topology_changed_since_last_sync = true;
}

// Return const reference to PuppetParserRQ
const Puppet::PuppetParserRQ& PuppetReader::get_pp()
{
  std::unique_lock<std::mutex> l(pp_mutex);
  return pp;
}

const Node * PuppetReader::get_const_root() const
{
  return root;
}

Node * PuppetReader::get_root() 
{
  return root;
}

Node * PuppetReader::set_root(Node * r)
{
  //// Do you really want to replace a non-null root? Are you going to clean up
  //// after yourself?
  //assert(root == NULL);
  Node * old_root = root;
  root = r;
  return old_root;
}

static std::string printable(const std::string s)
{
  using namespace std;
  string c = "";
  for(int i = 0;i<(int)s.length();i++)
  {
    switch(s[i])
    {
      case '\n':
        c += "\\n";
        break;
      case '\r':
        c += "\\r";
        break;
      default:
        c += s[i];
    }
  }
  return c;
}

void PuppetReader::send(const std::string s)
{
  using namespace std;
  cout<<YELLOWRUM("Sending \""<<printable(s)<<"\"")<<endl;
  std::unique_lock<std::mutex> l(pp_mutex);
  serial.Send(s);
}

void PuppetReader::sync()
{
  using namespace std;
  if(topology_changed_since_last_sync)
  {
    cout<<BLUEGIN("sync: topology_changed_since_last_sync "<<lines_read)<<endl;
    pp.printTopology(true,stdout);
    sync_topology();
    topology_changed_since_last_sync = false;
  }else if(measurements_ready_since_last_sync)
  {
    //cout<<BLUEGIN("sync: measurements_ready_since_last_sync "<<lines_read)<<endl;
    sync_measurements();
    measurements_ready_since_last_sync = false;
  }else
  {
    //cout<<REDGIN("Dropped frame")<<endl;
  }
}

void PuppetReader::sync_topology()
{
  using namespace std;
  using namespace Puppet;
  //const NodeDB & nodedb = pp.getTopology();
  // Double buffer (really copy the buffer)
  pp_mutex.lock();
  const NodeDB nodedb = buf_nodedb;
  pp_mutex.unlock();

  //bool unchanged = verify_topology_robust_to_splitter_spaghetti(nodedb,1,root);
  bool unchanged = verify_topology(nodedb,1,root);
  if(unchanged)
  {
    cout<<BLUEGIN("sync_topology(): no noticed change")<<endl;
    // Rebuild anyway because colors etc might be wrong if we don't (because
    // the firmware bug might have previous identified wrong order)
    cout<<BLUEGIN("sync_topology(): but rebuilding anyway")<<endl;
    //return;
  }
  
  // Update id to last node map. This may be the last time we see some of these
  // nodes for a while :-(
  for(
    map<string,Node*>::const_iterator iit = id_to_live_node.begin();
    iit != id_to_live_node.end();
    iit++)
  {
    // Remove any old information attached to this id, we'll replace it.
    if(id_to_last_node.count(iit->first))
    {
      delete id_to_last_node[iit->first];
      id_to_last_node.erase(iit->first);
    }
    Node * n = iit->second;
    Node * copy = NULL;
    if(TBT* tbt = dynamic_cast<TBT*>(n))
    {
      copy = new TBT(*tbt);
    }else
    {
      copy = new Node(*n);
    }
    id_to_last_node.insert(make_pair(iit->first,copy));
  }
  // Clear live map (will be rebuilt by rebuild_topology())
  id_to_live_node.clear();

  NodeDB::const_iterator nit = nodedb.find(1);
  assert(nit != nodedb.end());
  // Find root collector 
  if(nit == nodedb.end())
  {
    cout<<REDRUM("rebuild_topology(): No root collector")<<endl;
    return;
  }

  root = rebuild_topology(nodedb,1,root);
}

void PuppetReader::sync_measurements()
{
  using namespace std;
  using namespace Puppet;
  std::unique_lock<std::mutex> l(pp_mutex);
  //const NodeDB & nodedb = pp.getTopology();
  const NodeDB & nodedb = buf_nodedb;
  if(verify_topology(nodedb,1,root))
  {
    // TODO: Should I be verifying pp.getMeasurements() against something?
    // Both root and 1 exist
    //const MeasurementMap measurements = pp.getMeasurements();
    const MeasurementMap & measurements = buf_measurements;
    update_measurements(nodedb,measurements,1,root);
  }else{
    cout<<REDRUM("sync_measurements(): topology not valid")<<endl;
    if(looping && got_topology)
    {
      cout<<REDGIN("sync_measurements(): "
        "sending identify ")<<endl;
      //send("i\r\n");
      // Q: THIS HANGS?! Why can't I send here? 
      // A: Because I'm in the scope lock?
      string s = "i\r\n";
      cout<<YELLOWRUM("Sending \""<<printable(s)<<"\"")<<endl;
      serial.Send(s);
    }
  }
}

Node * PuppetReader::update_measurements(
  const Puppet::NodeDB & nodedb,
  const Puppet::MeasurementMap & measurements,
  const uint8_t nid,
  Node * d)
{
  using namespace std;
  using namespace Puppet;
  NodeDB::const_iterator nit = nodedb.find(nid);
  // Find root collector 
  if(nit == nodedb.end() && d != NULL)
  {
    cout<<REDRUM("update_measurements(): "
      "!nodedb["<<(int)nid<<"], node!=NULL")<<endl;
    return d;
  }
  if(nit != nodedb.end() && d == NULL)
  {
    cout<<REDRUM("update_measurements(): node==null, nodedb["<<(int)nid<<"]")<<endl;
    return d;
  }
  if(nit == nodedb.end() && d == NULL)
  {
    // then we both agree there's nobody home
    return d;
  }

  const NodeRecord& n = nit->second;
  MeasurementMap::const_iterator im = measurements.find(nit->first);
  if((n.type & NODE_SPLITTER_MASK) == NODE_SPLITTER_GENERIC)
  {
    // skipping
    goto recurse;
  }
  if (im == measurements.end())
  {
    //cout<<REDGIN("No data: "<<int(n.id))<<endl;
    // no data
    goto recurse;
  }

  update_measurement(im->second,d);

recurse:
  assert(d!=NULL);
  // Recurse on children
  assert((int)n.offspring.size() == d->get_max_children());
  for (int i=0;i<(int)n.offspring.size();i++)
  {
    uint8_t cid = n.offspring[i];
    // Only recurse on real children
    if(cid != 0)
    {
      // This is not being properly stored in parent (d)
      update_measurements(nodedb,measurements,cid,d->get_child(i));
    }
  }

  return d;
}

void PuppetReader::update_measurement(
  const Puppet::NodeMeasurement & m, 
  Node * d)
{
  using namespace igl;
  using namespace std;
  using namespace Puppet;
  assert(d);
  switch(m.type)
  {
    case NODE_JOINT_TRT:
    {
      TBT* tbt = dynamic_cast<TBT*>(d);
      if(tbt)
      {
        assert(m.angles_rad.size() == 3);
        assert(m.angles_rad.size() == m.data.size());
        for(int i = 0;i<(int)m.angles_rad.size();i++)
        {
          // Exactly 0 means no DATA
          if(m.data[i] == 0)
          {
            //cout<<REDGIN("0 data: (Node) "<<d->get_name())<<endl;
          }else
          {
            tbt->angle[i] = m.angles_rad[i]/PI*180.0;
          }
        }
      }else
      {
        assert(false);
        cout<<REDRUM("update_measurements(): type "<<
          nodeTypeToString(m.type)<<" but node not TBT")<<endl;
      }
      break;
    }
    default:
      cout<<REDRUM("update_measurements(): Unknown type: "<<
        nodeTypeToString(m.type))<<endl;
  }
}

// Return type of node 
static Puppet::NodeType type(const Node * n)
{
  using namespace Puppet;
  if(dynamic_cast<const Splitter*>(n))
  {
    switch(n->get_max_children())
    {
      case 2:
        return NODE_SPLITTER_2;
      case 3:
        return NODE_SPLITTER_3;
      case 5:
        return NODE_SPLITTER_5;
      default:
      {
        // Should handle spillers less generically
        assert(false);
        return NODE_SPLITTER_GENERIC;
      }
    }
  }else if(dynamic_cast<const TBT*>(n))
  {
    return NODE_JOINT_TRT;
  }else if(dynamic_cast<const Root*>(n))
  {
    return NODE_COLLECTOR;
  }else
  {
    //return NODE_JOINT_R;
    //return NODE_JOINT_T;
    return NODE_UNDEFINED;
  }
}

bool PuppetReader::verify_topology_robust_to_splitter_spaghetti(
  const Puppet::NodeDB & nodedb, 
  const uint8_t nid,
  Node * d)
{
  using namespace Puppet;
  using namespace igl;
  using namespace std;
  NodeRecord n;
  if(!find_if_local_match(nodedb,nid,d,n))
  {
    // local_match is taking care of output
    return d == NULL;
  }
  // type checking should have already determined that the number of children
  // will match
  assert((int)n.offspring.size() == d->get_max_children());
  int perm = 0;
  vector<int> order(n.offspring.size());
  // Insert integers in sorted order
  for(int k = 0;k<(int)order.size();k++)
  {
    order[k] = k;
  }
  // for posterity
  // sort(order.begin(),order.end());
  bool verified = false;
  string first_error;
  do
  {
    // This order verified?
    verified = true;
    // Recurse on children
    for (int i=0;i<(int)n.offspring.size();i++)
    {
      uint8_t cid = n.offspring[i];
      Node * c = d->get_child(order[i]);
      // Only recurse on real children
      if(cid == 0)
      {
        // check that corresponding child is either null or endcap
        if(c != NULL && dynamic_cast<const EndCap *>(c) == NULL)
        {
          if(perm == 0)
          {
            first_error = STR("verify_topology_rtss: "<<int(nid)<<" 0 child "<<i<<", non leaf");
            verified = false;
          }
        }
        // Check that children in this branch match resursively
      }else
      {
        //cout<<"  "<<i<<": "<<order[i]<<" . "<<(c!=NULL)<<endl;
        if(!verify_topology_robust_to_splitter_spaghetti(nodedb,cid,c))
        {
          verified = false;
        }
      }
      // At least one non-match for this ordering
      if(!verified)
      {
        break;
      }
    }
    if(verified)
    {
      if(const Splitter * s = dynamic_cast<const Splitter *>(d))
      {
        for (int i=0;i<(int)n.offspring.size();i++)
        {
          // only care if there's a child there
          if(s->get_child(order[i]))
          {
            Eigen::Quaterniond a = TBT::TBT_angles_to_quat(
              s->angles[order[i]][0],
              s->angles[order[i]][1],
              s->angles[order[i]][2]);
            Eigen::Quaterniond b = TBT::TBT_angles_to_quat(
              double(n.offspring_angles_rad[i][0]*180.0/PI),
              double(n.offspring_angles_rad[i][1]*180.0/PI),
              double(n.offspring_angles_rad[i][2]*180.0/PI));
            // http://3dgep.com/?p=1815#Quaternion_Dot_Product
            // THIS COULD BE WRONG: Maybe use frobenious norm of rotation
            // matrices instead
            const double th = 2.*acos(a.dot(b))*180./PI;
            if(th > 15.0)
            {
              cerr<<REDGIN("Rotation "<<i<<" on device TBT("<<
                double(n.offspring_angles_rad[i][0]*180.0/PI)<<" "<<
                double(n.offspring_angles_rad[i][1]*180.0/PI)<<" "<<
                double(n.offspring_angles_rad[i][2]*180.0/PI)<<") "<<
                " !~= splitter's "<<order[i]<<" TBT("<<
                s->angles[order[i]][0]<<" "<<
                s->angles[order[i]][1]<<" "<<
                s->angles[order[i]][2]<<"), "
                "angular difference ("<<th<<")")<<endl;
              verified = false;
              break;
            }
          }
          if(!verified)
          {
            break;
          }
        }
      }
    }
    if(verified)
    {
      break;
    }
    perm++;
  }while(next_permutation(order.begin(),order.end()));
  //cout<<"verified: "<<verified<<" --> "<<perm<<endl;

  if(verified)
  {
    // reorder
    // First pick up nodes and replace with NULL
    vector<Node *> original(d->get_max_children(),NULL);
    for(int i =0;i<d->get_max_children();i++)
    {
      original[i] = d->set_child(NULL,order[i]);
    }
    // Now replace with new order
    for(int i =0;i<d->get_max_children();i++)
    {
      d->set_child(original[i],i);
    }
    assert(verify_topology(nodedb,nid,d));
    return true;
  }else
  {
    cout<< REDGIN(first_error)<<endl;
    return false;
  }
}

bool PuppetReader::verify_topology(
  const Puppet::NodeDB & nodedb, 
  const uint8_t nid,
  const Node * d)
{
  using namespace Puppet;
  using namespace std;
  NodeRecord n;
  if(!find_if_local_match(nodedb,nid,d,n))
  {
    // local_match is taking care of output
    return d==NULL;
  }
  // type checking should have already determined that the number of children
  // will match
  assert((int)n.offspring.size() == d->get_max_children());
  // Recurse on children
  for (int i=0;i<(int)n.offspring.size();i++)
  {
    uint8_t cid = n.offspring[i];
    const Node * c = d->get_child(i);
    // Only recurse on real children
    if(cid == 0)
    {
      // check that corresponding child is either null or endcap
      if(c != NULL && dynamic_cast<const EndCap *>(c) == NULL)
      {
        cout<<
          REDGIN("verify_topology: "<<int(nid)<<" 0 child "<<i<<", non leaf")<<endl;
        return false;
      }
    }else if(!verify_topology(nodedb,cid,c))
    {
      // Check that children in this branch match resursively
      return false;
    }
  }
  return true;
}

bool PuppetReader::find_if_local_match(
  const Puppet::NodeDB & nodedb, 
  const uint8_t nid,
  const Node * d,
  Puppet::NodeRecord & n) const
{
  using namespace std;
  using namespace Puppet;
  NodeDB::const_iterator nit = nodedb.find(nid);
  // No (more) nodes in nodedb, but there is a drawing node
  if(nit == nodedb.end() && d != NULL)
  {
    cout<<REDGIN("verify_topology: "<<int(nid)<<" empty nodedb, d!=NULL")<<endl;
    return false;
  }
  // There are (more) nodes in nodedb, but there is not a drawing node
  if(nit != nodedb.end() && d == NULL)
  {
    cout<<REDGIN("verify_topology: "<<int(nid)<<" nonempty nodedb, d==NULL")<<endl;
    return false;
  }
  if(nit == nodedb.end())
  {
    assert(d == NULL);
    // Also return false, since this is a find after all
    return false;
  }
  // Retrieve node record
  n = nit->second;
  if(!local_match(n,d))
  {
    cout<<REDGIN("verify_topology: "<<int(nid)<<" local mismatch")<<endl;
    return false;
  }
  return true;
}

bool PuppetReader::local_match(const Puppet::NodeRecord & n, const Node * d) 
  const
{
  using namespace Puppet;
  // Can't match record against null node: mismatch
  if(d == NULL)
  {
    return false;
  }
  // Check that type of nit matches type of drawing node
  if(n.type != type(d))
  {
    return false;
  }
  const Splitter * s = dynamic_cast<const Splitter *>(d);
  if( n.type == NODE_SPLITTER_5 && n.unique_id == HAND_ID_1)
  {
    if(s==NULL)
    {
      return false;
    }
    return s->get_is_hand();
  }
  return true;
}

static Node * new_node(const Puppet::NodeType type)
{
  using namespace std;
  using namespace Puppet;
  cout<<GREENRUM("Adding new "<<nodeTypeToString(type))<<endl;
  switch(type)
  {
    // Defined in the-puppet/firmware/include/configuration.h
    case NODE_COLLECTOR:
      return new Root();
    // TBTs need to be told about announce_tare
    case NODE_JOINT_TRT:
      return new TBT();
    // Splitters need to be told about announce_angles_change
    case NODE_SPLITTER_2:
      return new Splitter(2);
    case NODE_SPLITTER_3:
      return new Splitter(3);
    case NODE_SPLITTER_5:
      return new Splitter(5);
    default:
      cout<<REDRUM("new_node(): Unknown type: "<<nodeTypeToString(type))<<endl;
      return NULL;
  }
}

static bool approx(const double a, const double b)
{
  return fabs(a-b)<igl::FLOAT_EPS;
}

Node * PuppetReader::rebuild_topology(
  const Puppet::NodeDB & nodedb,
  const uint8_t nid,
  Node * d)
{
  using namespace Puppet;
  using namespace std;
  using namespace igl;

  NodeDB::const_iterator nit = nodedb.find(nid);
  assert(nit != nodedb.end());
  // Find node in db
  if(nit == nodedb.end())
  {
    cout<<REDRUM("rebuild_topology(): No node with id "<<(int)nid)<<endl;
    return d;
  }

  const NodeRecord& n = nit->second;
  // *always* update. This might be the last time we see a node for a while :-(

  if(!local_match(n,d))
  {
    // Q: Can we assume that set_child will handle clean up? What about root?
    if(d != NULL && d->get_parent() == NULL)
    {
      delete d;
    }
    d = new_node(n.type);
    if(n.type == NODE_SPLITTER_5)
    {
      Splitter * s = dynamic_cast<Splitter *>(d);
      assert(s);
      s->set_is_hand(n.unique_id == HAND_ID_1);
      cout<<"I got a ham!"<<endl;
    }
    if(id_to_last_node.count(n.unique_id)==1)
    {
      Node * last = (id_to_last_node.find(n.unique_id)->second);
      // shallow copy
      if(TBT* d_tbt = dynamic_cast<TBT*>(d))
      {
        TBT * last_tbt = dynamic_cast<TBT*>(last);
        assert(last_tbt);
        *d_tbt = *last_tbt;
      }else
      {
        *d = *last;
      }
    }
    if(d == NULL)
    {
      cout<<REDRUM(
        "rebuild_topology: failed to create "<<
        nodeTypeToString(n.type))<<endl;
      return d;
    }
  }

  // (re)register every node (we just cleared id_to_live_node)
  assert(d != NULL);
  id_to_live_node[n.unique_id] = d;

  // Only really need to do this if creating new node for the first time
  // Register announce is_selected for nodes
  d->param_announce_is_selected_change = this;
  d->announce_is_selected_change=node_is_selected_changed;

  // Register announce set angles for splitters
  Splitter* splitter  = dynamic_cast<Splitter*>(d);
  if(splitter)
  {
    splitter->param_announce_angles_change = this;
    splitter->announce_angles_change = splitter_angles_changed;
  }

  // Register announce set angles for splitters
  TBT* tbt = dynamic_cast<TBT*>(d);
  if(tbt)
  {
    tbt->param_announce_tare = this;
    tbt->announce_tare = ::tare;
    tbt->LED_periods[0] = 1;
    tbt->LED_periods[1] = 0;
    tbt->LED_periods[2] = 0;
  }

  // (re)Initialize color
  for(int i = 0;i<3;i++)
  {
    d->color(i) = double(n.color[i])/255.0;
  }
  // Special translation for blue color which is not really (0,0,1)
  const double BLUE[3] = {0,0,1};
  const double OTHER_BLUE[3] = {0,0,double(0xee)/255.0};
  const double TRUE_BLUE[3] = {10./255.,80./255.,255./255.};
  if(
      equal(BLUE,BLUE+3,d->color.data(),approx) || 
      equal(OTHER_BLUE,OTHER_BLUE+3,d->color.data(),approx) )
  {
    copy(TRUE_BLUE,TRUE_BLUE+3,d->color.data());
  }
  const double RED[3] = {1,0,0};
  const double TRUE_RED[3] = {255./255.,50./255.,0./255.};
  if(equal(RED,RED+3,d->color.data(),approx))
  {
    copy(TRUE_RED,TRUE_RED+3,d->color.data());
  }

  // Recurse on children
  assert((int)n.offspring.size() == d->get_max_children());

  for (int i=0;i<(int)n.offspring.size();i++)
  {
    uint8_t cid = n.offspring[i];
    Node * child = NULL;
    // Only recurse on real children
    if(cid != 0)
    {
      // Pass current child so that it can be verified or deleted
      child = rebuild_topology(nodedb,cid,d->get_child(i));
    }
    // This is a really round about way of making sure that info stored in the
    // current EndCap child is maintained
    if(child == NULL && tbt)
    {
      child = new EndCap();
      if(dynamic_cast<const EndCap *>(d->get_child(i)))
      {
        *child = *d->get_child(i);
      }
    }
    // Set child (even if it's null)
    Node * old_child = d->set_child(child,i);
    // Be sure to remove any old child (often this just NULL, but if there was
    // a local match then it may contain an invalid tree). But be sure that if
    // the child was locally matched then we don't delete it when set_child
    // replaces it
    if(old_child != child)
    {
      delete old_child;
    }

    // Initialize splitter angles
    if(splitter)
    {
      for(int j = 0;j<3;j++)
      {
        //cout<<BLUEGIN(int(cid)<<": n.offspring_angles_rad["<<i<<"]: "<<
        //    n.offspring_angles_rad[i][j]<<" "<<
        //    n.offspring_angles_rad[i][j]<<" "<<
        //    n.offspring_angles_rad[i][j]<<" ")<<endl;
        splitter->angles[i][j] = double(n.offspring_angles_rad[i][j]*180.0/PI);
      }
    }
  }
  return d;
}

bool PuppetReader::find(const Node * q, uint8_t & qid)
{
  using namespace std;
  using namespace Puppet;
  //const NodeDB & nodedb = pp.getTopology();
  // Double buffer (really copy the buffer)
  pp_mutex.lock();
  const NodeDB nodedb = buf_nodedb;
  pp_mutex.unlock();
  if(verify_topology(nodedb,1,root))
  {
    return find(nodedb,1,root,q,qid);
  }else{
    cout<<REDRUM("find(): topology not valid")<<endl;
  }
  return false;
}

bool PuppetReader::find(
  const Puppet::NodeDB & nodedb, 
  const uint8_t nid, 
  const Node * d, 
  const Node * q, 
  uint8_t & qid)
{

  using namespace std;
  using namespace Puppet;
  NodeDB::const_iterator nit = nodedb.find(nid);
  // Find root collector 
  if(nit == nodedb.end() && d != NULL)
  {
    cout<<REDRUM("find(): !nodedb["<<(int)nid<<"], node!=NULL")<<endl;
    return false;
  }
  if(nit != nodedb.end() && d == NULL)
  {
    cout<<REDRUM("find(): node==null, nodedb["<<(int)nid<<"]")<<endl;
    return false;
  }
  if(nit == nodedb.end() && d == NULL)
  {
    // then we both agree there's nobody home
    return false;
  }
  // Now we now we have a well formed nodedb

  // early base case
  if(q == d)
  {
    qid = nid;
    return true;
  }
  const NodeRecord& n = nit->second;

  assert(d!=NULL);
  // Recurse on children
  assert((int)n.offspring.size() == d->get_max_children());
  for (int i=0;i<(int)n.offspring.size();i++)
  {
    uint8_t cid = n.offspring[i];
    // Only recurse on real children
    if(cid != 0)
    {
      if(find(nodedb,cid,d->get_child(i),q,qid))
      {
        return true;
      }
    }
  }
  // Not found here or in children
  return false;
}

void PuppetReader::send_set_splitter_angle(const uint8_t nid, const Splitter & s, const int c)
{
  send(STR("g,"<<(int)nid<<","<<(int)c<<
    ","<<int(s.get_angles()[c][0]+359.0)%360+1<<
    ","<<int(s.get_angles()[c][1]+359.0)%360+1<<
    ","<<int(s.get_angles()[c][2]+359.0)%360+1<<"\r\n"));
}

bool PuppetReader::set_splitter_angles(const Splitter & s)
{
  using namespace std;
  using namespace igl;
  using namespace Puppet;
  uint8_t nid;
  if(find(&s,nid))
  {
    // Verify
#ifndef NDEBUG
    {
      std::unique_lock<std::mutex> l(pp_mutex);
      const NodeDB & nodedb = buf_nodedb;
      NodeDB::const_iterator nit = nodedb.find(nid);
      assert(nit != nodedb.end());
      const NodeRecord& n = nit->second;
      assert((int)n.offspring.size() == s.get_max_children());
    }
#endif
    for(int i = 0;i<s.get_max_children();i++)
    {
      send_set_splitter_angle(nid,s,i);
      if((i+1)<s.get_max_children())
      {
        cout<<BLUEGIN("Waiting "<<1000<<"ms before sending next command")<<endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }
    return true;
  }
  return false;
}

bool PuppetReader::set_splitter_angle(const Splitter & s,const int c)
{
  using namespace std;
  using namespace igl;
  using namespace Puppet;
  assert(c>=0 && "Child index should be positive.");
  assert(c<s.get_max_children() && "Child index should be less than max.");
  uint8_t nid;
  if(find(&s,nid))
  {
    // Verify
#ifndef NDEBUG
    {
      std::unique_lock<std::mutex> l(pp_mutex);
      const NodeDB & nodedb = buf_nodedb;
      NodeDB::const_iterator nit = nodedb.find(nid);
      assert(nit != nodedb.end());
      const NodeRecord& n = nit->second;
      assert((int)n.offspring.size() == s.get_max_children());
    }
#endif
    send_set_splitter_angle(nid,s,c);
    return true;
  }
  return false;
}

bool PuppetReader::set_is_selected(const Node & n)
{
  using namespace std;
  using namespace igl;
  using namespace Puppet;
  uint8_t nid;
  if(find(&n,nid))
  {
    // Q: What should be verified here?
    
    if(n.get_is_selected())
    {
      set_LED(nid,0,0,200);
    }else
    {
      set_LED(nid,1,0,0);
    }
    //cout<<BLUEGIN("Waiting "<<1000<<"ms before sending next command")<<endl;
    //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    return true;
  }
  return false;
}

static Eigen::Vector3i default_LEDs(const uint8_t)
{
  return Eigen::Vector3i(1,0,0);
}

void PuppetReader::toggle_is_partying()
{
  using namespace Puppet;
  using namespace std;
  using namespace Eigen;
  pp_mutex.lock();
  const NodeDB nodedb = buf_nodedb;
  pp_mutex.unlock();
  is_partying=!is_partying;
  if(is_partying)
  {
    const auto & party = [](const uint8_t nid)->Vector3i
    {
      return Vector3i(63 +(nid*7 )%17, 87 +(nid*11)%17, 100+(nid*13)%17);
    };
    set_LEDs(nodedb,1,party);
  }else
  {
    set_LEDs(nodedb,1,default_LEDs);
  }
}

void PuppetReader::toggle_is_no_LEDs()
{
  using namespace Puppet;
  using namespace std;
  using namespace Eigen;
  pp_mutex.lock();
  const NodeDB nodedb = buf_nodedb;
  pp_mutex.unlock();
  is_no_LEDs=!is_no_LEDs;
  if(is_no_LEDs)
  {
    const auto & no_LEDs = [](const uint8_t)->Vector3i
    {
      return Vector3i(0,0,0);
    };
    set_LEDs(nodedb,1,no_LEDs);
  }else
  {
    set_LEDs(nodedb,1,default_LEDs);
  }
}

void PuppetReader::set_LEDs(
  const Puppet::NodeDB & nodedb, 
  const uint8_t nid,
  Eigen::Vector3i (*LED_rates_from_nid)(const uint8_t))
{
  using namespace std;
  using namespace Puppet;
  using namespace Eigen;
  NodeDB::const_iterator nit = nodedb.find(nid);
  // Find root collector 
  if(nit == nodedb.end())
  {
    return;
  }
  Vector3i f = LED_rates_from_nid(nid);
  set_LED(nodedb,nid,f(0),f(1),f(2));
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  const NodeRecord& n = nit->second;
  // Recurse on children
  for (int i=0;i<(int)n.offspring.size();i++)
  {
    uint8_t cid = n.offspring[i];
    // Only recurse on real children
    if(cid != 0)
    {
      set_LEDs(nodedb,cid,LED_rates_from_nid);
    }
  }
}

bool PuppetReader::set_LED( 
  const uint8_t nid, 
  const int r, 
  const int g, 
  const int b)
{
  using namespace Puppet;
  using namespace std;
  pp_mutex.lock();
  const NodeDB nodedb = buf_nodedb;
  pp_mutex.unlock();
  return set_LED(nodedb,nid,r,g,b);
}

bool PuppetReader::set_LED(
  const Puppet::NodeDB & nodedb,
  const uint8_t nid,
  const int r,
  const int g,
  const int b)
{
  using namespace Puppet;
  send(STR("l,"<<(int)nid<<
    ","<<r<<
    ","<<g<<
    ","<<b<<"\r\n"));

  // Find node in db
  NodeDB::const_iterator nit = nodedb.find(nid);
  assert(nit != nodedb.end());
  if(nit == nodedb.end())
  {
    return false;
  }

  const NodeRecord& n = nit->second;
  if(id_to_live_node.count(n.unique_id)!=1)
  {
    return false;
  }
  Node * d = id_to_live_node[n.unique_id];
  TBT* tbt = dynamic_cast<TBT*>(d);
  if(tbt == NULL)
  {
    return false;
  }
  tbt->LED_periods[0] = r;
  tbt->LED_periods[1] = g;
  tbt->LED_periods[2] = b;
  return true;
}

bool PuppetReader::tare(const TBT & tbt)
{
  using namespace std;
  using namespace igl;
  using namespace Puppet;
  uint8_t nid;
  if(find(&tbt,nid))
  {
    send(STR("o,"<<(int)nid<<"\r\n"));
    return true;
  }
  return false;
}

bool PuppetReader::is_attached() const
{
  return attached;
}

void PuppetReader::clear_id_to_last_node()
{
  for(auto & kv : id_to_last_node)
  {
    delete kv.second;
  }
  id_to_last_node.clear();
}
