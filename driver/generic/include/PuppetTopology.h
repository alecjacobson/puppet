#ifndef PUPPET_TOPOLOGY_H
#define PUPPET_TOPOLOGY_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <string>
#include <vector>
#include <map>
#include <list>

#include <math.h>
#include "firmware/include/configuration.h"

namespace Puppet {

    // Basic function to get the corresponding string
    const char * nodeTypeToString(NodeType t);

    /**
     * \class NodeRecord: struct used to store object in the Node Database
     * */
    struct NodeRecord {
        // Node ID
        uint8_t id;
        // Node maturity, i.e. how often its measurement was missing
        // more than one means that the node might have been removed
        uint8_t maturity;
        // Node type
        NodeType type;
        // 92 bit micro-controller address
        std::string unique_id;
        // Nodes spawning from this node. Should have length one, for anything
        // except a splitter
        std::vector<uint8_t> offspring;
        // offspring angles in radian. Should have length one, for anything but a
        // splitter.
        std::vector<std::vector<float> >offspring_angles_rad;
        // Color of this node (I'm using uint32_t just to match what
        // PuppetMessage uses to read data), really this should be an integer
        // between 0 and 255
        uint32_t color[3];

        // Default constructor, required by map
        NodeRecord() {maturity=id=0;color[0]=113;color[1]=129;color[2]=9;}
    };

    /** 
     * \class NodeMeasurement: struct used to store measurement in a
     * measurement map
     * */
    struct NodeMeasurement {
        // Node type. Not strictly needed, but practical
        NodeType type;
        // Vector of data represented as uint32
        std::vector<uint32_t> data;
        // Vector of the angles, after conversion
        std::vector<float> angles_rad;
        // Default constructor, required by map
        NodeMeasurement() {type=(NodeType)0;}
    };

    typedef std::map< uint8_t,NodeRecord,std::less<uint8_t> > NodeDB;
    typedef std::map< uint8_t,NodeMeasurement,std::less<uint8_t> > MeasurementMap;
    typedef void (*MeasurementCallBack)(void *, const MeasurementMap&);
    typedef void (*TopologyChangeCallBack)(void *, const NodeDB &);


    /** \class PuppetTopology
     * This class will be used to represent the Puppet state 
     * */
    class PuppetTopology {
        protected:
            // Node topology and information
            NodeDB nodedb;
            // Measurement tables
            MeasurementMap measurements;
            
            bool init_required;

            // True if we know that the topology has been changed (an ID
            // message received).
            // Alec: But, set to false at start of each new message
            bool topology_changed;

            // True if a complete measurement set has been received
            bool measurement_complete;
            // True if a measurement has been properly begun by receiving a
            // "A" (MESSAGE_CYCLE_START)
            bool measurement_begun;

            // Helper function to print the topology tree starting at iterator
            // it. Recursively call itself on the offspring of *it. If
            // withMeasurements, then output the measurement map next to the
            // topology. 
            void printTopologyRec(NodeDB::const_iterator it,bool withMeasurements,
                    const std::string & indent, FILE * fp = stdout) const;

            // Callbacks
            void * paramMeasurementCallback;
            MeasurementCallBack measurementCB;

            void * paramTopologyCallback;
            TopologyChangeCallBack topologyCB;


            // Helper function for inherited classes
            void signalTopologyChange() {
                if (topologyCB) {
                    (*topologyCB)(paramTopologyCallback,nodedb);
                }
            }

            void signalNewMeasurement() {
                if (measurementCB) {
                    (*measurementCB)(paramMeasurementCallback,measurements);
                }
            }
        public:
            PuppetTopology() { 
              init_required = true;
              topology_changed = measurement_complete = measurement_begun = false;
              paramTopologyCallback = NULL;
              topologyCB = NULL;
              paramMeasurementCallback = NULL;
              measurementCB = NULL;
            }

            void registerTopologyChangeCB(TopologyChangeCallBack cb, void * param) {
                paramTopologyCallback = param;
                topologyCB = cb;
            }
            void removeTopologyChangeCB() {
                paramTopologyCallback = NULL;
                topologyCB = NULL;
            }

            void registerMeasurementCB(MeasurementCallBack cb, void * param) {
                paramMeasurementCallback = param;
                measurementCB = cb;
            }
            void removeMeasurementCB() {
                paramMeasurementCallback = NULL;
                measurementCB = NULL;
            }


            // Helper function to get the topology
            const NodeDB & getTopology() const {return nodedb;}
            // Helper function to get the measurements
            const MeasurementMap & getMeasurements() const {return measurements;}

            // Helper function to check if the topology has changed
            bool topologyHasChanged() const {return topology_changed;}

            // Helper function to check is a complete set of measurements is
            // ready. TODO: Add a callback mechanism.
            bool measurementsAreReady() const {return measurement_complete;}

            bool initIsRequired() const {return init_required;}

            void resetInitFlag() {init_required = false;}

            // Compare the topology and the measurement tree and detect if some
            // nodes have been removed (or did not send a message). If cleanup
            // is true, then the removed node are also deleted. 
            std::list<uint8_t> findRemovedId(bool cleanup=true);

            // Print the topology (and eventually the measurements) on fp
            void printTopology(bool withMeasurements,FILE * fp = stdout) const;
    };

};


#endif // PUPPET_TOPOLOGY_H
