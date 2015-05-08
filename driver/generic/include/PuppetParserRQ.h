#ifndef PUPPET_PARSER_RQ_H
#define PUPPET_PARSER_RQ_H

#include <queue>
#include <list>
#include <stdint.h>

#include "PuppetMessage.h"
#include "PuppetTopology.h"

namespace Puppet {


    /** \class PuppetParserRQ
     * This class will be used to receive UART messages (line by line), parse
     * them and update a the topology and a map of measurements.
     * This assumes that the Puppet is using request-based communications.
     * */
    class PuppetParserRQ : public PuppetTopology {
        protected:
            static const unsigned int DEBUG_MEMORY = 100;
            std::queue<std::string> debug;

            void pushDebugLine(const std::string & s) {
                debug.push(s);
                if (debug.size() > DEBUG_MEMORY) {
                    debug.pop();
                }
            }

            bool updating_topology;
            std::vector<std::string > current_measurement_text;
            std::vector<std::string > last_measurement_text;
            std::vector<std::string > last_topology_measurement_text;

            // Helper structure to build the topology
            struct Parent {
                NodeDB::iterator node;
                uint8_t branch;
                Parent(NodeDB::iterator i, uint8_t b) : node(i), branch(b) {}
            };
            // Stack for building the topology.
            std::list<Parent> parent_stack;

        public:
            PuppetParserRQ() {updating_topology = false;}

            // Process one message received from the UART. The parsing is done
            // using the PuppetMessage class. The topology gets updated as the
            // message is processed, if necessary, and measurement messages are
            // inserted in the measurements map. The Measurement maps is
            // cleared each time a message from the master is received
            // (beginning of a measurement cycle).
            bool parseLine(const std::string & line);

            const std::queue<std::string> & getDebugBuffer() const {
                return debug;
            }
            const std::vector<std::string > & get_current_measurement_text() const
            {
              return current_measurement_text;
            }
            const std::vector<std::string > & get_last_measurement_text() const
            {
              return last_measurement_text;
            }
            const std::vector<std::string > & get_last_topology_measurement_text() const
            {
              return last_topology_measurement_text;
            }
    };



};

#endif // PUPPET_PARSER_RQ_H
