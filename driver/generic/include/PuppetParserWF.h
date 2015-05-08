#ifndef PUPPET_PARSER_WF_H
#define PUPPET_PARSER_WF_H

#include <list>
#include <stdint.h>

#include "PuppetMessage.h"
#include "PuppetTopology.h"

namespace Puppet {


    /** \class PuppetParserWF
     * This class will be used to receive UART messages (line by line), parse
     * them and update a the topology and a map of measurements.
     * This assumes that the Puppet is using the wave-front approach.
     * */
    class PuppetParserWF : public PuppetTopology {
        protected:
            // Helper structure to build the topology
            struct Parent {
                NodeDB::iterator node;
                uint8_t branch;
                Parent(NodeDB::iterator i, uint8_t b) : node(i), branch(b) {}
            };
            // Stack for building the topology.
            std::list<Parent> parent_stack;
        public:
            PuppetParserWF() {}

            // Process one message received from the UART. The parsing is done
            // using the PuppetMessage class. The topology gets updated as the
            // message is processed, if necessary, and measurement messages are
            // inserted in the measurements map. The Measurement maps is
            // cleared each time a message from the master is received
            // (beginning of a measurement cycle).
            bool parseLine(const std::string & line);

            // TODO: Configure a the led on one joint
            // void setLed(uint8_t id, uint8_t red, uint8_t green, uint8_t blue); 
    };



};

#endif // PUPPET_PARSER_WF_H
