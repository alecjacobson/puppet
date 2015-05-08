#ifndef PUPPET_MESSAGE_H
#define PUPPET_MESSAGE_H


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include "PuppetTopology.h"

namespace Puppet {
    // The type of message that can be received
    typedef enum {
        MESSAGE_DEBUG          = 0, // #...
        MESSAGE_CYCLE_START    = 1, // A
        MESSAGE_CYCLE_END      = 2, // Z
        MESSAGE_IDENTIFICATION = 3, // I,<id>,<type>,<sn1>,<sn2>,<sn3>
        MESSAGE_SPLITTER       = 4, // S,<id>,<branch>
        MESSAGE_ERROR          = 5, // E,<id>
        MESSAGE_MEASUREMENT    = 6, // M,<id>,<value>[,<value>,<value>]
        MESSAGE_TIMESTAMP      = 7, // T,<ts>
        MESSAGE_NUMBER         = 8, // Not a real message, but helps for iterating
    } MessageType;

    // Basic function to get the corresponding string
    const char * messageTypeToString(MessageType t);

    /**
     * \class PuppetMessage: class used to represent messages received from the
     * UART. A lot of parsing and basic checking is already done at this level
     * Identification messages are expected to be
     // Alec: This is out of date:
     * I,<id>,<type>,[<address 0-31><address 32-63><address 63-92>]
     * for physical objects
     * I,<id>,<type>
     * for splitter lines
     // Alec This is the new version
     * I,<id>,<type>,[<address 0-31><address 32-63><address 63-92>],<r>,<g>,<b>
     * for physical objects
     * I,<id>,<type>,<r>,<g>,<b>
     * for splitter lines
     *
     * Measurement messages are expected to be (according to the number of
     * measurement)
     * M,<id>,<data 1>
     * or
     * M,<id>,<data 1>,<data 2>,<data 3>
     * or even
     * M,<id> 
     * for splitter lines and master
     *
     * Splitter messages are expected to be:
     * S,<id>,<branch>,<θ₁>,<θ₂>,<θ₃>
     * where θ₁ is the 1st angle of a TBT joint sticking out of the <branch>
     * exit from the splitter aligned to match the input.
     * 
     * */
    class PuppetMessage {
        protected:
            // The message type
            MessageType type;
            // The node type
            NodeType node;
            // The node id, sometimes recovered from a NodeDB instead of the
            // message.
            uint8_t nodeId;
            // remaining data.
            std::vector<uint32_t> data;

            // Debug string for debug messages, or raw message text
            std::string raw_text;
            std::string text;

            // Unique ID string for identification message
            std::string unique_id;
        public:
            PuppetMessage() {}
            // Helper function to get the node type
            NodeType getNodeType() const {return node;}
            // Helper function to get the message type
            MessageType getMessageType() const {return type;}
            // Helper function to get the node id
            uint8_t getNodeId() const {return nodeId;}
            // Helper function to get the data
            const std::vector<uint32_t> & getData() const {return data;}

            // Return the message text
            const std::string & getText() const {return text;}
            const std::string & get_raw_text() const {return raw_text;}
            
            // Parse an input string, possibly using some data from the node
            // database nodedb
            bool fillFromString(const std::string & input, const NodeDB & nodedb, bool & init_required);

            // Pretty printing, for debug only
            void print(bool pretty,FILE * fp = stdout);

            // Parsing functions
            // Only applicable if m is an ID message, <= NODE_SPLITTER, with
            // data.size() == 3
            NodeRecord createNodeRecord() const;
            NodeMeasurement createNodeMeasurement() const;
          // Static helper method
        public:
          // Convert data measurements into floating point angles
          //
          // Inputs:
          //   data_begin  n angle measurements begin iterator
          //   data_end n angle measurements end iterator
          //   type  type of node the measurement came from (TBT or SPLITTER)
          // Outputs
          //   angles_rad  n angles in floating point with offsets applied 
          static void data_to_angles_rad(
            const std::vector<uint32_t>::const_iterator & data_begin,
            const std::vector<uint32_t>::const_iterator & data_end,
            const NodeType type,
            std::vector<float> & angles_rad);
    };


};

#endif // PUPPET_MESSAGE_H
