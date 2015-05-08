#include <assert.h>

#include "PuppetParserRQ.h"
#include "PuppetMessage.h"

// #define DEBUG_PARSER

using namespace Puppet;

static std::string printable(const std::string & str) 
{ 
  std::string out = str;
  for(int i = 0;i<(int)str.length();i++)
  {
    if(
        str[i] == '\n' || 
        str[i] == '\t' || 
        str[i] == '\r' || 
        (str[i] >= 32 && str[i] <=126)
      )
    {
      out[i] = str[i];
    }else
    {
      out[i] = '.';
    }
  }
  return out;
}


bool PuppetParserRQ::parseLine(const std::string & line) {
    PuppetMessage msg;
    char tmp[128];
    std::string trimmed;
    size_t n=0;
    // Find first reasonable character
    while (isspace(line[n]) || (line[n]<0x20) || ((unsigned)line[n] > 0x7F)) {
        n++;
        if (n>=line.size()) break;
    }
    if(n<line.size())
    {
      trimmed = line.substr(n,line.find_last_not_of(" \t\n\r")-n+1);
    }else
    {
      // Should never call substring with n>=line.size()
      trimmed = "";
    }
    if (!msg.fillFromString(trimmed,nodedb,init_required)) {
      // Alec: This happens ALL the time, better to not print it
      //fprintf(stderr,"Received measurement for unknown node: %s\n",input.c_str());
      //  fprintf(stderr,"Failed to parse '%s'-> %d '%s'\n",
      //      printable(line).c_str(),(int)n,
      //      printable(trimmed).c_str());
        return false;
    }

#ifdef DEBUG_PARSER
    // printf("Parsed '%s'-> %d '%s'\n",line.c_str(),n,trimmed.c_str());
    msg.print(true);
#endif
    // If we haven't properly started a new measurement then only let certain
    // measurements through. That is, if we just finished a measurement, ignore
    // everything until we properly begin a measurement again.
    if(measurement_begun || 
      msg.getMessageType() == MESSAGE_CYCLE_START ||
      msg.getMessageType() == MESSAGE_DEBUG ||
      msg.getMessageType() == MESSAGE_ERROR ||
      msg.getMessageType() == MESSAGE_TIMESTAMP)
    {
      measurement_complete = false;
      current_measurement_text.push_back(msg.get_raw_text());
      switch (msg.getMessageType()) {
          case MESSAGE_DEBUG:
              pushDebugLine(msg.getText());
              break;
          case MESSAGE_CYCLE_END:
              measurement_complete = true;
              // Now we've finished reading a measurement and we shouldn't
              // process anything until we begin a new one
              if(measurement_begun)
              {
                last_measurement_text = current_measurement_text;
                signalNewMeasurement();
                if (topology_changed) {
                  last_topology_measurement_text = current_measurement_text;
                    signalTopologyChange();
                }
              }
              measurement_begun = false;
              break;

          case MESSAGE_CYCLE_START:
              // On reception of a measurement start message, 
              // And the measurement map
              measurements.clear();
              // Keep track that we've just begun reading a new measurement
              measurement_begun = true;
              // Alec: What's the point of this?:
              measurement_complete = false;
              topology_changed = false;
              updating_topology = false;
              current_measurement_text.clear();
              current_measurement_text.push_back(msg.get_raw_text());
              break;
          case MESSAGE_IDENTIFICATION:
              {
                  // In request mode, we only get I messages when the topology is
                  // rebuilt
                  topology_changed = true;
                  if (!updating_topology) {
                      nodedb.clear();
                      parent_stack.clear();
                      updating_topology = true;
                  } 
                  std::pair<NodeDB::iterator,bool> ir = 
                      nodedb.insert(NodeDB::value_type(msg.getNodeId(),msg.createNodeRecord()));
                  assert(ir.second);
                  if (parent_stack.empty()) {
                      parent_stack.push_back(Parent(ir.first,0));
                      // printf("Parent: pushing %d\n",ir.first->second.id);
                  } else {
                      // if the nodedb is not empty, then we have at least one
                      // parent
                      Parent parent = parent_stack.back();
                      parent_stack.pop_back();
                      // printf("Parent: pop %d\n",parent.node->second.id);
                      parent.node->second.offspring[parent.branch] = msg.getNodeId();
                      parent_stack.push_back(Parent(ir.first,0));
                      // printf("Parent: pushing %d\n",ir.first->second.id);
                  }

              }
              break;
          case MESSAGE_SPLITTER:
              {
                  if (updating_topology) {
                      NodeDB::iterator it = nodedb.find(msg.getNodeId());
                      if (it == nodedb.end()) {
                        init_required = true;
                          fprintf(stderr,"Received splitter message from unknown node\n");
                          return false;
                      }
                      if ((it->second.type & NODE_SPLITTER_MASK) != NODE_SPLITTER_GENERIC) {
                        init_required = true;
                          fprintf(stderr,"Received splitter message from non-splitter node\n");
                          return false;
                      }
                      size_t branch = msg.getData()[0];

                      Parent parent = parent_stack.back();
                      if (branch > 0) {
                          parent_stack.pop_back();
                      }
                      // printf("Parent: pop %d\n",parent.node->second.id);
                      if (branch != NODE_SPLITTER_COMPLETE) {
                          if (it->second.offspring.size() <= branch) {
                            init_required = true;
                              fprintf(stderr,"Desired splitter branch larger than declared type\n");
                              return false;
                          }

                          // Store splitter angles
                          PuppetMessage::data_to_angles_rad(
                            msg.getData().begin()+1,
                            msg.getData().end(),
                            // msg.getNodeType() cannot be trusted to give the
                            // node type of the splitter!!!! 
                            //msg.getNodeType(),
                            it->second.type,
                            it->second.offspring_angles_rad[branch]);

                          parent.node = it;
                          parent.branch = msg.getData()[0];
                          parent_stack.push_back(parent);
                          // printf("Parent: pushing %d\n",it->second.id);
                      } 
                      

                  } else {
                      // Ignore splitter message if not updating the topology
                      // There should not be any anyway
                  }
              }
              break;
          case MESSAGE_ERROR:
              // Ignored for now
              sprintf(tmp,"Error: %d",msg.getData()[0]);
              pushDebugLine(tmp);
              break;
          case MESSAGE_TIMESTAMP:
              // Ignored for now
              sprintf(tmp,"Timestamp: %d",msg.getData()[0]);
              pushDebugLine(tmp);
              break;
          case MESSAGE_MEASUREMENT:
              {
                  // TODO: filter by node type?
                  NodeDB::iterator it = nodedb.find(msg.getNodeId());
                  assert(it != nodedb.end());
                  measurements[it->first] = msg.createNodeMeasurement();
              }
              break;
          default:
              {
                  // Should never reach this line
                  bool messageTypeIsCorrect = false;
                  assert(messageTypeIsCorrect);
              }
              break;
      }
    }else
    {
      // we've ignored the message
    }

    return true;
}

