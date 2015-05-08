#include <assert.h>

#include "PuppetParserWF.h"

// #define DEBUG_PARSER

using namespace Puppet;


bool PuppetParserWF::parseLine(const std::string & line) {
    PuppetMessage msg;
    if (!msg.fillFromString(line,nodedb,init_required)) {
        return false;
    }

#ifdef DEBUG_PARSER
    msg.print(true);
#endif
    if (msg.getMessageType() == MESSAGE_CYCLE_END) {
        measurement_complete = true;
        if (topology_changed) {
            signalTopologyChange();
        }
        signalNewMeasurement();
        return true;
    }

    if (msg.getMessageType() == MESSAGE_CYCLE_START) {
        // On reception of a measurement start message, we clear the stack
        parent_stack.clear();
        // And the measurement map
        measurements.clear();
        measurement_complete = false;
#ifdef DEBUG_PARSER
        printf("Parent: cleared\n");
#endif
        return true;
    }



    switch (msg.getNodeType()) {
        case NODE_COLLECTOR:
            {
                switch (msg.getMessageType()) {
                    case MESSAGE_IDENTIFICATION:
                        {
                            // This should be the first message received
                            topology_changed = true;
                            NodeRecord active_node = msg.createNodeRecord();
                            std::pair<NodeDB::iterator,bool> ir = 
                                nodedb.insert(NodeDB::value_type(active_node.id,active_node));
                            assert(ir.second);
                            // Prepare the stack of parents
                            parent_stack.push_back(Parent(ir.first,0));
#ifdef DEBUG_PARSER
                            printf("Parent: pushed %d\n",ir.first->first);
#endif
                        } 
                        break;
                    case MESSAGE_MEASUREMENT:
                        { 
                            // If it is a measurement, then this signal the
                            // beginning of a new measurement phase, so we
                            // clear the measurement map
                            topology_changed = false;
                            NodeDB::iterator it = nodedb.find(msg.getNodeId());
                            assert(it != nodedb.end());
                            measurements[it->first] = msg.createNodeMeasurement();
                            // But we still need to prepare the stack of
                            // parents
                            parent_stack.push_back(Parent(it,0));
#ifdef DEBUG_PARSER
                            printf("Parent: pushed %d\n",it->first);
#endif
                        }
                    default:
                        break;
                }
            }
            break;
        case NODE_JOINT_TRT:
        case NODE_JOINT_T:
        case NODE_JOINT_R:
        case NODE_SPLITTER_2:
        case NODE_SPLITTER_3:
        case NODE_SPLITTER_5:
            {
                // For all these nodes, we first start by popping the parent on
                // top of the stack, and eventually replacing it a bit later.
                Parent parent = parent_stack.back();
#ifdef DEBUG_PARSER
                printf("Parent: top is %d\n",parent.node->first);
#endif
                parent_stack.pop_back();
#ifdef DEBUG_PARSER
                printf("Parent: pop top in %s\n",nodeTypeToString(msg.getNodeType()));
#endif
                switch (msg.getMessageType()) {
                    case MESSAGE_IDENTIFICATION:
                        {
#ifdef DEBUG_PARSER
                            printf("Adding node %d:%s in parent %d\n",
                                    msg.getNodeId(),nodeTypeToString(msg.getNodeType()),
                                    parent.branch);
#endif
                            // Connect this node to the right place in the
                            // topology. 
                            parent.node->second.offspring[parent.branch] = msg.getNodeId();
                            NodeRecord active_node = msg.createNodeRecord();
                            std::pair<NodeDB::iterator,bool> ir = 
                                nodedb.insert(NodeDB::value_type(active_node.id,active_node));
                            assert(ir.second);
                            topology_changed = true;
                            // And push back the current node on top of the
                            // stack
                            parent_stack.push_back(Parent(ir.first,0));
#ifdef DEBUG_PARSER
                            printf("Parent: pushed %d\n",ir.first->first);
#endif
                        }
                        break;
                    case MESSAGE_MEASUREMENT:
                        {
                            // Check that the measurement corresponds to a
                            // known node
                            NodeDB::iterator it = nodedb.find(msg.getNodeId());
                            assert(it != nodedb.end());
                            // and then add it to the measurement map
                            measurements[it->first] = msg.createNodeMeasurement();
                            // but still push back current node on the top of
                            // the stack
                            parent_stack.push_back(Parent(it,0));
#ifdef DEBUG_PARSER
                            printf("Parent: pushed %d\n",it->first);
#endif
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
#if 0
#warning Not implemented
        case NODE_SPLITTER_LINE1:
            {
                // This means that we start parsing one line of a splitter
                // node. We start by reading the parent on top of the stack,
                // which should be the splitter itself
                Parent parent = parent_stack.back();
#ifdef DEBUG_PARSER
                printf("Parent: top is %d\n",parent.node->first);
#endif
                // And we push it on top a second time, to be used and replaced
                // by anybody on the tree
                parent_stack.push_back(parent);
#ifdef DEBUG_PARSER
                printf("Parent: %d pushed back with branch %d\n",parent.node->first,parent.branch);
#endif
                switch (msg.getMessageType()) {
                    case MESSAGE_IDENTIFICATION:
                        // nothing more to do
                        break;
                    case MESSAGE_MEASUREMENT:
                        {
                            // If we get a measurement message, just store it.
                            NodeDB::iterator it = nodedb.find(msg.getNodeId());
                            assert(it != nodedb.end());
                            measurements[it->first] = msg.createNodeMeasurement();
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
        case NODE_SPLITTER_LINE2:
        case NODE_SPLITTER_LINE3:
        case NODE_SPLITTER_LINE4:
        case NODE_SPLITTER_LINE5:
        case NODE_SPLITTER_LINE6:
        case NODE_SPLITTER_COMPLETE:
            {
                // This means that we continue parsing one line of a splitter
                // node. We start by removing the parent on top of the stack,
                Parent top = parent_stack.back();
#ifdef DEBUG_PARSER
                printf("Parent: top is %d\n",top.node->first);
#endif
                parent_stack.pop_back();
                // Now, the top of the stack should be the splitter itself
#ifdef DEBUG_PARSER
                printf("Parent: pop top in %s\n",nodeTypeToString(msg.getNodeType()));
#endif
                // So we push it a second time, so it can be replaced by
                // further element on the branch
                Parent parent = parent_stack.back();
                switch (msg.getMessageType()) {
                    case MESSAGE_IDENTIFICATION:
                        break;
                    case MESSAGE_MEASUREMENT:
                        {
                            // No measurement per se, but that confirms that the splitter
                            // is here
                            NodeDB::iterator it = nodedb.find(msg.getNodeId());
                            assert(it != nodedb.end());
                            measurements[it->first] = msg.createNodeMeasurement();
                        }
                        break;
                    default:
                        break;
                }
                if (msg.getNodeType() != NODE_SPLITTER_COMPLETE) {
                    // Make sure that the list of connectivity has the right
                    // length, and update the branch on top of the stack,
                    // but only if the splitter is not complete
                    parent.branch = msg.getNodeType() - NODE_SPLITTER_LINE1;
                    if (parent.branch >= parent.node->second.offspring.size()) {
                        parent.node->second.offspring.resize(parent.branch+1,0);
                    }
                    parent_stack.push_back(parent);
#ifdef DEBUG_PARSER
                    printf("Parent: %d pushed back with branch %d\n",parent.node->first,parent.branch);
#endif
                }
            }
            break;
#endif
        default:
            assert(false);
    }
    return true;

}

