
#include <ctype.h>
#include <assert.h>
#include "PuppetMessage.h"

using namespace Puppet;

struct MessageParsingData {
    char header;
    bool generic_parser;
    size_t num_argument_min;
    size_t num_argument_max;
    const char * format;
}; 

// It is expected that the order of this array match exactly the order of the
// MessageType enum in PuppetMessage.h
static MessageParsingData mpd[] = {
    {'#', false, 0,     0,      "%s"}, // MESSAGE_DEBUG
    {'A', true,  0,     0,      ""}, // MESSAGE_CYCLE_START
    {'Z', true,  0,     0,      ""}, // MESSAGE_CYCLE_END
    {'I', false, 6,     6,      ",%x,%x,%[0123456789abcdef],%x,%x,%x"}, // MESSAGE_IDENTIFICATION
    {'S', true,  2,     5,      ",%x,%x,%d,%d,%d"}, // MESSAGE_SPLITTER
    {'E', true,  1,     1,      ",%x"}, // MESSAGE_ERROR
    {'M', true,  2,     4,      ",%x,%x,%x,%x"}, // MESSAGE_MEASUREMENT
    {'T', true,  1,     1,      ",%x"}, // MESSAGE_TIMESTAMP
};



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

bool PuppetMessage::fillFromString(const std::string & input,const NodeDB & nodedb, bool & init_required) {
    int r,n = 0;
    // The maximum amount of data to read in is 6
    int d[6];
    char mt;
    int temp_mt;
    data.clear();
    r = sscanf(input.c_str()," %c%n",&mt,&n);
    if (r < 1) {
        fprintf(stderr,"Can't parse line, ignoring: %s\n",input.c_str());
        return false;
    }
    mt = toupper(mt);
    raw_text = input;
    text = input.substr(n,std::string::npos);
    switch (mt) {
        case '#':
            type = MESSAGE_DEBUG;
            break;
        // Q: Why is 'I' treated special here?
        // A: I guess because the number of arguments for "physical objects"
        // (TBT joints) and "splitter lines" (splitters) are different
        case 'I':
            type = MESSAGE_IDENTIFICATION;
            // OLD:
            //r = sscanf(text.c_str(),",%x,%x,%n",d+0,d+1,&n);
            //if (r != 2) {
            //    fprintf(stderr,"Incorrect number of argument in: '%s'\n",input.c_str());
            //    return false;
            //}
            //node = (NodeType)d[1];
            //nodeId = d[0];
            //unique_id = text.substr(n,std::string::npos);
            char id[1024];
            r = sscanf(text.c_str(),mpd[MESSAGE_IDENTIFICATION].format,
              d+0,d+1,id,d+2,d+3,d+4);
            node = (NodeType)d[1];
            nodeId = d[0];
            unique_id = std::string(id);
            for(int i = 0;i<3;i++)
            {
              data.push_back(d[2+i]);
            }
            break;
        case 'S':
        case 'M':
        case 'E':
        case 'T':
        case 'A':
        case 'Z':
            for (temp_mt=MESSAGE_DEBUG; temp_mt != MESSAGE_NUMBER; temp_mt++) {
                if (mpd[temp_mt].header == mt) {
                    type = (MessageType)temp_mt;
                    if (mpd[temp_mt].num_argument_max > 0) {
                        r = sscanf(text.c_str(),mpd[type].format,d+0,d+1,d+2,d+3,d+4,d+5);
                        if ((r<(signed)mpd[temp_mt].num_argument_min) || (r>(signed)mpd[temp_mt].num_argument_max)) {
                            fprintf(stderr,"Incorrect number of argument in: '%s'\n",input.c_str());
                            return false;
                        }
                    }
                    break;
                }
            }
            break;
        default:
            fprintf(stderr,"Unrecognised message: '%s'\n",printable(input).c_str());
            return false;
    }

    switch (type) {
        case MESSAGE_IDENTIFICATION:
        case MESSAGE_CYCLE_START:
        case MESSAGE_CYCLE_END:
        case MESSAGE_DEBUG:
            return true;
        case MESSAGE_SPLITTER:
            nodeId = d[0];
            data.resize(4);
            data[0] = d[1];
            // Splitter angles
            data[1] = d[2];
            data[2] = d[3];
            data[3] = d[4];
            return true;
        case MESSAGE_ERROR:
            data.resize(1);
            data[0] = d[0];
            return true;
        case MESSAGE_TIMESTAMP:
            data.resize(1);
            data[0] = d[0];
            return true;
        case MESSAGE_MEASUREMENT:
            {
                nodeId = d[0];
                NodeDB::const_iterator details = nodedb.find(nodeId);
                type = MESSAGE_MEASUREMENT; 
                if (details == nodedb.end()) {
                  init_required = true;
                  // Alec: This happens ALL the time, better to not print it
                    //fprintf(stderr,"Received measurement for unknown node: %s\n",input.c_str());
                    return false;
                }
                node = details->second.type;
                switch (node) {
                    case NODE_COLLECTOR:
                    case NODE_SPLITTER_2:
                    case NODE_SPLITTER_3:
                    case NODE_SPLITTER_5:
                        // ignoring the end of the line, if any
                        break;
                    case NODE_JOINT_TRT:
                        // The data are the angle measurements
                        if (r < 4) {
                          init_required = true;
                            fprintf(stderr,"Insufficient measurement for TBT node: %s\n",input.c_str());
                            return false;
                        }
                        data.resize(3);
                        data[0] = d[1];
                        data[1] = d[2];
                        data[2] = d[3];
                        break;
                    case NODE_JOINT_T:
                    case NODE_JOINT_R:
                        if (r < 2) {
                          init_required = true;
                            fprintf(stderr,"Insufficient measurement for T/B node: %s\n",input.c_str());
                            return false;
                        }
                        // The data is the angle measurement
                        data.resize(1);
                        data[0] = d[1];
                        break;
                    default:
                        init_required = true;
                        fprintf(stderr,"Invalid node type in record: %s\n",input.c_str());
                        return false;
                }
            }
            return true;
        default:
            // Can't reach here
            break;
    }
    return false;
}


void PuppetMessage::print(bool pretty,FILE * fp) {
    switch (type) {
        case MESSAGE_DEBUG:
            if (pretty) {
                fprintf(fp,"Debug: %s",text.c_str());
            } else {
                fprintf(fp,"#%s",text.c_str());
            }
            break;
        case MESSAGE_CYCLE_START:
            if (pretty) {
                fprintf(fp,"Cycle Start\n");
            } else {
                fprintf(fp,"A\n");
            }
            break;
        case MESSAGE_CYCLE_END:
            if (pretty) {
                fprintf(fp,"Cycle End\n");
            } else {
                fprintf(fp,"Z\n");
            }
            break;
        case MESSAGE_SPLITTER:
            if (pretty) {
                fprintf(fp,"Splitter: id %d branch %d\n",nodeId,data[0]);
            } else {
                fprintf(fp,"S,%d,%d\n",nodeId,data[0]);
            }
            break;
        case MESSAGE_ERROR:
            if (pretty) {
                fprintf(fp,"Error: code %x\n",data[0]);
            } else {
                fprintf(fp,"E,%x\n",data[0]);
            }
            break;
        case MESSAGE_TIMESTAMP:
            if (pretty) {
                fprintf(fp,"Timestamp: counter %d\n",data[0]);
            } else {
                fprintf(fp,"T,%x\n",data[0]);
            }
            break;
        case MESSAGE_IDENTIFICATION:
            if (pretty) {
                fprintf(fp,"Identification: id %d type %s address '%s'\n",
                        nodeId,nodeTypeToString(node),unique_id.c_str());
            } else {
                fprintf(fp,"I,%d,%02X,%s\n",nodeId,node,unique_id.c_str());
            }
            break;
        case MESSAGE_MEASUREMENT:
            switch (node) {
                case NODE_COLLECTOR:
                case NODE_SPLITTER_2:
                case NODE_SPLITTER_3:
                case NODE_SPLITTER_5:
                    if (pretty) {
                        fprintf(fp,"Measurement: id %d type %s\n",nodeId,nodeTypeToString(node));
                    } else {
                        fprintf(fp,"M,%d\n",nodeId);
                    }
                    break;
                case NODE_JOINT_TRT:
                    if (pretty) {
                        fprintf(fp,"Measurement: id %d type %s value %04x, %04x, %04x\n",
                                nodeId,nodeTypeToString(node),data[0],data[1],data[2]);
                    } else {
                        fprintf(fp,"M,%d,%04x,%04x,%04x\n",
                                nodeId,data[0],data[1],data[2]);
                    }
                    break;
                case NODE_JOINT_T:
                case NODE_JOINT_R:
                    if (pretty) {
                        fprintf(fp,"Measurement: id %d type %s value %04x\n",
                                nodeId,nodeTypeToString(node),data[0]);
                    } else {
                        fprintf(fp,"M,%d,%04x\n",
                                nodeId,data[0]);
                    }
                    break;
                default:
                    // Ignore
                    break;
            } 
        default:
            break;
    }
}

    

const char * Puppet::messageTypeToString(MessageType t) {
    switch (t) {
        case MESSAGE_IDENTIFICATION:
            return "Identification";
        case MESSAGE_MEASUREMENT:
            return "Measurement";
        case MESSAGE_CYCLE_START:
            return "Measurement Start";
        case MESSAGE_CYCLE_END:
            return "Measurement End";
        case MESSAGE_SPLITTER:
            return "Splitter";
        case MESSAGE_ERROR:
            return "Error";
        case MESSAGE_TIMESTAMP:
            return "Timestamp";
        default:
            return "Unknown";
    }
}

NodeRecord PuppetMessage::createNodeRecord() const {
    NodeRecord nr;
    assert(type == MESSAGE_IDENTIFICATION);
    // This is no longer true because MESSAGE_IDENTIFICATIONs now contain
    // colors
    //assert(data.size() == 0);
    // Data should contain color information
    assert(data.size() == 3);
    nr.id = nodeId;
    nr.type = node;
    nr.unique_id = unique_id;
    // Copy colors from data
    nr.color[0] = data[0];
    nr.color[1] = data[1];
    nr.color[2] = data[2];
    const std::vector<float> DEFAULT_OFFSPRING_ANGLES_RAD(3,0);
    switch (node) {
        case NODE_SPLITTER_2:
            nr.offspring.resize(2);
            nr.offspring_angles_rad.resize(2,DEFAULT_OFFSPRING_ANGLES_RAD);
            break;
        case NODE_SPLITTER_3:
            nr.offspring.resize(3);
            nr.offspring_angles_rad.resize(3,DEFAULT_OFFSPRING_ANGLES_RAD);
            break;
        case NODE_SPLITTER_5:
            nr.offspring.resize(5);
            nr.offspring_angles_rad.resize(5,DEFAULT_OFFSPRING_ANGLES_RAD);
            break;
        default:
            nr.offspring.resize(1);
            nr.offspring_angles_rad.resize(1,DEFAULT_OFFSPRING_ANGLES_RAD);
            break;
    }
    for (unsigned int i=0;i<nr.offspring.size();i++) {
        nr.offspring[i] = 0;
    }
    nr.maturity = 0;
    return nr;
}

// Splitter angles are output as integers in degrees
// TBT angles are as fractions of 2*PI times 0x1000
void PuppetMessage::data_to_angles_rad(
  const std::vector<uint32_t>::const_iterator & data_begin,
  const std::vector<uint32_t>::const_iterator & data_end,
  const NodeType type,
  std::vector<float> & angles_rad)
{
  angles_rad.resize(data_end-data_begin,0.0);
  for(
    std::vector<uint32_t>::const_iterator it = data_begin;
    it<data_end;
    it++)
  {
    float f;
    switch(type)
    {
      case NODE_SPLITTER_2:
      case NODE_SPLITTER_3:
      case NODE_SPLITTER_5:
        f = (float(*it) * M_PI/180.0);
        break;
      case NODE_JOINT_TRT:
      case NODE_JOINT_T:
      case NODE_JOINT_R:
      default:
        f = NODE_ANGLE_OFFSET_RAD + float(*it) * NODE_ANGLE_CONVERSION_FACTOR_RAD;
        break;
    }
    angles_rad[it-data_begin] = f;
  }
}

NodeMeasurement PuppetMessage::createNodeMeasurement() const {
    NodeMeasurement nm;
    assert(type == MESSAGE_MEASUREMENT);
    nm.type = node;
    switch (node) {
        case NODE_JOINT_TRT:
        case NODE_JOINT_T:
        case NODE_JOINT_R:
            nm.data = data;
            break;
        default:
            nm.data.clear();
    }
    PuppetMessage::data_to_angles_rad(
      nm.data.begin(),
      nm.data.end(),
      node,
      nm.angles_rad);

    return nm;
}

