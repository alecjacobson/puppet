#include <assert.h>

#include "PuppetTopology.h"

using namespace Puppet;

const char * Puppet::nodeTypeToString(NodeType t) {
    switch (t) {
        case NODE_COLLECTOR: 
            return "Collector";
        case NODE_JOINT_TRT:
            return "TRT Joint";
        case NODE_JOINT_T:
            return "T Joint";
        case NODE_JOINT_R:
            return "R Joint";
        case NODE_SPLITTER_2:
            return "2-way Splitter";
        case NODE_SPLITTER_3:
            return "3-way Splitter";
        case NODE_SPLITTER_5:
            return "5-way Splitter";
        default:
            return "Unknown";
    }
}

std::list<uint8_t> PuppetTopology::findRemovedId(bool cleanup) {
    std::list<uint8_t> removed;
    // Now go through all the node in the list and increment the maturity of
    // the one for which no measurement has been found. It is normal the first
    // time they popup, but they will be detected as missing afterwards
    for (NodeDB::iterator it = nodedb.begin(); it!=nodedb.end(); it++) {
        if (measurements.find(it->first) == measurements.end()) {
            it->second.maturity += 1;
            if (it->second.maturity > 1) {
                removed.push_back(it->first);
            }
        }
    }
    if (cleanup) {
        for (std::list<uint8_t>::iterator ir = removed.begin();ir!=removed.end();ir++) {
            NodeDB::iterator it = nodedb.find(*ir);
            nodedb.erase(it);
            // Now remove any reference to this node.
            for (NodeDB::iterator it = nodedb.begin(); it!=nodedb.end(); it++) {
                if (it->second.offspring.empty()) {
                    continue;
                }
                for (unsigned int i=0;i<it->second.offspring.size();i++) {
                    if (it->second.offspring[i] == *ir) {
                        it->second.offspring[i] = 0;
                    }
                }
            }
        }
    }
    return removed;
}

void PuppetTopology::printTopologyRec(NodeDB::const_iterator it, bool withMeasurements, 
        const std::string & ind, FILE * fp) const{
    std::string indent(ind);
    signed int i;
    if (withMeasurements) {
        fprintf(fp,"%s%3d: %s #%02X%02X%02X ",
          indent.c_str(),
          it->second.id,
          nodeTypeToString(it->second.type),
          it->second.color[0],
          it->second.color[1],
          it->second.color[2]);
        MeasurementMap::const_iterator im = measurements.find(it->first);
        if (im == measurements.end()) {
            fprintf(fp," -no data-");
        } else if (im->second.data.size()>0) {
#if 0
            // Raw data
            fprintf(fp,"(");
            for (i=0;i<(signed)im->second.data.size()-1;i++) {
                fprintf(fp,"%04X,",im->second.data[i]);
            }
            fprintf(fp,"%04X)",im->second.data[i]);
#else
            fprintf(fp,"(");
            for (i=0;i<(signed)im->second.data.size()-1;i++) {
                fprintf(fp,"%+6.2f,",im->second.angles_rad[i]*180./M_PI);
            }
            fprintf(fp,"%+6.2f) deg",im->second.angles_rad[i]*180./M_PI);
#endif
        }
    } else {
        fprintf(fp,"%s%3d: %s (%s)",indent.c_str(),
                it->second.id,nodeTypeToString(it->second.type),
                it->second.unique_id.c_str());
    }
    fprintf(fp,"  [");
    if (it->second.offspring.size()) {
        for (i=0;i<(signed)it->second.offspring.size()-1;i++) {
            fprintf(fp,"%d,",it->second.offspring[i]);
        }
        fprintf(fp,"%d",it->second.offspring[i]);
    }
    fprintf(fp,"]");
    fprintf(fp,"\n");
    if (it->second.offspring.size() > 1) {
        indent += "    ";
    }
    for (i=0;i<(signed)it->second.offspring.size();i++) {
        if(it->second.offspring.size()>1)
        { 
          fprintf(fp,"%s  @(%+6.2f, %+6.2f, %+6.2f) deg\n",
            ind.c_str(),
            it->second.offspring_angles_rad[i][0],
            it->second.offspring_angles_rad[i][1],
            it->second.offspring_angles_rad[i][2]);
        }
        // printf("Processing offspring %d: %d\n",i,it->second.offspring[i]);
        if (it->second.offspring[i]) 
        {
            NodeDB::const_iterator jt = nodedb.find(it->second.offspring[i]);
            assert(jt != nodedb.end());
            printTopologyRec(jt,withMeasurements,indent,fp);
        } else if (it->second.offspring.size() > 1) {
            fprintf(fp,"%s      -\n",ind.c_str());
        }
    }
}

void PuppetTopology::printTopology(bool withMeasurements, FILE * fp) const{
    NodeDB::const_iterator it = nodedb.find(1); // Find master
    if (it == nodedb.end()) {
        fprintf(fp,"No Nodes (or no master)\n");
        return;
    }
    assert(it->second.type == NODE_COLLECTOR);
    printTopologyRec(it,withMeasurements,"",fp);
}




