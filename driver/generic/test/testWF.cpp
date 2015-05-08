
#include <stdlib.h>
#include <stdio.h>
#include <string>

#include "PuppetParserWF.h"

using namespace Puppet;

int main(int argc, char * argv[])
{
    if (argc < 2) {
        printf("Usage: %s <input file>\n",argv[0]);
        return 0;
    }

    FILE * fp = fopen(argv[1],"r");
    if (fp == NULL) {
        perror("fopen");
        printf("Can't open source file: '%s'\n",argv[1]);
        return -1;
    }



    char buffer[1024];

    PuppetParserWF pp;

    while (!feof(fp)) {
        // Parsing file line by line
        buffer[0] = 0;
        char * r = fgets(buffer,1024,fp);
        if (!r) break;

        std::string line(buffer);
        if (line[0]=='#') {
            printf("%s",buffer);
            continue;
        }
        pp.parseLine(line);
        if (pp.measurementsAreReady()) {
            // We consider the comment lines as the mark between cycles (we
            // could also use the master's starting message
            printf("Measurements are ready, printing status\n");
            // First see if some nodes are off (in this case they would have
            // not sent a measurement, but still be in the topology)
            // By passing true as the "cleanup" argument nodes that seems
            // absent will be removed from the topology
            std::list<uint8_t> removed = pp.findRemovedId(true);
            if (!removed.empty()) {
                // Print the list of removed node
                printf("------Detected Removal --------------\n");
                for (std::list<uint8_t>::const_iterator it = removed.begin();it!=removed.end();it++) {
                    printf("%d\n",*it);
                }
            }
            printf("-------------------------------------\n");
            // Print the topology on stdout
            pp.printTopology(true,stdout);
            printf("-------------------------------------\n");
        }
    }

    return 0;
}

