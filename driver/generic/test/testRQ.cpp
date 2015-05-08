
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <string>

#include "PuppetParserRQ.h"

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

    PuppetParserRQ pp;

    while (!feof(fp)) {
        // Parsing file line by line
        buffer[0] = 0;
        char * r = fgets(buffer,1024,fp);
        if (!r) break;

        std::string line(buffer);
        assert(pp.parseLine(line));
        if (pp.measurementsAreReady()) {
            // We consider the comment lines as the mark between cycles (we
            // could also use the master's starting message
            printf("Measurements are ready, printing status\n");
            printf("-------------------------------------\n");
            // Print the topology on stdout
            if (pp.topologyHasChanged()) {
                pp.printTopology(false,stdout);
            } else {
                pp.printTopology(true,stdout);
            }
            printf("-------------------------------------\n");
            // getchar();
        }
    }

    return 0;
}

