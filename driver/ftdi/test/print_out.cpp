
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <string>

#include "PuppetParserRQ.h"
#include "D2XXSerial.h"
#include "OpenD2XXSerial.h"
#include "SerialLineHandler.h"

using namespace Puppet;

int end = 0;
void sighdl(int n) {
    end ++;
    if (end>2) {
        kill(getpid(),SIGKILL);
    }
}

class PuppetInterface {
    protected:
        SerialLineHandler<D2XXSerial> serial;
        //SerialLineHandler<OpenD2XXSerial> serial;
        PuppetParserRQ pp;

        bool verbose;
        bool leave_zeros;

        static void handler(void * arg, const std::string & s) {
            PuppetInterface * pi = (PuppetInterface*)arg;
            pi->parseLine(s);
        }




        void parseLine(const std::string & s) {

            printf("%s\n",s.c_str());
            // printf(".");fflush(stdout);
            if (!pp.parseLine(s)) {
                return;
            }
            if (pp.measurementsAreReady()) {
                // Print the topology on stdout
                if (!pp.topologyHasChanged()) {
                    pp.printTopology(true,stdout);
                } else {
                    pp.printTopology(false,stdout);
                    // If the topology has changed, we'll get the value at the next
                    // run. Nothing to do for now.
                }
            }
        }

    public:
        PuppetInterface(unsigned int devId) : serial(devId,handler,this) {
            verbose=true;
            leave_zeros = true;
            serial.setMaxSize(256);

    //sleep(1);
    //printf("Sending identify...\n");
    //serial.Send("i\r\n");
    //sleep(1);
    //printf("Sent identify...\n");

        }

        bool open(unsigned int speed=115200)
        {
          printf("open\n");
            bool opened = serial.Open(speed);
            return opened;
        }

        void mainloop() {
          printf("mainloop\n");
          sleep(3);
          // Alec: This is a hack so that the device is actually initialized
            printf("Sending identify again...\n");
            serial.Send(std::string("i\r\n"));
            printf("Sent identify again...\n");

          while (!end) 
          {
            // nothing to do here, serial is creating its own thread
            serial.update();
            sleep(0.01);
          }
          printf("dun' loopin'\n");
          serial.Close();
        }
};


int main(int argc, char * argv[])
{
    unsigned int baudrate = 115200;
    unsigned int devId = 0;

    if (argc > 1) {
        sscanf(" %d",argv[1],&devId);
    }
    if (argc > 2) {
        sscanf(argv[2]," %d",&baudrate);
    }
    int i = 0;
    printf("Using schmerial port '%d' @ %d\n",devId,baudrate);

    printf("a%d\n",i++);
    PuppetInterface pi(devId);
    printf("b%d\n",i++);
    bool opened = pi.open(baudrate);
    assert(opened);
    printf("c%d\n",i++);
    
    signal(SIGINT,sighdl);

    printf("d%d\n",i++);
    pi.mainloop();
    printf("e%d\n",i++);

    return 0;
}

