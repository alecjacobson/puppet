
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <string>

#include "PuppetParserRQ.h"
#include "SerialLineHandler.h"
#include "Serial.h"

int end = 0;
void sighdl(int n) {
  end ++;
  if (end>2) {
    kill(getpid(),SIGKILL);
  }
}

using namespace Puppet;

class PuppetInterface {
    protected:
        SerialLineHandler<Serial> serial;
        PuppetParserRQ pp;
        int counter;

        bool verbose;
        bool leave_zeros;
        bool got_topology;

        static void handler(void * arg, const std::string & s) {
            PuppetInterface * pi = (PuppetInterface*)arg;
            pi->parseLine(s);
        }

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



        void parseLine(const std::string & s)
        {

          counter++;
            // ROS_INFO("Line %s",s.c_str());
          //if(s[0] != '#')
          //{
            printf("%s\n",printable(s).c_str());fflush(stdout);
          //}
            if (!pp.parseLine(s)) {
                return;
            }
            if (pp.measurementsAreReady()) {
                // Print the topology on stdout (This only makes sense for
                // a serial program, which this is not
                if (!pp.topologyHasChanged()) {
                    pp.printTopology(true,stdout);
                } else {
                    got_topology = true;
                    pp.printTopology(false,stdout);
                    // If the topology has changed, we'll get the value at the next
                    // run. Nothing to do for now.
                }
            }
        }

    public:
        PuppetInterface(const std::string & devname) : serial(devname.c_str(),handler,this),counter(0) {
            verbose=true;
            leave_zeros = true;
            got_topology = false;
            serial.setMaxSize(256);
        }

        bool open(unsigned int speed=115200) {
            return serial.Open(speed);
        }

        //void reset() 
        //{
        //    while (counter==0) {
        //        serial.update();
        //    }
        //    counter = 0;
        //    // serial.Send("b\ni\n");
        //    serial.reset();
        //}

        void mainloop() 
        {
          // Loop until we at least manage to read one line
          while(!end && counter<10)
          {
            serial.update();
          }
          printf("Read 10 lines\n");
          size_t num_written;
          serial.reset();
          num_written = serial.Send("i\n");
          printf("i Number of bytes written: %d\n",(int)num_written);
          //num_written = serial.Send("b\n");
          //printf("b Number of bytes written: %d\n",(int)num_written);
          int loop_count = 0;
            while (!end) 
            {
              //printf("loop_count: %d\n",loop_count);
                serial.update();
                //if (((counter>100) && !got_topology) || pp.initIsRequired()) {
                //  pp.resetInitFlag();
                //  counter = 0;
                //  got_topology = false;
                //  printf("\n\n\nSending init command\n\n\n\n");
                //  serial.EmptyBuffers();
                //  //serial.reset();
                //  size_t num_written = serial.Send("i\n");
                //  printf("Number of bytes written: %d\n",(int)num_written);
                //}
                //if(loop_count%200 == 100)
                //{
                //  serial.reset();
                //  //serial.EmptyBuffers();
                //  num_written = serial.Send("b\n");
                //  printf("b %d Number of bytes written: %d\n",loop_count,(int)num_written);
                //}else if(loop_count%200== 199)
                //{
                //  serial.reset();
                //  //serial.EmptyBuffers();
                //  num_written = serial.Send("i\n");
                //  printf("i %d Number of bytes written: %d\n",loop_count,(int)num_written);
                //}
              loop_count++; 
            }
            close();
        }
        bool close()
        {
          return serial.Close();
        }
};

int main(int argc, char * argv[])
{
    unsigned int baudrate = 115200;
    //std::string devname = "/dev/cu.mcat-DevB";
    std::string devname = "/dev/cu.usbserial-FTF3QCG9"; 
    if (argc > 1) {
        devname = argv[1];
    }
    if (argc > 2) {
        sscanf(argv[2]," %d",&baudrate);
    }
    printf("Using serial port '%s' @ %d\n",devname.c_str(),baudrate);
    signal(SIGINT,sighdl);

    PuppetInterface pi(devname);
    pi.open(baudrate);
    pi.mainloop();

    //for(double ssecs = 0.0; ssecs<60; ssecs++)
    //{
    //  printf("ssecs: %lf\n",ssecs);
    //  pi.open(baudrate);
    //  usleep(1000*1000*ssecs);
    //  pi.close();
    //  printf("\n");
    //}

    return 0;
}

