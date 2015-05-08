#include "SerialLineHandler.h"
#include "Serial.h"

template <>
SerialLineHandler<Serial>::SerialLineHandler() :
    Serial(),
    fb_func(NULL), 
    context(NULL), 
    line(""), 
    max_size(256)
{
}

template <>
SerialLineHandler<Serial>::SerialLineHandler(const std::string & dev, 
        void (*handler)(void *, const std::string &),
        void * ctxt) : Serial(dev) ,
    fb_func(handler), context(ctxt), line(""), max_size(256) { }


#include <iostream>
template <>
void SerialLineHandler<Serial>::update(){
  using namespace std;
    
    if (WaitData(50)) {
        unsigned char buffer[128] = "";
        size_t n = Receive(buffer,128);
        for (unsigned int i=0;i<n;i++) {
            if (buffer[i] == '\n') {
                fb_func(context,line);
                line.clear();
            } else {
                line += buffer[i];
            }
        }
        //cout<<"line: \n"<<line<<endl;
        if (line.size() >= max_size) {
            fb_func(context,line);
            line.clear();
        }
    }
}


