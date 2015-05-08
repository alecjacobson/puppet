#include "SerialLineHandler.h"
#include "D2XXSerial.h"
#include "OpenD2XXSerial.h"
#include "GenericD2XXSerial.h"
//#include </opt/local/igl/igl_lib/include/igl/Timer.h>
template <>
SerialLineHandler<D2XXSerial>::SerialLineHandler(): 
  D2XXSerial(),
  fb_func(NULL), 
  context(NULL), 
  line(""), 
  max_size(128) 
{
}

template <>
SerialLineHandler<D2XXSerial>::SerialLineHandler(unsigned int devId, 
        void (*handler)(void *, const std::string &),
        void * ctxt) : D2XXSerial(devId) ,
    fb_func(handler), context(ctxt), line(""), max_size(128) { }

template <>
SerialLineHandler<OpenD2XXSerial>::SerialLineHandler(unsigned int /*devId*/,
        void (*handler)(void *, const std::string &),
        void * ctxt) : OpenD2XXSerial() ,
    fb_func(handler), context(ctxt), line(""), max_size(128) { }


template <>
void SerialLineHandler<OpenD2XXSerial>::update()
#include "SerialLineHandlerGenericD2XXSerialupdate.impl"

template <>
void SerialLineHandler<D2XXSerial>::update()
#include "SerialLineHandlerGenericD2XXSerialupdate.impl"

template class SerialLineHandler<OpenD2XXSerial>;
template class SerialLineHandler<D2XXSerial>;
