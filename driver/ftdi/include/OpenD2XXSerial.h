#ifndef OpenD2XXSerial_H
#define OpenD2XXSerial_H
#include "GenericD2XXSerial.h"

#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <list>
#include <ftdi.h>

class OpenD2XXSerial : public GenericD2XXSerial
{
	protected :
        struct ftdi_context ftdic;
        bool is_open;
        virtual void reading_loop();
        virtual void update();

	public :
		OpenD2XXSerial();
		virtual ~OpenD2XXSerial();
		virtual bool Open(unsigned int speed);
		virtual bool Close();
		virtual size_t Send(const unsigned char * data,size_t size);
		virtual size_t Send(const std::string & text) {
                  return Send((const unsigned char*)text.c_str(),text.size());
                }
		//virtual size_t Receive(unsigned char * data,size_t size);
		//bool WaitData(size_t millisec);
		//bool EmptyBuffers();

		virtual bool isOpen() {return (bool)is_open;}
};	


#endif
