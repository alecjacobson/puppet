/**Signature>
* Author      : Cedric Pradalier 
* Universite  : INRIA - GRAVIR - INPG
* Email       : cedric.pradalier@inrialpes.fr
* Contexte    : These MESR 
* Date        : 2001 - 2004
* License     : Libre (???)
<Signature**/
#ifndef Serial_H
#define Serial_H

#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>
#include <string>

// Alec: There seems to be a bug in the original unix implementation. When I
// first plug in the device (on a linux machine!). The original Serial
// implementation only reads junk (nonsense characters). If I first run the screen
// command and then rerun, it seems to be fixed. I guess that screen is setting
// something correctly that this Serial code expects to be set but does not set
// itself.
//
// Whether on purpose or not, the APPLE implementation I added, manages to read
// correct characters without first running screen. Therefore I'm using this
// implmentation even on linux.
#define APPLE_IMPL

class Serial
{
	private :
        std::string device;
		int fd;
		fd_set rfs;
		//structure definie dans /bits/termios.h
		struct termios newtio;
#ifdef APPLE_IMPL
                struct termios original_tio;
#endif

	public :
                // Need default constructor for C++ immutable type
		Serial();
		Serial(const std::string & dev);
		~Serial();
		bool Open(unsigned int speed);
		bool Close();
		size_t Send(const std::string & text) const {
            return Send((const unsigned char*)text.c_str(),text.size());
        }
		size_t Send(const unsigned char * data,size_t size) const;
		size_t Receive(unsigned char * data,size_t size);
		bool WaitData(size_t millisec);
		bool EmptyBuffers();

		bool isOpen() {return fd>=0;}
};	


#endif // Serial_H
