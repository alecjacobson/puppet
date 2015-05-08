/**Signature>
* Author      : Cedric Pradalier 
* Universite  : INRIA - GRAVIR - INPG
* Email       : cedric.pradalier@inrialpes.fr
* Contexte    : These MESR 
* Date        : 2001 - 2004
* License     : Libre (???)
<Signature**/
#ifndef D2XXSerial_H
#define D2XXSerial_H

#include "GenericD2XXSerial.h"

#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <list>
#include <ftd2xx.h>

class D2XXSerial : public GenericD2XXSerial
{
  protected :
    unsigned int devIndex;
    FT_HANDLE  ftHandle;
    int  iNumDev;
    static const unsigned int MAX_DEVICES=64;
    char cBufLD[MAX_DEVICES][64];
    virtual void reading_loop();
    virtual void update();

  public :
    // Default constructor for C++ immutable type
    D2XXSerial(unsigned int index=0);
    virtual ~D2XXSerial();
    virtual bool Open(unsigned int speed);
    virtual bool Close();
    virtual size_t Send(const unsigned char * data,size_t size);
      virtual size_t Send(const std::string & text) {
        return Send((const unsigned char*)text.c_str(),text.size());
      }
    //virtual size_t Receive(unsigned char * data,size_t size);
    //bool WaitData(size_t millisec);
    //bool EmptyBuffers();

    virtual bool isOpen() {return (bool)ftHandle;}
};  


#endif // Serial_H
