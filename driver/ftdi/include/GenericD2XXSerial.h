/**Signature>
* Author      : Cedric Pradalier 
* Universite  : INRIA - GRAVIR - INPG
* Email       : cedric.pradalier@inrialpes.fr
* Contexte    : These MESR 
* Date        : 2001 - 2004
* License     : Libre (???)
<Signature**/
#ifndef GenericD2XXSerial_H
#define GenericD2XXSerial_H

#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <list>
#include <ftd2xx.h>

class GenericD2XXSerial
{
      protected :
        bool shuttingDown;

        static void* static_read_thread(void * arg)
        {
          GenericD2XXSerial *that = (GenericD2XXSerial*)arg;
          that->reading_loop();
          return NULL;
        }
        virtual void reading_loop(){}

        pthread_t th_id;
        pthread_mutex_t data_mtx;
        pthread_cond_t data_cond;

        //std::list<unsigned char> data;
        std::list<unsigned char> data;

        // Alec: See note in incluce/SerialLineHandler.h
        //virtual void process_data();
        virtual void update(){}

	public :
		GenericD2XXSerial(){};
		virtual ~GenericD2XXSerial(){};
		virtual bool Open(unsigned int speed){return false;}
		virtual bool Close(){return false;}
		virtual size_t Send(const unsigned char * data,size_t size){return 0;}
		virtual size_t Send(const std::string & text) {
                  return Send((const unsigned char*)text.c_str(),text.size());
                }
		//virtual size_t Receive(unsigned char * data,size_t size){return 0;};
		virtual bool isOpen(){return false;}
                virtual bool EmptyBuffers()
                {
                  pthread_mutex_lock(&data_mtx);
                  data.clear();
                  pthread_mutex_unlock(&data_mtx);
                  return true;
                }
};	


#endif // Serial_H
