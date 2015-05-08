#ifndef SERIAL_LINE_HANDLER_H
#define SERIAL_LINE_HANDLER_H

#include <string>

// Alec: Q: Should this be combined and templated with
//   ../unix/src/SerialLineHandler.h?
// Alec: A: Yes, I think so. And voila.
template <typename SERIAL>
class SerialLineHandler : public SERIAL {
      
  public:
    SerialLineHandler();
    SerialLineHandler(const std::string & dev, 
            void (*fb_func)(void*,const std::string &), void * ctxt );
    SerialLineHandler(unsigned int devId,
            void (*fb_func)(void*,const std::string &), void * ctxt );
    // Alec: Q:Is this the same as the other SerialLineHandler's update()
    // void update();?
    // A: Seems like yes.
    //virtual void process_data();
    virtual void update();

    void setMaxSize(unsigned int s) {max_size = s;}

    void reset() {
        line.clear();
    }
    
  protected:
    void (*fb_func)(void *,const std::string &);
    void * context;
    std::string line;
    unsigned int max_size;

};

#endif // SERIAL_LINE_HANDLER_H

