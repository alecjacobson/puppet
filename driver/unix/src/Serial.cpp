
/**Signature>
* Author      : Cedric Pradalier 
* Universite  : INRIA - GRAVIR - INPG
* Email       : cedric.pradalier@inrialpes.fr
* Contexte    : These MESR 
* Date        : 2001 - 2004
* License     : Libre (???)
<Signature**/
#include <sys/time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "Serial.h"

#ifdef APPLE_IMPL
#  include <sys/ioctl.h>
#  include <iostream>
#endif


/*************************************************************/

Serial::Serial():
  device(""),
  fd(-1)
{
}

Serial::Serial(const std::string& dev) 
{
    device = dev;
    fd = -1;
}

Serial::~Serial()
{
  // Call Close() directly. Calling it in the destructor makes copying this
  // class impractical.
    //if (fd!=-1)
    //{
    //    Close();
    //}
    //fd = -1;
}

bool Serial::Open(unsigned int speed)
{
    unsigned int realspeed = speed;
#ifdef APPLE_IMPL
    fd = -1;
    printf("Serial::Open APPLE_IMPL\n");


    // Open the serial port read/write, with no controlling terminal, and don't wait for a connection.
    // The O_NONBLOCK flag also causes subsequent I/O on the device to be non-blocking.
    // See open(2) ("man 2 open") for details.

    fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1)
    {
        printf("Error opening serial port %s - %s(%d).\n",
                device.c_str(), strerror(errno), errno);
        goto error;
    }

    // Note that open() follows POSIX semantics: multiple open() calls to the same file will succeed
    // unless the TIOCEXCL ioctl is issued. This will prevent additional opens except by root-owned
    // processes.
    // See tty(4) ("man 4 tty") and ioctl(2) ("man 2 ioctl") for details.

    if (ioctl(fd, TIOCEXCL) == -1)
    {
        printf("Error setting TIOCEXCL on %s - %s(%d).\n",
                device.c_str(), strerror(errno), errno);
        goto error;
    }

    // Now that the device is open, clear the O_NONBLOCK flag so subsequent I/O will block.
    // See fcntl(2) ("man 2 fcntl") for details.

    if (fcntl(fd, F_SETFL, 0) == -1)
    {
        printf("Error clearing O_NONBLOCK %s - %s(%d).\n",
                device.c_str(), strerror(errno), errno);
        goto error;
    }

    // Get the current options and save them so we can restore the default settings later.
    if (tcgetattr(fd, &original_tio) == -1)
    {
        printf("Error getting tty attributes %s - %s(%d).\n",
                device.c_str(), strerror(errno), errno);
        goto error;
    }

    // The serial port attributes such as timeouts and baud rate are set by modifying the termios
    // structure and then calling tcsetattr() to cause the changes to take effect. Note that the
    // changes will not become effective without the tcsetattr() call.
    // See tcsetattr(4) ("man 4 tcsetattr") for details.

    newtio = original_tio;

    // Print the current input and output baud rates.
    // See tcsetattr(4) ("man 4 tcsetattr") for details.

    printf("Current input baud rate is %d\n", (int) cfgetispeed(&newtio));
    printf("Current output baud rate is %d\n", (int) cfgetospeed(&newtio));

    // Set raw input (non-canonical) mode, with reads blocking until either a single character 
    // has been received or a one second timeout expires.
    // See tcsetattr(4) ("man 4 tcsetattr") and termios(4) ("man 4 termios") for details.

    cfmakeraw(&newtio);
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 10;

    cfsetspeed(&newtio, realspeed);    // Set 115200 baud    

    // Print the new input and output baud rates. Note that the IOSSIOSPEED ioctl interacts with the serial driver 
    // directly bypassing the termios struct. This means that the following two calls will not be able to read
    // the current baud rate if the IOSSIOSPEED ioctl was used but will instead return the speed set by the last call
    // to cfsetspeed.

    printf("Input baud rate changed to %d (%d)\n", (int) cfgetispeed(&newtio),realspeed);
    printf("Output baud rate changed to %d\n", (int) cfgetospeed(&newtio));

    // Cause the new options to take effect immediately.
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        printf("Error setting tty attributes %s - %s(%d).\n",
                device.c_str(), strerror(errno), errno);
        goto error;
    }

    // Success
    return true;

    // Failure path
error:
    if (fd != -1)
    {
        close(fd);
    }

    return false;
#else
    switch (speed)
    {
        case B9600: realspeed = 9600; break;
        case B19200: realspeed = 19200; break;		
        case B38400: realspeed = 38400; break;
        case B57600: realspeed = 57600; break;
        case B115200: realspeed = 115200; break;
        case B230400: realspeed = 230400; break;
        default:break;
    }
    int r=0;

    //char command[1024];
    //sprintf(command,"stty -f %s -echo -icanon raw %d ",
    //		device.c_str(),realspeed);
    //r = system(command);
    //if (r != 0) {
    //  printf("[ERROR] System command 'stty' failed (to set baudrate)");
    //  return false;
    //}

    fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); 
    if (fd < 0) {
        fd = -1;
        perror("Serial::Open1");
        return false;
    }
    printf("r: %d, errno: %d\n",r,errno);
    perror("Serial::Open");
    cfmakeraw (&newtio);
    cfsetospeed(&newtio,(speed));
    cfsetispeed(&newtio,(speed));
    newtio.c_iflag = 0;
    newtio.c_iflag = IGNBRK | IGNPAR | INPCK;
    newtio.c_oflag = 0;
    newtio.c_cflag = 0;
    newtio.c_cflag = CS8 | CSTOPB | CREAD | CLOCAL | PARODD;
    cfsetospeed(&newtio,(speed));
    cfsetispeed(&newtio,(speed));
    // Alec: Is this necessary?
    r = tcflush(fd, TCIFLUSH);
    printf("r: %d, errno: %d\n",r,errno);
    r = tcsetattr(fd,TCSANOW,&newtio);
    printf("r: %d, errno: %d\n",r,errno);

    EmptyBuffers();

    return true;
#endif
}

bool Serial::Close()
{

  int ret = -1;
  if (fd>=0)
  {
#ifdef APPLE_IMPL
      // Block until all written output has been sent from the device.
      // Note that this call is simply passed on to the serial device driver. 
      // See tcsendbreak(3) ("man 3 tcsendbreak") for details.
      // We don't really need that...
      if (tcdrain(fd) == -1)
      {
          printf("Error waiting for drain - %s(%d).\n",
                  strerror(errno), errno);
      }

      // Traditionally it is good practice to reset a serial port back to
      // the state in which you found it. This is why the original termios struct
      // was saved.
      if (tcsetattr(fd, TCSANOW, &original_tio) == -1)
      {
          printf("Error resetting tty attributes - %s(%d).\n",
                  strerror(errno), errno);
      }
#endif
    // Alec: This is not reliable
    ret = close(fd);
  }
  fd = -1;
  return ret == 0;
}

size_t Serial::Send(const unsigned char * data,size_t size) const
{
    size_t wes;
    if (fd<0)
        return false;
    wes=write(fd,data,size);
#ifndef APPLE_IMPL
    if(wes!=-1)
    {
      int tcd_ret = tcdrain(fd);
      if(tcd_ret != 0)
      {
        wes= size_t(-3);
      }else
      {
        int tcf_ret = tcflush(fd, TCOFLUSH);
        if(tcf_ret!=0)
        {
          wes = size_t(-2);
        }
      }
    }
#endif
    return wes;
}
    
size_t Serial::Receive(unsigned char * data, size_t size)
{
    size_t w;

    if (fd<0)
        return 0;
    w = read(fd,data,size);
    return w;
}


bool Serial::EmptyBuffers()
{
    unsigned int i=0;
    unsigned char c;
    if (fd<0)
        return false;
    tcdrain(fd);
    // This command causes Alec's mac to hang (or at least kills the keyboard
    // and mouse)
    //tcflush(fd, TCIFLUSH);

    printf("What am I discarding?\n");
    while (WaitData(1)) {
        // discard
        Receive(&c,1);
        printf("%c",c);
        i++;
    }
    printf("\n");
    printf("Discarded %d characters\n",i);

    return true;
}

bool Serial::WaitData(size_t millisec)
{
    struct timeval to = {0,0};
    to.tv_sec = millisec / 1000;
    to.tv_usec = (millisec % 1000) * 1000;
    FD_ZERO(&rfs);FD_SET(fd,&rfs);
    int r = select(fd+1,&rfs,NULL,NULL,&to);
    return (r >= 1);
}
        


