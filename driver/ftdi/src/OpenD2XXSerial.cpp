
#include "OpenD2XXSerial.h"

// Alec: because clock_gettime was not present on mac os x 
#include "current_utc_time.h"

#include <sys/time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

//#include </opt/local/igl/igl_lib/include/igl/Timer.h>


/*************************************************************/


OpenD2XXSerial::OpenD2XXSerial():
  ftdic()
{

  /* Initialize context for subsequent function calls */
  ftdi_init(&ftdic);
  is_open = false;
  shuttingDown = false;
  th_id = 0;
  pthread_mutex_init(&data_mtx,NULL);
  pthread_cond_init(&data_cond,NULL);
}

OpenD2XXSerial::~OpenD2XXSerial()
{
  // Call close explicitly, otherwise cannot copy this class. 
  //if(is_open)
  //{
  //  Close();
  //}
  //ftdi_deinit(&ftdic);
}

void OpenD2XXSerial::reading_loop()
{
  printf("OpenD2XXSerial::reading_loop\n");
  const int BUF_SIZE = 1024;
  unsigned char buffer[BUF_SIZE];
  int has_read = 0 ;
  while (!shuttingDown) 
  {
    has_read = ftdi_read_data(&ftdic,buffer,BUF_SIZE);
    if(has_read < 0)
    {
      fprintf(stderr,"reading_loop ftdi_read_data error(%d)\n",has_read);
    }
    //printf("has_read: %d\n",has_read);
    pthread_mutex_lock(&data_mtx);
    for (int i=0;i<has_read;i++) {
        data.push_back(buffer[i]);
    }
    pthread_mutex_unlock(&data_mtx);
          usleep(5000);
  }
  printf("OpenD2XXSerial::reading_loop finish\n");
}

void OpenD2XXSerial::update() {
    pthread_cond_broadcast(&data_cond);
}

bool OpenD2XXSerial::Open(unsigned int speed)
{
  /* Open FTDI device based on FT232R vendor & product IDs */
  printf("OpenD2XXSerial::Open()\n");
  int ret;
  ret = ftdi_usb_open(&ftdic, 0x0403, 0x6001);
  if(ret < 0) {
    fprintf(stderr,"OpenD2XXSerial ftdi_usb_open Error: %d\n",ret);
    return false;
  }
  ret = ftdi_set_baudrate(&ftdic,115200);
  if(ret < 0) {
    fprintf(stderr,"OpenD2XXSerial ftdi_set_baudrate Error: %d\n",ret);
    return false;
  }
  //ret = ftdi_enable_bitbang(&ftdic,0x00);
  //if (ret<0)
  //{
  //  fprintf(stderr,"OpenD2XXSerial ftdi_enable_bitbang Error: %d\n",ret);
  //  return false;
  //}
  ret = ftdi_read_data_set_chunksize(&ftdic,64);
  if(ret < 0) {
    fprintf(stderr,"OpenD2XXSerial ftdi_read_data_set_chunksize Error: %d\n",ret);
    return false;
  }
  //ret = ftdi_write_data_set_chunksize(&ftdic,64);
  //if(ret < 0) {
  //  fprintf(stderr,"OpenD2XXSerial ftdi_write_data_set_chunksize Error: %d\n",ret);
  //  return false;
  //}
  ret = ftdi_set_latency_timer(&ftdic,1);
  if(ret < 0) {
    fprintf(stderr,"OpenD2XXSerial ftdi_set_latency_timer Error: %d\n",ret);
    return false;
  }

  pthread_create(&th_id,NULL,static_read_thread,this);
  
  is_open = true;
  return true;
}

bool OpenD2XXSerial::Close()
{
  printf("Close()\n");
  if (th_id) {
      shuttingDown = true;
      printf("cancel()\n");
      pthread_cancel(th_id);
      printf("join()\n");
      pthread_join(th_id,NULL);
  }
  th_id = 0;

  //// This final reset seem important
  //printf("ftdi_usb_reset()\n");
  //ftdi_usb_reset(&ftdic);
  //printf("ftdi_usb_close()\n");
  //ftdi_usb_close(&ftdic);
  //printf("ftdi_usb_close completed()\n");
  is_open = false;
  return true;
}

size_t OpenD2XXSerial::Send(const unsigned char * buffer,size_t size)
{
  int ret = -1;
  unsigned char * nc_buffer = new unsigned char[size];
  for(int i = 0;i<(int)size;i++)
  {
    nc_buffer[i] = buffer[i];
  }
  ret = ftdi_write_data(&ftdic,nc_buffer,size);
  // immediately clean up
  delete[] nc_buffer;
  if(ret<0)
  {
    fprintf(stderr,"Error ftdi_write_data(%d)\n",ret);
    return 0;
  }
  return ret;
}
