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
#include <math.h>

//#include </opt/local/igl/igl_lib/include/igl/Timer.h>

#include "D2XXSerial.h"

// Alec: because clock_gettime was not present on mac os x 
#include "current_utc_time.h"

/*************************************************************/


D2XXSerial::D2XXSerial(unsigned int index) : devIndex(index) 
{
    shuttingDown = false;
    th_id = 0;
    ftHandle = 0;
    pthread_mutex_init(&data_mtx,NULL);
    pthread_cond_init(&data_cond,NULL);
    iNumDev = 0;
    	
    FT_STATUS	ftStatus;
	char * 	pcBufLD[MAX_DEVICES + 1];
	for(unsigned int i = 0; i < MAX_DEVICES; i++) {
		pcBufLD[i] = cBufLD[i];
	}
	pcBufLD[MAX_DEVICES] = NULL;
    ftStatus = FT_ListDevices(pcBufLD, &iNumDev, FT_LIST_ALL | FT_OPEN_BY_SERIAL_NUMBER);
    if (ftStatus != FT_OK) {
        printf("Error FT_ListDevices(%d)\n", (int)ftStatus);
    }
}

D2XXSerial::~D2XXSerial()
{
  // Call Close() explicitly, otherwise cannont copy this class
    //if (ftHandle) {
    //    Close();
    //}
}

//void *D2XXSerial::static_read_thread(void *arg) {
//    D2XXSerial *that = (D2XXSerial*)arg;
//    that->reading_loop();
//    return NULL;
//}

void D2XXSerial::reading_loop() {
  printf("reading_loop\n");

        const int BUF_SIZE = 256;
        unsigned char buffer[BUF_SIZE];
        DWORD dwRxSize = 0 ;

        //igl::Timer timer;
    int count = 0;
    while (!shuttingDown) 
    {
      //timer.start();
      int i = 0;
        FT_STATUS	ftStatus;

        DWORD dwAmountInRxQueue, dwAmountInTxQueue=123, dwEventStatus;
        ftStatus = FT_GetStatus(ftHandle, &dwAmountInRxQueue, &dwAmountInTxQueue, &dwEventStatus);
        if (ftStatus != FT_OK) 
        {
            printf("Error FT_GetStatus(%d)\n", (int)ftStatus);	
            // TODO: shutdown? reopen?
        }
        //printf("dwAmountInRxQueue: %d\n",dwAmountInRxQueue);

        if(dwAmountInRxQueue)
        {
          //printf("dwAmountInRxQueue: %d\n",dwAmountInRxQueue);
          //ftStatus = FT_Read(ftHandle, buffer, BUF_SIZE, &dwRxSize);
          ftStatus = FT_Read(ftHandle, buffer, (dwAmountInRxQueue<BUF_SIZE?dwAmountInRxQueue:BUF_SIZE), &dwRxSize);
          if (ftStatus != FT_OK) 
          {
              printf("Error FT_Read(%d)\n", (int)ftStatus);	
              // TODO: shutdown? reopen?
          }

          pthread_mutex_lock(&data_mtx);
          for (DWORD i=0;i<dwRxSize;i++) {
              data.push_back(buffer[i]);
          }
          //// Alec: Q: is this O(n*n)?!
          //while (data.size() > 10000) {
          //  printf("long list\n");
          //    data.pop_front();
          //}
          pthread_mutex_unlock(&data_mtx);
        }

        //if (data.size()) {
        //    this->update();
        //}
          //usleep(5000);
       //if(count % 1000)
       //{
       //   timer.stop();
       //  printf("rl: %g frames per second\n",double(count)/timer.getElapsedTimeInSec());
       //   timer.start();
       //  count = 0;
       //}
       //count++;
    }
}

//void D2XXSerial::process_data() {
void D2XXSerial::update() {
    pthread_cond_broadcast(&data_cond);
}

struct FT_STATUS_String
{
  FT_STATUS status;
  const char * str;
} stati[] = {
  {FT_OK,"FT_OK"},
  {FT_INVALID_HANDLE,"FT_INVALID_HANDLE"},
  {FT_DEVICE_NOT_FOUND,"FT_DEVICE_NOT_FOUND"},
  {FT_DEVICE_NOT_OPENED,"FT_DEVICE_NOT_OPENED"},
  {FT_IO_ERROR,"FT_IO_ERROR"},
  {FT_INSUFFICIENT_RESOURCES,"FT_INSUFFICIENT_RESOURCES"},
  {FT_INVALID_PARAMETER,"FT_INVALID_PARAMETER"},
  {FT_INVALID_BAUD_RATE,"FT_INVALID_BAUD_RATE"},
  {FT_DEVICE_NOT_OPENED_FOR_ERASE,"FT_DEVICE_NOT_OPENED_FOR_ERASE"},
  {FT_DEVICE_NOT_OPENED_FOR_WRITE,"FT_DEVICE_NOT_OPENED_FOR_WRITE"},
  {FT_FAILED_TO_WRITE_DEVICE,"FT_FAILED_TO_WRITE_DEVICE"},
  {FT_EEPROM_READ_FAILED,"FT_EEPROM_READ_FAILED"},
  {FT_EEPROM_WRITE_FAILED,"FT_EEPROM_WRITE_FAILED"},
  {FT_EEPROM_ERASE_FAILED,"FT_EEPROM_ERASE_FAILED"},
  {FT_EEPROM_NOT_PRESENT,"FT_EEPROM_NOT_PRESENT"},
  {FT_EEPROM_NOT_PROGRAMMED,"FT_EEPROM_NOT_PROGRAMMED"},
  {FT_INVALID_ARGS,"FT_INVALID_ARGS"},
  {FT_NOT_SUPPORTED,"FT_NOT_SUPPORTED"},
  {FT_OTHER_ERROR,"FT_OTHER_ERROR"}
  //{FT_DEVICE_LIST_NOT_READY,"FT_DEVICE_LIST_NOT_READY"}
};
#define NUM_FT_STATUS 20

const char * get_status_as_string(const FT_STATUS f)
{
  for(int i = 0;i<NUM_FT_STATUS;i++)
  {
    if(f==stati[i].status)
    {
      return stati[i].str;
    }
  }
  return "Unknown Status";
}

bool D2XXSerial::Open(unsigned int speed)
{
    FT_STATUS	ftStatus;
    if (th_id) {
        printf("Error: D2XXSerial::Open: receiving thread is already running\n");
        return false;
    }

    if ((ftStatus = FT_Open(devIndex, &ftHandle)) != FT_OK) {
        printf("Error FT_Open(%d)==%s, cBufLD[i] = %s\n", (int)ftStatus, get_status_as_string(ftStatus),cBufLD[devIndex]);
#ifdef __APPLE__
      if(ftStatus == -1)
      {
        printf("Consider checking whether linking against correct "
          "libftd2xx.*.*.*.dylib (http://www.alecjacobson.com/weblog/?p=2934)");
      }
#endif
        return false;
    }

    if((ftStatus = FT_SetBaudRate(ftHandle, speed)) != FT_OK) {
        printf("Error FT_SetBaudRate(%d), cBufLD[i] = %s\n", (int)ftStatus, cBufLD[devIndex]);
        return false;
    }
    FT_SetTimeouts(ftHandle, 100, 100);	// 0.1 second read timeout

    //FT_SetDeadmanTimeout(ftHandle,1);
    //if (ftStatus == FT_OK) 
    //{
    //}

    if((ftStatus = FT_SetUSBParameters(ftHandle, 64,64)) != FT_OK) {
        printf("Error FT_SetUSBParameters(%d), cBufLD[i] = %s\n", (int)ftStatus, cBufLD[devIndex]);
        return false;
    }
    FT_SetTimeouts(ftHandle, 1, 1);	// 0.001 second read timeout

    EmptyBuffers();

    pthread_create(&th_id,NULL,static_read_thread,this);

    return true;
}

bool D2XXSerial::Close()
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

        printf("FTClose()\n");
    FT_Close(ftHandle);
    ftHandle = 0;
    return true;
}

size_t D2XXSerial::Send(const unsigned char * buffer,size_t size)
{
    FT_STATUS ftStatus;
    DWORD wes;
    if (!ftHandle) return 0;
    ftStatus = FT_Write(ftHandle, (unsigned char*)buffer, size, &wes);
    if (ftStatus != FT_OK) {
        printf("Error FT_Write(%d), cBufLD[i] = %s\n", (int)ftStatus, cBufLD[devIndex]);
        return 0;
    }
    return wes;
}
    
//size_t D2XXSerial::Receive(unsigned char * buffer, size_t size)
//{
//    size_t w = 0;
//    pthread_mutex_lock(&data_mtx);
//    for (w=0;w<std::min(size,data.size());w++) {
//        buffer[w] = data.front();
//        data.pop_front();
//    }
//    pthread_mutex_unlock(&data_mtx);
//    return w;
//}
//
//
//
//// Alec: Is this ever used?
//bool D2XXSerial::WaitData(size_t millisec)
//{
//  printf("WaitData\n");
//    struct timespec ts;
//    bool data_avalaible = false;
//    pthread_mutex_lock(&data_mtx);
//    //clock_gettime(CLOCK_REALTIME, &ts);
//    current_utc_time(&ts);
//    double timeout = ts.tv_sec + 1e-9*ts.tv_nsec + 1e-3*millisec;
//    while (data.size() == 0) {
//        //clock_gettime(CLOCK_REALTIME, &ts);
//        current_utc_time(&ts);
//        double remaining = timeout - (ts.tv_sec + 1e-9*ts.tv_nsec);
//        if (remaining <= 0.0) {
//            break;
//        }
//        ts.tv_sec = (unsigned long)floor(timeout);
//        ts.tv_nsec = (unsigned long)floor((timeout - ts.tv_sec)*1e9);
//        // Release mutex and wait a bit
//        pthread_cond_timedwait(&data_cond, &data_mtx, &ts);
//        // data_cond might have been triggered by a read timeout
//    }
//    data_avalaible = (data.size() > 0);
//    pthread_mutex_unlock(&data_mtx);
//    return data_avalaible;
//}
