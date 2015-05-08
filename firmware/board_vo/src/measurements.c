#include "uart_functions.h"
#include "uart_queue.h"
#include "stm32f10x_it.h"
#include "node_functions.h"
#include "i2c_functions.h"
#include "spi_functions.h"
#include "stdio.h"
#include "platform_config.h"

#include "measurements.h"

Measurements measurements;

extern uint8_t n_sensors;
extern uint8_t Buffer_Tx2[];
extern uint16_t *CALIB_VALUES;

void initialize_measurements() {
    int i;
    measurements.i2c_request = 0;
    measurements.num_measurements = 0;
    for (i=0;i<MAX_NUM_MEASUREMENTS;i++) {
        measurements.buffer[2*i+0] = 0;
        measurements.buffer[2*i+1] = 0;
        measurements.angle[i] = 0;
    }
}

void collect_joint_measurements(int num_sensors) {
    int i;
    measurements.num_measurements = num_sensors;
    for(i=0; i<num_sensors ; i++) {
        unsigned int angle = read_spi(i);

        if(angle)
        {
        	measurements.angle[i] = (angle - CALIB_VALUES[i] + 0x7FFF) % 0xFFFF;
        }
        else
        {
            measurements.angle[i] = angle;
        }

        // store in buffer to be sent to collector via i2c
        measurements.buffer[2*i+0] = measurements.angle[i] >> 8;
        measurements.buffer[2*i+1] = measurements.angle[i] ;
    }
}

void pack_joint_measurements_for_i2c() {
    __disable_irq();

    int i;

	for(i=0; i<n_sensors*2 ; i++)
	{
		Buffer_Tx2[i] = measurements.buffer[i];
    }
    __enable_irq();
}

void create_joint_message(int node_id, Line *l) {
    l->length = snprintf(l->text,UART_QUEUE_LINE_LENGTH,
            "m,%02x,%04x,%04x,%04x\r\n", node_id, 
            measurements.angle[0], measurements.angle[1], measurements.angle[2]);
}



