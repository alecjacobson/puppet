/*
 * node_functions.h
 *
 *  Created on: Jan 12, 2012
 *      Author: glausero
 */

#ifndef NODE_FUNCTIONS_H_
#define NODE_FUNCTIONS_H_

#include "configuration.h"

typedef enum{
		I2C_ID_BROADCAST = 0x04,
		I2C_COMPLETE = 0x01,
		I2C_SPLITTER = 0x02,
		SET_LED = 0x07,
		CALIB = 0x20,
		NEW_NODE = 0x44
		} NodeCommand;

/* function prototypes */
void calibrate_angles();
void set_leds(uint16_t rate_r, uint16_t rate_g, uint16_t rate_b);
void set_type(uint8_t module_type);
void Delay(__IO uint32_t nTime);
void waiter_config();
void wait(uint16_t usTime);
void topo_config();
void led_config();
void send_topo(uint16_t PIN);
ErrorStatus is_splitter(uint8_t type_test);
ErrorStatus is_joint(uint8_t type_test);

#endif /* NODE_FUNCTIONS_H_ */
