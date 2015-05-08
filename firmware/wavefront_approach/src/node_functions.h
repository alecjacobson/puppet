/*
 * node_functions.h
 *
 *  Created on: Jan 12, 2012
 *      Author: glausero
 */

#ifndef NODE_FUNCTIONS_H_
#define NODE_FUNCTIONS_H_

/* typedefs */
typedef enum{
		COLLECTOR = 0x00,
		JOINT_TRT = 0xF4,  	// 0b11110100
		JOINT_R = 0x94, 	// 0b10010100
		JOINT_T = 0x54,   	// 0b01010100
		SPLITTER_2 = 0x28,
		SPLITTER_3 = 0x38,
		SPLITTER_5 = 0x58,
		UNDEFINED = 0xFF
		} NodeType;

typedef enum{
		I2C_ID = 0x00,
		I2C_COMPLETE = 0x01,
		I2C_SPLITTER = 0x02,
		I2C_SPLITTER_START = 0x03,
		SET_LED = 0x07,
		I2C_DATA = 0x04,
		I2C_NEW = 0x08
		} NodeCommand;

typedef enum{
		NODE_POWERED = 0x00,
		NODE_SAMPLING_START = 0x01,
		NODE_SAMPLING_END = 0x02,
		NODE_IDED = 0x03,
		NODE_PROBING = 0x04
		} NodeMode;

struct{
uint8_t completed : 1;
uint8_t exists : 1;
uint8_t id : 8;
} next_splitter;

struct{
uint8_t taken : 1;
uint8_t type : 8;
} id_map[256];

/* function prototypes */
void set_leds(uint16_t rate_r, uint16_t rate_g, uint16_t rate_b);
void set_type(uint8_t module_type);
void Delay(__IO uint32_t nTime);
void topo_config();
void led_config();
void send_topo(uint16_t PIN);
void waiter_config();
void wait(uint16_t usTime);
ErrorStatus is_splitter(uint8_t type_test);
ErrorStatus is_joint(uint8_t type_test);

#endif /* NODE_FUNCTIONS_H_ */
