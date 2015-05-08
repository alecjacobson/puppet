/*
 * configuration.h
 *
 *  Created on: Mar 8, 2012
 *      Author: glausero
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

typedef enum{
		COLLECTOR = 0x00,
		JOINT_TRT = 0xF4,
		JOINT_R = 0x94,
		JOINT_T = 0x54,
		SPLITTER_2 = 0x28,
		SPLITTER_3 = 0x38,
		SPLITTER_5 = 0x58,
		UNDEFINED = 0xFF
		} NodeType;

#endif /* CONFIGURATION_H_ */
