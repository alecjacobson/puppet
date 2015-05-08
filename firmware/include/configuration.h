/*
 * configuration.h
 *
 *  Created on: Mar 8, 2012
 *      Author: glausero
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#ifdef __cplusplus
namespace Puppet {
#endif

    typedef enum{
        NODE_COLLECTOR  = 0x00, 
        NODE_JOINT_TRT  = 0xF4, // 3DOF, Twist-Bend-Twist
        NODE_JOINT_R    = 0x94, // 1DOF, Bend
        NODE_JOINT_T    = 0x54, // 1DOF, Twist
        NODE_SPLITTER_GENERIC = 0x08, // Splitter 2 out-going joints
        NODE_SPLITTER_2 = 0x28, // Splitter 2 out-going joints
        NODE_SPLITTER_3 = 0x38, // Splitter 3 out-going joints
        NODE_SPLITTER_5 = 0x58, // Splitter 5 out-going joints
        NODE_UNDEFINED  = 0xFF
    } NodeType;

#define NODE_SPLITTER_MASK 0x0F
#define NODE_SPLITTER_COMPLETE 0xFF

// Alec: doesn't make sense to have 0 be when the bending joint is
// overlapping upon itself. Better to have 0 be when it is straight
//#define NODE_ANGLE_OFFSET_RAD (0.)
#define NODE_ANGLE_OFFSET_RAD (-M_PI)
#define NODE_ANGLE_CONVERSION_FACTOR_RAD (2*M_PI/0x10000)

#ifdef __cplusplus
};
#endif


#endif /* CONFIGURATION_H_ */
