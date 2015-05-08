/*
 * i2c_functions.h
 *
 *  Created on: Jan 13, 2012
 *      Author: glausero
 */

#ifndef I2C_FUNCTIONS_H_
#define I2C_FUNCTIONS_H_

void i2c_config();
ErrorStatus send_i2c(I2C_TypeDef * I2Cx, uint8_t * TxBuffer, uint8_t slave_address, uint8_t package_size);
ErrorStatus receive_i2c(I2C_TypeDef * I2Cx, uint8_t * RxBuffer, uint8_t slave_address, uint8_t package_size);

#endif /* I2C_FUNCTIONS_H_ */
