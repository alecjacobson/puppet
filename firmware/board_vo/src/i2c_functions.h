/*
 * i2c_functions.h
 *
 *  Created on: Jan 13, 2012
 *      Author: glausero
 */

#ifndef I2C_FUNCTIONS_H_
#define I2C_FUNCTIONS_H_

typedef enum{
		NONE = 0x00,
		BUSY = 0x01,
		NOT_MASTER = 0x02,
		NO_ACKNOWLEDGEMENT = 0x03,
		DATA_NOT_SENT = 0x04,
		STOP_NOT_SENT = 0x05,
		DATA_NOT_RECEIVED = 0x06
		} I2cError;

void i2c_config();
ErrorStatus send_i2c(I2C_TypeDef * I2Cx, uint8_t * TxBuffer, uint8_t slave_address, uint8_t package_size);
ErrorStatus receive_i2c(I2C_TypeDef * I2Cx, uint8_t * RxBuffer, uint8_t slave_address, uint8_t package_size);
void clear_busy_i2c();

#endif /* I2C_FUNCTIONS_H_ */
