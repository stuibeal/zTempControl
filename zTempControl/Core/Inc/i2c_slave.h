/*
 * i2c_slave.h
 *
 *  Created on: Jun 21, 2024
 *      Author: al
 */
#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#include "main.h"
#include "zCommunication.h"
#include <stdint.h>
#include <string.h>
#include "common.h"

extern I2C_HandleTypeDef hi2c1;

void i2c_slaveStartListen(I2C_HandleTypeDef *hi2c);



#endif /* INC_I2C_SLAVE_H_ */
