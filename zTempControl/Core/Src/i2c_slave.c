/*
 * i2c_slave.c
 *
 *  Created on: Jun 21, 2024
 *      Author: al
 */
#include "i2c_slave.h"

uint8_t i2cRxBuffer[TEMP_RX_BYTES];
uint8_t i2cTxBuffer[TEMP_TX_BYTES];


/**
 * @brief	Startet I2C HAL in Slave Mode DMA
 */
void i2c_slaveStartListen(I2C_HandleTypeDef *I2cHandle) {
	//wegen flipper nur 1, sonst TEMP_RX_BYTES
	if (HAL_I2C_Slave_Receive_DMA(I2cHandle, i2cRxBuffer, 1)
			!= HAL_OK) {
		/* Transfer error in reception process */
		Error_Handler();
	}
}

/**
 * @brief  Tx Transfer completed callback.
 * @param  I2cHandle: I2C handle.
 * @note   Switcht vom Sender wieder zum Reciever
 * @retval None
 */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
	if (HAL_I2C_Slave_Receive_DMA(I2cHandle, i2cRxBuffer, 1) //flipperzero kann nur ein, sonst TEMP_RX_BYTES
	!= HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief  Rx Transfer completed callback.
 * @param  I2cHandle: I2C handle
 * @note   Schaltet nach Empfang wieder in Slave Mode
 *         kopiert die empfangenen Daten vom Buffer in das Register
 *         wenn man wissen will dass was empfangen wurde, einfach dataRxRegister[0] ansehen
 * @retval None
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {

	//Wenn recievt wird, wird auch gesendet, daher erstmal die Registerdaten in den Buffer schieben
	memcpy(i2cTxBuffer, (uint8_t*) &dataTxRegister, TEMP_TX_BYTES);

	/* Jetzt verschick mal wieder Modus*/
	if (HAL_I2C_Slave_Transmit_DMA(&hi2c1, i2cTxBuffer, 2) //flipperzero kann nur zwei, sont TEMP_TX_BYTES
	!= HAL_OK) {
		Error_Handler();
	}
	/* kucken bis er wirklich in Slave Mode is */
	while (HAL_I2C_GetMode(I2cHandle) != HAL_I2C_MODE_SLAVE) {
	}
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	//Nach Empfang die Bufferdaten in das Register schieben
	memcpy(dataRxRegister, &i2cRxBuffer, TEMP_RX_BYTES);
}


//memcpy(aTxBuffer, (uint8_t*) &i2cRegister2, 6);


/*
 void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
 {
 HAL_I2C_EnableListen_IT(hi2c);
 }

 void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
 {
 if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
 {
 // receive using sequential function.
 // The I2C_FIRST_AND_LAST_FRAME implies that the slave will send a NACK after receiving "entered" num of bytes
 HAL_I2C_Slave_Sequential_Receive_IT(hi2c, i2cRXBuffer, BUFFER_SIZE, I2C_FIRST_AND_LAST_FRAME);
 }
 else  // if the master requests the data from the slave
 {
 Error_Handler();  // call error handler
 }
 }

 void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
 {
 count++;
 }

 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_EnableListen_IT(hi2c);
}

