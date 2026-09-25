#ifndef CAN_H
#define CAN_H

#include "stm32g4xx_hal.h"
#include "flash.h"

typedef struct {
	FDCAN_HandleTypeDef *hcan;

	FDCAN_TxHeaderTypeDef *TxHeader;
	FDCAN_RxHeaderTypeDef *RxHeader;

	uint8_t tx_data[8];
	uint8_t rx_data[8];

	int tx_success;
	int tx_fail;
	int rx_success;
	int rx_fail;

	uint32_t can_err;
} CAN_t;

void CAN_Cfg(CAN_t *can, FDCAN_HandleTypeDef *hcan);
void CAN_Header_Cfg(CAN_t *can, FDCAN_TxHeaderTypeDef *TxHeader, FDCAN_RxHeaderTypeDef *RxHeader, uint32_t id);

uint8_t CAN_Send(CAN_t *can, uint8_t *data);

#endif
