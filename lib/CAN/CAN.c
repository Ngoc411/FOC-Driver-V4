#include "CAN.h"

//static const uint32_t FDCAN_DLC_MAP[9] = {
//    FDCAN_DLC_BYTES_0,  /* [0] = 0 bytes */
//    FDCAN_DLC_BYTES_1,  /* [1] = 1 byte  */
//    FDCAN_DLC_BYTES_2,  /* [2] = 2 bytes */
//    FDCAN_DLC_BYTES_3,  /* [3] = 3 bytes */
//    FDCAN_DLC_BYTES_4,  /* [4] = 4 bytes */
//    FDCAN_DLC_BYTES_5,  /* [5] = 5 bytes */
//    FDCAN_DLC_BYTES_6,  /* [6] = 6 bytes */
//    FDCAN_DLC_BYTES_7,  /* [7] = 7 bytes */
//    FDCAN_DLC_BYTES_8,  /* [8] = 8 bytes */
//};

void CAN_Cfg(CAN_t *can, FDCAN_HandleTypeDef *hcan) {
	can->hcan = hcan;
}

void CAN_Header_Cfg(CAN_t *can, FDCAN_TxHeaderTypeDef *TxHeader, FDCAN_RxHeaderTypeDef *RxHeader, uint32_t id) {
	can->TxHeader = TxHeader;
	can->RxHeader = RxHeader;

	can->TxHeader->Identifier			= id;
	can->TxHeader->IdType				= FDCAN_STANDARD_ID;
	can->TxHeader->TxFrameType			= FDCAN_DATA_FRAME;
	can->TxHeader->FDFormat				= FDCAN_CLASSIC_CAN;
	can->TxHeader->BitRateSwitch		= FDCAN_BRS_OFF;
	can->TxHeader->ErrorStateIndicator	= FDCAN_ESI_ACTIVE;
	can->TxHeader->TxEventFifoControl	= FDCAN_NO_TX_EVENTS;
	can->TxHeader->MessageMarker		= 0;
	can->TxHeader->DataLength			= FDCAN_DLC_BYTES_8;
}

uint8_t CAN_Send(CAN_t *can, uint8_t *data)
{
    if (HAL_FDCAN_GetTxFifoFreeLevel(can->hcan) == 0) { // free level
        can->tx_fail++;
        return 0;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(can->hcan, can->TxHeader, data) != HAL_OK) {
        can->tx_fail++;
        return 0;
    }

    return 1;
}

