//
// Created by yangxiao on 2026/10/3.
//
/* UserCode/can_user.h */
#include "stdint.h"
#include "can.h"

#include "can_user.h"

/* UserCode/can_user.c */
uint32_t can_tx_mailbox = 0;

CAN_TxHeaderTypeDef tx_header = {
    .StdId = 0x200,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

CAN_RxHeaderTypeDef rx_header = {
    .StdId = 0x203,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8
};

// CAN_RxHeaderTypeDef rx_header = {0};

CAN_FilterTypeDef can_filter_config = {
    .FilterBank = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_32BIT,
    .FilterIdHigh = 0x0000,
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000,
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_RX_FIFO0,
    .FilterActivation = ENABLE,
    .SlaveStartFilterBank = 14,
};
