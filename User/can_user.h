//
// Created by yangxiao on 2026/10/3.
//

#ifndef CAN_CAN_USER_H
#define CAN_CAN_USER_H
#include "../Core/Inc/can.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mailbox;
extern CAN_FilterTypeDef can_filter_config;
#ifdef __cplusplus
}
#endif
#endif //CAN_CAN_USER_H
