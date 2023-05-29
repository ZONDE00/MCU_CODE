/*
 * hq.h
 *
 *  Created on: Dec 17, 2022
 *      Author: Kristers
 */

#ifndef INC_HQ_H_
#define INC_HQ_H_

#include "main.h"

// variables that MCU can requested from COM
typedef enum {
    HQ_CMD_RX_TIME,
    HQ_CMD_RX_LONGITUDE,
    HQ_CMD_RX_LATITUDE,
    HQ_CMD_RX_ALTITUDE,
    HQ_CMD_RX_SPEED,
    HQ_CMD_RX_MCU_TMP,
    HQ_CMD_RX_CMD_OK,
    HQ_CMD_RX_CMD_NCK,
    HQ_CMD_RX_CMD_CRC,
} HQ_Commands_RX;

// variables that COM can request from MCU
typedef enum {
    HQ_CMD_TX_BAT_VOLT,
    HQ_CMD_TX_4V_CUR,
    HQ_CMD_TX_3V3_CUR,
    HQ_CMD_TX_3V3,
    HQ_CMD_TX_4V,
    HQ_CMD_TX_TMP_MCU,
    HQ_CMD_TX_TMP_IN ,
    HQ_CMD_TX_TMP_OUT,
    HQ_CMD_TX_PRESSURE,
    HQ_CMD_TX_HUMID,
    HQ_CMD_TX_AIRQ,
    HQ_CMD_TX_SEN_STATUS
} HQ_Commands_TX;

typedef enum {
    HQ_IDLE,
    HQ_OK,
    HQ_ERROR,
    HQ_BUSY,
    HQ_TIMEOUT,
    HQ_ERROR_UART,
    HQ_ERROR_CRC,
    HQ_ERROR_CMD,
} HQ_StatusTypeDef;

typedef struct {
    UART_HandleTypeDef *uart; // for communication with COM
} HQ_Handle;

HQ_StatusTypeDef HQ_Init(HQ_Handle *handle);

HQ_StatusTypeDef HQ_Loop();

void HQ_BURN_WORLD();
void HQ_BUZZZ();

#endif /* INC_HQ_H_ */
