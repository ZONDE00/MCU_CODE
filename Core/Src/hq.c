/*
 * hq.c
 *
 *  Created on: Dec 17, 2022
 *      Author: Kristers
 */

#include "hq.h"
#include "string.h"
#include "stdio.h"
#include "boardtrx.h"

HQ_Handle *hqHandle;

// UART STUFF
#define HQ_UART_START_BYTE      0x45
uint8_t uartRxBuf[64];
uint8_t uartRxLen = 255;
uint8_t uartRxToGet = 0;
uint8_t uartGotFrame = 0;
uint8_t uartMiniBuf;
uint8_t uartRxToReceive = 0;

// callback variables
uint8_t buzzTime = 0;
uint8_t burnTime = 0;

// TIMER STUFF

uint32_t timer = 0;
// well do stuff every second
uint32_t timerDelay = 10000;

// local variables that can be sent to COM
BOARDTRX_RX_Data mcuRxBatVolt = { 0 };
BOARDTRX_RX_Data mcuRx4vCurr = { 0 };
BOARDTRX_RX_Data mcuRx3v3Curr = { 0 };
BOARDTRX_RX_Data mcuRxTemp = { 0 };
BOARDTRX_RX_Data testVarRx = { 0 };

// TODO replace testVarRx with missing variables
// variable and cmd pointer arrays, sequence of variables should be like @ref HQ_Commands_TX
BOARDTRX_RX_Data *rxDataArray[12] = { &mcuRxBatVolt, &mcuRx4vCurr, &mcuRx3v3Curr, &mcuRxTemp, &testVarRx, &testVarRx, &testVarRx, &testVarRx,
        &testVarRx, &testVarRx, &testVarRx, &testVarRx, };

//HQ_VARS_RX_LONGITUDE,
//HQ_VARS_RX_LATITUDE,
//HQ_VARS_RX_ALTITUDE,
//HQ_VARS_RX_SPEED,
//HQ_VARS_RX_TIME
//HQ_VARS_RX_MCU_TEMP,
//HQ_VARS_RX_CMD_OK,
//HQ_VARS_RX_CMD_NCK,
//HQ_VARS_RX_CMD_CRC,

uint8_t gpsLong[12];
uint8_t gpsLat[12];
uint8_t gpsAlt[8];
uint8_t gpsSpeed[7];
uint8_t gpsTime[6];
float comTemperature;
uint32_t comCmdOk;      // messages that were  ok
uint32_t comCmdNck;     // messages that were not ok
uint32_t comCmdCrc;     // messages with crc errors
uint32_t comCmdTmo;     // messages that timed out

BOARDTRX_TX_Data comTxGpsLong = { 0 };
BOARDTRX_TX_Data comTxGpsLat = { 0 };
BOARDTRX_TX_Data comTxGpsAlt = { 0 };
BOARDTRX_TX_Data comTxGpsSpeed = { 0 };
BOARDTRX_TX_Data comTxGpsTime = { 0 };
BOARDTRX_TX_Data comTxComTemperature = { 0 };
BOARDTRX_TX_Data comTxComCmdOk = { 0 };
BOARDTRX_TX_Data comTxComCmdNck = { 0 };
BOARDTRX_TX_Data comTxComCmdCrc = { 0 };
BOARDTRX_TX_Data comTxComCmdTmo = { 0 };

// data that can be received from COM
BOARDTRX_TX_Data *txDataArray[10] = { &comTxGpsLong, &comTxGpsLat, &comTxGpsAlt, &comTxGpsSpeed, &comTxGpsTime, &comTxComTemperature, &comTxComCmdOk, &comTxComCmdNck,
        &comTxComCmdCrc, &comTxComCmdTmo};


BOARDTRX_RX_Cmd mcuTxBurn = { 0 };
BOARDTRX_RX_Cmd comTrxBuzz = { 0 };

// commands that COM can ask for MCU to execute, only burner and buzzer
BOARDTRX_RX_Cmd *txCmdArray[2] = { &mcuTxBurn, &comTrxBuzz };

// initialize main trx handle
BOARDTRX_Handle comTrxHandle = { 0 };

// stolen variables, should be put in @ref trxDataArray with their proper structs
extern uint16_t vref_avg;
extern uint16_t temp_avg;
extern float vdda; // Result of VDDA calculation
extern float vref; // Result of vref calculation
extern float temp; // Result of temp calculation
extern float v3_3; // Result of 3.3 ADC calculation
extern float v4;   // Result of 4 ADC calculation
extern float i3_3; // Result of 3.3 ADC calculation
extern float i4;   // Result of 4 ADC calculation
extern float bat_v; // Result of BAT_V calculation

// for testing TODO remove once proper variables in @ref trxDataArray are set
uint32_t testVariable = 2244668800;

/*
 * @Brief Initialises HQ_Handle and starts data receiving
 * @param HQ_Handle - pointer to configured handle
 * @retval value of initialisation status
 */
HQ_StatusTypeDef HQ_Init(HQ_Handle *handle) {

    hqHandle = handle;

    // initialize shared variables tx

    comTxGpsLong.data = (uint8_t *)&gpsLong;
    comTxGpsLong.size = 12;

    comTxGpsLat.data = (uint8_t *)&gpsLat;
    comTxGpsLat.size = 12;

    comTxGpsAlt.data = (uint8_t *)&gpsAlt;
    comTxGpsAlt.size = 8;

    comTxGpsSpeed.data = (uint8_t *)&gpsSpeed;
    comTxGpsSpeed.size = 7;

    comTxGpsTime.data = (uint8_t *)&gpsTime;
    comTxGpsTime.size = 6;

    comTxComTemperature.data = (uint8_t *)&comTemperature;
    comTxComTemperature.size = 4;

    comTxComCmdOk.data = (uint8_t *)&comCmdOk;
    comTxComCmdOk.size = 4;

    comTxComCmdNck.data = (uint8_t *)&comCmdNck;
    comTxComCmdNck.size = 4;

    comTxComCmdCrc.data = (uint8_t *)&comCmdCrc;
    comTxComCmdCrc.size = 4;

    comTxComCmdTmo.data = (uint8_t *)&comCmdTmo;
    comTxComCmdTmo.size = 4;

    // initialize shared variables
    mcuRxBatVolt.data = (uint8_t*) &bat_v;
    mcuRxBatVolt.size = 4;

    mcuRx4vCurr.data = (uint8_t*) &i4;
    mcuRx4vCurr.size = 4;

    mcuRx3v3Curr.data = (uint8_t*) &i3_3;
    mcuRx3v3Curr.size = 4;

    mcuRxTemp.data = (uint8_t*) &temp;
    mcuRxTemp.size = 4;

    testVarRx.data = (uint8_t*) &testVariable;
    testVarRx.size = 4;

    // initialize remote commands
    mcuTxBurn.CMD_CB = HQ_BURN_WORLD;
    mcuTxBurn.data = &burnTime;
    mcuTxBurn.size = 1;

    comTrxBuzz.CMD_CB = HQ_BUZZZ;
    comTrxBuzz.data = &buzzTime;
    comTrxBuzz.size = 1;

    // this is what can be requested
    comTrxHandle.countDataRx = 12;
    comTrxHandle.dataRx = rxDataArray;
    // some timings
    comTrxHandle.rxRetries = 3;
    comTrxHandle.rxTimeout = 20000;
    // this is what we can request
    comTrxHandle.countDataTx = 10;
    comTrxHandle.dataTx = txDataArray;
    // this is commands that can be executed here
    comTrxHandle.countCmdRx = 2;
    comTrxHandle.cmdRx = txCmdArray;
    // other config
    comTrxHandle.uart = handle->uart;
    BOARDTRX_Status ret;
    ret = BOARDTRX_Init(&comTrxHandle);

    if (ret.isNewError) {
        return HQ_ERROR;
    }

    return HQ_OK;
}

HQ_StatusTypeDef HQ_Loop() {
    // TODO process ret
    BOARDTRX_Status ret;
    ret = BOARDTRX_Loop();

    return HQ_IDLE;
}

/*
 * @brief function for burner
 * @ TODO ask Rodruigo to implement this, should maybe also pass parameters directly instead of
 *          global variables
 */
void HQ_BURN_WORLD() {
    for (uint8_t i = 0; i < burnTime; i++) {
        printf("BURNING THINGS \n\r");
        HAL_Delay(1000);
    }
}

/*
 * @brief function for buzzer
 * @ TODO ask Rodruigo to implement this, should maybe also pass parameters directly instead of
 *          global variables
 */

void HQ_BUZZZ() {
    for (uint8_t i = 0; i < buzzTime; i++) {
        printf("Do you hear bees? \n\r");
        HAL_Delay(1000);
    }
}

