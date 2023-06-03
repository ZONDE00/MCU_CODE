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
BOARDTRX_RX_Data mcuRx3v3 = { 0 };
BOARDTRX_RX_Data mcuRxTemp = { 0 };
BOARDTRX_RX_Data mcuRxTempIn = { 0 };
BOARDTRX_RX_Data mcuRxTempOut = { 0 };
BOARDTRX_RX_Data mcuRxTempBme = { 0 };
BOARDTRX_RX_Data mcuRxPressure = { 0 };
BOARDTRX_RX_Data mcuRxHumid = { 0 };
BOARDTRX_RX_Data mcuRxAirQ = { 0 };
BOARDTRX_RX_Data mcuRxGasOhm = { 0 };
BOARDTRX_RX_Data mcuRxSenState = { 0 };

// variable and cmd pointer arrays, sequence of variables should be like @ref HQ_Commands_RX
BOARDTRX_RX_Data *rxDataArray[HQ_CMD_RX_END] = {
        &mcuRxBatVolt, &mcuRx4vCurr, &mcuRx3v3Curr, &mcuRx3v3,
        &mcuRxTemp, &mcuRxTempIn, &mcuRxTempOut, &mcuRxTempBme,
        &mcuRxPressure, &mcuRxHumid, &mcuRxAirQ, &mcuRxGasOhm,
        &mcuRxSenState };

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
BOARDTRX_TX_Data *txDataArray[HQ_CMD_TX_END] = {
        &comTxGpsLong, &comTxGpsLat, &comTxGpsAlt, &comTxGpsSpeed,
        &comTxGpsTime, &comTxComTemperature,
        &comTxComCmdOk, &comTxComCmdNck, &comTxComCmdCrc, &comTxComCmdTmo};

BOARDTRX_RX_Cmd mcuTxBurn = { 0 };
BOARDTRX_RX_Cmd comTrxBuzz = { 0 };

// commands that COM can ask for MCU to execute, only burner and buzzer
BOARDTRX_RX_Cmd *txCmdArray[2] = { &mcuTxBurn, &comTrxBuzz };

// initialize main trx handle
BOARDTRX_Handle comTrxHandle = { 0 };

// stolen variables, should be put in @ref trxDataArray with their proper structs
extern float vdda; // Result of VDDA calculation
extern float temp; // Result of temp calculation ?? TODO is this temperature or temporary, should be temp either way
extern float i3_3; // Result of 3.3 ADC calculation
extern float i4;   // Result of 4 ADC calculation
extern float bat_v; // Result of BAT_V calculation
extern float temp_in;
extern float temp_out;
extern float temp_bme;
extern float bme_pressure;
extern float bme_air_q;
extern float bme_humid;
extern float bme_air_ohm;
extern uint32_t senStatus;

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

    mcuRx3v3.data = (uint8_t*) &vdda;
    mcuRx3v3.size = 4;

    mcuRxTemp.data = (uint8_t*) &temp;
    mcuRxTemp.size = 4;

    mcuRxTempIn.data = (uint8_t*) &temp_in;
    mcuRxTempIn.size = 4;

    mcuRxTempOut.data = (uint8_t*) &temp_out;
    mcuRxTempOut.size = 4;

    mcuRxTempBme.data = (uint8_t*) &temp_bme;
    mcuRxTempBme.size = 4;

    mcuRxPressure.data = (uint8_t*) &bme_pressure;
    mcuRxPressure.size = 4;

    mcuRxHumid.data = (uint8_t*) &bme_humid;
    mcuRxHumid.size = 4;

    mcuRxAirQ.data = (uint8_t*) &bme_air_q;
    mcuRxAirQ.size = 4;

    mcuRxGasOhm.data = (uint8_t*) &bme_air_ohm;
    mcuRxGasOhm.size = 4;

    mcuRxSenState.data = (uint8_t*) &senStatus;
    mcuRxSenState.size = 4;

    // initialize remote commands
    mcuTxBurn.CMD_CB = HQ_BURN_WORLD;
    mcuTxBurn.data = &burnTime;
    mcuTxBurn.size = 1;

    comTrxBuzz.CMD_CB = HQ_BUZZZ;
    comTrxBuzz.data = &buzzTime;
    comTrxBuzz.size = 1;

    // this is what can be requested
    comTrxHandle.countDataRx = HQ_CMD_RX_END;
    comTrxHandle.dataRx = rxDataArray;
    // some timings
    comTrxHandle.txRetries = 3;
    comTrxHandle.txTimeout = 20000;
    // this is what we can request
    comTrxHandle.countDataTx = HQ_CMD_TX_END;
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

