/* USER CODE BEGIN Header */
/*
 * XXX: temp sensors has delay(100) thats painfully bad, there should be better way
 * XXX:  RTC INIT FAILED! <--- fix pls
 * XXX: sensor status as register where each bit represents whether or not sensor is working
 * XXX: this file is a mess
 *
 *
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "mcp795.h"
#include "bme680_defs.h"
#include "bme680.h"
#include "hq.h"
#include "boardtrx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CRC_TEST_STRING "poopdy_doo"
#define ADC_RESOLUTION 4095
#define ADC_SAMPLES 80

/* Rope cut and sleep mode defines */
#define ROPE_NOT_CUT 0
#define ROPE_CUTTING 1
#define ROPE_CUT_DONE 2

#define ROPE_CUT_TIME 10			// in seconds
#define LOWER_ALTITUDE_LIMIT 1000 	// in meters
#define UPPER_ALTITUDE_LIMIT 28000 	// in meters
#define ALTITUDE_UPDATE_PERIOD 30	// in seconds

/* SD card related defines */
#define SD_MAX_RETRIES 5

/* Camera related defines */
#define CAM_NOK 0
#define CAM_OK 1
#define CAM_NREC 0
#define CAM_REC 1
#define CAM_STARTUP_TIME 6	// x*5 seconds
#define CAM_OFF 0
#define CAM_ON 1
#define CAM_RECORDING 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart4_rx;

/* USER CODE BEGIN PV */

/* SD card handling variables */
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
volatile uint32_t totalSpace, freeSpace;
char buffer[100];
volatile FRESULT sd_status;
volatile uint8_t sd_mounted;
volatile uint8_t sd_file_cnt;
volatile uint8_t sd_error_cnt;
rtc_time_t rtc_time;
/* SD card handling variables */

/* CAM specific */
typedef struct CAM_struct {
    uint16_t CAM_HB_Pin;
    GPIO_TypeDef *CAM_HB_Port;
    uint8_t CAM_ON_OFF;
    uint8_t REC_CAM;
    TIM_HandleTypeDef *TIM;
    uint8_t REC_STATUS;
    uint8_t Do_Restart;
    uint8_t Do_EXTI;
    uint8_t Startup_delay;
    uint8_t CAM_STATUS;
} CAM_struct;

CAM_struct CAM_1;
CAM_struct CAM_2;
/* CAM specific */

float ta, tb; // transfer function using calibration data

uint16_t adc_buffer[ADC_SAMPLES * 5 * 2] = { 0 }; // ADC_SAMPLES samples, 5 channels, 2 buffers

uint32_t tim_cnt = 0;

uint16_t vref_avg = 0;
uint16_t temp_avg = 0;
float vdda = 0; // Result of VDDA calculation
float vref = 0; // Result of vref calculation
float temp = 0; // Result of temp calculation
float v3_3 = 0; // Result of 3.3 ADC calculation
float v4 = 0;   // Result of 4 ADC calculation
float i3_3 = 0; // Result of 3.3 ADC calculation
float i4 = 0;   // Result of 4 ADC calculation
float bat_v = 0; // Result of BAT_V calculation
uint8_t set_required_settings = 0;
uint32_t crc = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
uint8_t Mount_open_SD_Card();
void START_CAM(CAM_struct *CAM);
void OFF_CAM(CAM_struct *CAM);
void STOP_CAM(CAM_struct *CAM);
void ON_CAM(CAM_struct *CAM);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);
    return ch;

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    mcp795_t rtc;
    adt7310_t adt7310_in;
    adt7310_t adt7310_out;
    struct bme680_dev bme680;
    bme680.dev_id = BME680;
    bme680.intf = BME680_SPI_INTF;
    bme680.read = bus_read;
    bme680.write = bus_write;
    bme680.delay_ms = HAL_Delay;
    // amb_temp can be set to 25 prior to configuring the gas sensor
    //or by performing a few temperature readings without operating the gas sensor.
    bme680.amb_temp = 25;
    bme680.hspi = &hspi1;
//  uint8_t rtc_status = 0;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_CRC_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_USART4_UART_Init();
    if (MX_FATFS_Init() != APP_OK) {
        Error_Handler();
    }
    MX_TIM15_Init();
    MX_TIM14_Init();
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_TIM7_Init();
    /* USER CODE BEGIN 2 */
    /*------------------SD card setup----------------------*/
    /* Wait for SD module reset */
    HAL_Delay(2000);

    sd_file_cnt = 0;
    sd_error_cnt = 0;
    sd_mounted = 0;
    sd_status = Mount_open_SD_Card();

    while (sd_status == FR_NOT_READY)
        sd_status = Mount_open_SD_Card();

    if (sd_status == FR_OK) {
        sd_mounted = 1;
    } else {
        printf("Card status: %u\r\n", sd_status);
    }
    /*------------------SD card setup----------------------*/
    /*----------------- Camera init ----------------------*/
    CAM_1.CAM_HB_Pin = CAM_HB_0_Pin;
    CAM_1.CAM_HB_Port = CAM_HB_0_GPIO_Port;
    CAM_1.CAM_ON_OFF = CAM0_ON;
    CAM_1.REC_CAM = CAM0_REC;
    CAM_1.REC_STATUS = CAM_NREC;
    CAM_1.TIM = &htim16;
    CAM_1.Do_Restart = 0;
    CAM_1.Do_EXTI = 0;
    CAM_1.Startup_delay = CAM_STARTUP_TIME;
    CAM_1.CAM_STATUS = CAM_OFF;

    CAM_2.CAM_HB_Pin = CAM_HB_1_Pin;
    CAM_2.CAM_HB_Port = CAM_HB_1_GPIO_Port;
    CAM_2.CAM_ON_OFF = CAM1_ON;
    CAM_2.REC_CAM = CAM1_REC;
    CAM_2.REC_STATUS = CAM_NREC;
    CAM_2.TIM = &htim17;
    CAM_2.Do_Restart = 0;
    CAM_2.Do_EXTI = 0;
    CAM_2.Startup_delay = CAM_STARTUP_TIME;
    CAM_2.CAM_STATUS = CAM_OFF;

    ON_CAM(&CAM_1);
    ON_CAM(&CAM_2);
    START_CAM(&CAM_1);
    START_CAM(&CAM_2);
    /*----------------- Camera init ----------------------*/

    if (mcp795_init(&rtc, &hspi1, RTC_MOD) == 6) {
        printf("RTC INIT FAILED!\r\n");
        for (;;);
    } else {
        printf("RTC INIT success!\r\n");
    }
    if (adt7310_init(&adt7310_in, &hspi1, TEMP_IN) != 0) {
        printf("ADT7310 TEMP_IN INIT FAILED!\r\n");
        for (;;);
    } else {
        printf("ADT7310 TEMP_IN INIT successful!\r\n");
    }

    if (adt7310_init(&adt7310_out, &hspi1, TEMP_OUT) != 0) {
        printf("ADT7310 TEMP_OUT INIT FAILED!\r\n");
        for (;;);
    } else {
        printf("ADT7310 TEMP_OUT INIT successful!\r\n");
    }

    printf("\n\nStarting\n\r");
    printf("VREFINT_CAL = %d (0x%04x)\n\r", (uint16_t) *VREFINT_CAL_ADDR, (uint16_t) *VREFINT_CAL_ADDR);
    printf("TEMPSENSOR_CAL1 = %d (0x%04x)\n\r", (uint16_t) *TEMPSENSOR_CAL1_ADDR, (uint16_t) *TEMPSENSOR_CAL1_ADDR);
    printf("TEMPSENSOR_CAL2 = %d (0x%04x)\n\r", (uint16_t) *TEMPSENSOR_CAL2_ADDR, (uint16_t) *TEMPSENSOR_CAL2_ADDR);

    // Calculate transfer function values - a and b in simple linear equation y = ax + b
    calculate_calibration();
    printf("Temp calibration: t = %0.3f * tmeasured + %0.3f\r\n", ta, tb);
    //ta = (float)((uint16_t)TEMPSENSOR_CAL2_TEMP - (uint16_t)TEMPSENSOR_CAL1_TEMP) / ((uint16_t)*TEMPSENSOR_CAL2_ADDR - (uint16_t)*TEMPSENSOR_CAL1_ADDR);
    //tb = (uint16_t)TEMPSENSOR_CAL1_TEMP - ta * (uint16_t)*TEMPSENSOR_CAL1_ADDR;
    //mcp795_read_status_register(&rtc, rtc_status);
    //mcp795_write_id(&rtc, (uint8_t *)RTC_ID);
    //sleep(10);
    mcp795_start_counting(&rtc);
// rtc_time.milliseconds = 00;
// rtc_time.seconds = 00;
// rtc_time.minutes = 00;
// rtc_time.hours = 15;
// rtc_time.days = 7;
// rtc_time.date = 18;
// rtc_time.month = 12;
// rtc_time.year = 22;
// mcp795_set_time(&rtc, &rtc_time);

    HAL_TIM_Base_Start_IT(&htim7); // First get the timer running
    HAL_TIM_Base_Start_IT(&htim14); // First get the timer running
    HAL_TIM_Base_Start_IT(&htim15); // First get the timer running
    HAL_TIM_Base_Start_IT(&htim16); // First get the timer running
    HAL_TIM_Base_Start_IT(&htim17); // First get the timer running

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buffer, ADC_SAMPLES * 2 * 5); // Now fire up the ADC DMA
    uint8_t command[2] = { 0 };
    uint8_t received[2] = { 0 };
    command[0] = 0x80 | 0x0F;
    uint8_t config = 0;
    select_sensor(IMU);
    /* Perform the transaction */
    HAL_SPI_TransmitReceive(&hspi1, command, received, 2, 100);
    deselect_sensors();
    adt7310_set_config(&adt7310_in, ADT7310_CONF_RESOLUTION(1) | ADT7310_MODE_1SPS);
    adt7310_read_reg(&adt7310_in, 1, 1, &config);
    adt7310_set_config(&adt7310_out, ADT7310_CONF_RESOLUTION(1) | ADT7310_MODE_1SPS);
    adt7310_read_reg(&adt7310_out, 1, 1, &config);
    //MAG test

    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        return HAL_ERROR;
    }

    // dummy byter
    HAL_SPI_TransmitReceive(&hspi1, command, received, 2, 100);
// uint8_t id_bme680 = 80 | (1 << 7);
//  select_sensor(BME680);
//  HAL_SPI_Transmit(&hspi1, &id_bme680, 1, 100);
//  HAL_SPI_Receive(&hspi1, &id_bme680, 1, 100);
//  deselect_sensors();
//  if(id_bme680 != 97){
//      for(;;);
//  }

    command[0] = 0x62;
    //command[0] = 0x62 << 1 | 0x00;
    command[1] = 0x04;
    select_sensor(MAG);
    /* Perform the transaction */
    HAL_SPI_TransmitReceive(&hspi1, command, received, 2, 100);
    deselect_sensors();
    HAL_Delay(100);
    command[0] = 0x80 | 0x4F;
    //command[0] = 0x4F << 1 | 0x01;
    command[1] = 0x00;
    select_sensor(MAG);
    /* Perform the transaction */
    HAL_SPI_TransmitReceive(&hspi1, command, received, 2, 100);
    deselect_sensors();
//command[0] = 0x60 << 1 | 0x01;
    command[0] = 0x80 | 0x60;
    command[1] = 0x00;
    select_sensor(MAG);
    /* Perform the transaction */
    HAL_SPI_TransmitReceive(&hspi1, command, received, 2, 100);
    deselect_sensors();

    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        return HAL_ERROR;
    }
// uint8_t id_bme680 = 80 | (1 << 7);
//   select_sensor(BME680);
//   HAL_SPI_Transmit(&hspi1, &id_bme680, 1, 100);
//   HAL_SPI_Receive(&hspi1, &id_bme680, 1, 100);
//   deselect_sensors();
//   if(id_bme680 != 97){
//       for(;;);
//   }
    bme680_init(&bme680);
    /* Set the temperature, pressure and humidity settings */
    bme680.tph_sett.os_hum = BME680_OS_2X;
    bme680.tph_sett.os_pres = BME680_OS_4X;
    bme680.tph_sett.os_temp = BME680_OS_8X;
    bme680.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    bme680.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    bme680.gas_sett.heatr_temp = 320; /* degree Celsius */
    bme680.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    bme680.power_mode = BME680_FORCED_MODE;

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    bme680_set_sensor_settings(set_required_settings, &bme680);

    /* Set the power mode */
    bme680_set_sensor_mode(&bme680);
    uint8_t lol = 255;
    uint8_t lol1 = 255;
    /* Get the total measurement duration so as to sleep or wait till the
     * measurement is complete */
    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &bme680);

    struct bme680_field_data data;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    // initialize HQ
    HQ_Handle hqHandle;
    hqHandle.uart = &huart4;
    HQ_Init(&hqHandle);

    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        uint32_t now = 0, then = 0;

        for (;;) { // Just to fuel the debate if this is better than while(1)
            // while(1) is easier to understand

            // TODO process return value of HQ_Loop
            HQ_Loop();

            now = HAL_GetTick();

            if (now - then >= 2000) {
                mcp795_read_time(&rtc, &rtc_time);
                printf("Year: 20%02u\r\nMonth: %s\r\nDate: %u\r\nDay: %s\r\nTime: %02u:%02u:%02u:%02u\r\n", rtc_time.year, month(rtc_time.month),
                        rtc_time.date, day_of_the_week(rtc_time.days), rtc_time.hours, rtc_time.minutes, rtc_time.seconds, rtc_time.milliseconds);
                printf("VDDA = %5.3f V Vref = %5.3f V (raw = %d) Temp = %4.2f °C (raw = %d)\r\n", vdda, vref, vref_avg, temp, temp_avg);
                printf("V_BAT = %5.3f V V_3.3 = %5.3f V V_4 = %5.3f V  I_4V = %5.3f A I_3.3V = %5.3f \r\n", bat_v, v3_3, v4, i4, i3_3);
                adt7310_read_reg(&adt7310_in, 0x00, 1, &lol);

                if (!(lol & 8)) {
                    printf("TEMP_IN: %wf °C\r\n", adt7310_read_float(&adt7310_in));
                }
                adt7310_read_reg(&adt7310_in, 0x00, 1, &lol1);
                if (!(lol1 & 8)) {
                    printf("TEMP_OUT: %f °C\r\n", adt7310_read_float(&adt7310_out));
                }
                bme680_get_sensor_data(&data, &bme680);
                printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f, data.pressure / 100.0f, data.humidity / 1000.0f);
                /* Avoid using measurements from an unstable heating setup */
                if (data.status & BME680_GASM_VALID_MSK) {
                    printf(", G: %ld ohms", data.gas_resistance);
                }
                printf("\r\n");
                printf("\r\n");

                /* Trigger the next measurement if you would like to read data out continuously */
                if (bme680.power_mode == BME680_FORCED_MODE) {
                    bme680_set_sensor_mode(&bme680);
                }
                //crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)CRC_TEST_STRING, strlen(CRC_TEST_STRING));
                //printf("String '%s' - (MPEG-2 CRC) => 0x%08lu!!!\r\nString '%s' - (BZIP2 CRC) => 0x%08lu!!!\r\n", CRC_TEST_STRING, crc, CRC_TEST_STRING, ~crc);
                then = now;
            }
        }
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = { 0 };

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 5;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T15_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
    hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

    /* USER CODE BEGIN TIM7_Init 0 */

    /* USER CODE END TIM7_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    /* USER CODE BEGIN TIM7_Init 1 */

    /* USER CODE END TIM7_Init 1 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 64000;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 500;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM7_Init 2 */

    /* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

    /* USER CODE BEGIN TIM14_Init 0 */

    /* USER CODE END TIM14_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = { 0 };

    /* USER CODE BEGIN TIM14_Init 1 */

    /* USER CODE END TIM14_Init 1 */
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 64000;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 1000;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim14) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM14_Init 2 */

    /* USER CODE END TIM14_Init 2 */

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void) {

    /* USER CODE BEGIN TIM15_Init 0 */

    /* USER CODE END TIM15_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    /* USER CODE BEGIN TIM15_Init 1 */

    /* USER CODE END TIM15_Init 1 */
    htim15.Instance = TIM15;
    htim15.Init.Prescaler = 639;
    htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim15.Init.Period = 999;
    htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = 0;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim15) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM15_Init 2 */

    /* USER CODE END TIM15_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

    /* USER CODE BEGIN TIM16_Init 0 */

    /* USER CODE END TIM16_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = { 0 };
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

    /* USER CODE BEGIN TIM16_Init 1 */

    /* USER CODE END TIM16_Init 1 */
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 64000;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 5000;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim16) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM16_Init 2 */

    /* USER CODE END TIM16_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

    /* USER CODE BEGIN TIM17_Init 0 */

    /* USER CODE END TIM17_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = { 0 };
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

    /* USER CODE BEGIN TIM17_Init 1 */

    /* USER CODE END TIM17_Init 1 */
    htim17.Instance = TIM17;
    htim17.Init.Prescaler = 64000;
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.Period = 5000;
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.RepetitionCounter = 0;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim17) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_OC_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM17_Init 2 */

    /* USER CODE END TIM17_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART4_UART_Init(void) {

    /* USER CODE BEGIN USART4_Init 0 */

    /* USER CODE END USART4_Init 0 */

    /* USER CODE BEGIN USART4_Init 1 */

    /* USER CODE END USART4_Init 1 */
    huart4.Instance = USART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART4_Init 2 */

    /* USER CODE END USART4_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel2_3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, LED_0_Pin | S0_SENS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, ROPE_CUT_Pin | BUZZER_Pin | MUX_EN_1_Pin | MUX_EN_0_Pin | S1_SENS_Pin | S2_SENS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, S1_CAM_Pin | S0_CAM_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : LED_0_Pin S0_SENS_Pin */
    GPIO_InitStruct.Pin = LED_0_Pin | S0_SENS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : ROPE_CUT_Pin BUZZER_Pin MUX_EN_1_Pin MUX_EN_0_Pin
     S1_SENS_Pin S2_SENS_Pin */
    GPIO_InitStruct.Pin = ROPE_CUT_Pin | BUZZER_Pin | MUX_EN_1_Pin | MUX_EN_0_Pin | S1_SENS_Pin | S2_SENS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : S1_CAM_Pin S0_CAM_Pin */
    GPIO_InitStruct.Pin = S1_CAM_Pin | S0_CAM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : CAM_HB_0_Pin */
    GPIO_InitStruct.Pin = CAM_HB_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAM_HB_0_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : CAM_HB_1_Pin */
    GPIO_InitStruct.Pin = CAM_HB_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAM_HB_1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// Process half a buffer full of data
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

    // Core cycle, runs every 1s
//	if (htim == &htim3)
//	{
//		uint16_t tickstart = HAL_GetTick();
//
//		// Get RTC time (date must be read as well for proper updating or something)
//		HAL_RTC_GetTime(&hrtc, &Current_Time, RTC_FORMAT_BIN);
//		HAL_RTC_GetDate(&hrtc, &Current_Date, RTC_FORMAT_BIN);
//
//
//		// Get all sensor readings
//		Get_BME280_in_all_readings();
//		Get_BME280_ex_all_readings();
//		Get_MPU6050_all_readings();
//		Get_SI1145_all_readings();
//
//
//		if(Sensors.BME280_External.Altitude > Altitude_max)
//			Altitude_max = Sensors.BME280_External.Altitude;
//
//		// Check if probe has fallen below altitude limit for going to sleep mode
//		if((Altitude_max > LOWER_ALTITUDE_LIMIT+200) && (Sensors.BME280_External.Altitude < LOWER_ALTITUDE_LIMIT))
//		{
//			HAL_TIM_Base_Stop_IT(&htim3);
//			HAL_TIM_Base_Start_IT(&htim10);
//			Is_asleep = 1;
//		}
//
//
//		// Max altitude rope cut condition
//		if((Sensors.BME280_External.Altitude > UPPER_ALTITUDE_LIMIT) && (Rope_cut_status != ROPE_CUTTING))
//		{
//			Rope_cut_status = ROPE_CUTTING;
//			Rope_cut_delay = ROPE_CUT_TIME;
//		}
//
//
//		// Rope cut control
//		if(Rope_cut_status == ROPE_CUTTING)
//		{
//			if(Rope_cut_delay > 0)
//			{
//				Rope_cut_delay--;
//				HAL_GPIO_WritePin(ROPE_CUT_EN_GPIO_Port, ROPE_CUT_EN_Pin, GPIO_PIN_SET);
//			}
//			else
//			{
//				HAL_GPIO_WritePin(ROPE_CUT_EN_GPIO_Port, ROPE_CUT_EN_Pin, GPIO_PIN_RESET);
//				Rope_cut_status = ROPE_CUT_DONE;
//			}
//		}
//
//
//		// Prepare data string that will be sent to COM
//		memset(Data_to_send, 0, sizeof(Data_to_send));
//		Data_to_send[0] = ',';
//		if(Sensors.BME280_Internal.Status == HAL_OK)
//		{
//			char TempStr[20];
//			sprintf(TempStr, "%.2f", Sensors.BME280_Internal.Temperature);
//			strcat(Data_to_send, TempStr);
//		}
//		else
//			sprintf(Data_to_send, "--");
//
//
//		if(Sensors.BME280_External.Status == HAL_OK)
//		{
//			char TempStr[20];
//			sprintf(TempStr, ",%.2f,%d", Sensors.BME280_External.Temperature, (int16_t)Sensors.BME280_External.Altitude);
//			strcat(Data_to_send, TempStr);
//		}
//		else
//			strcat(Data_to_send, ",--,--");
//
//
//		if(Sensors.SI1145.Status == HAL_OK)
//		{
//			char TempStr[10];
//			sprintf(TempStr, ",%.2f", Sensors.SI1145.UV);
//			strcat(Data_to_send, TempStr);
//		}
//		else
//			strcat(Data_to_send, ",--");
//
//
//		// Write to file and sync current file addition with physical SD card
//		if((sd_mounted == 1) && (sd_status == FR_OK))
//		{
//			sd_error_cnt = 0;
//			f_printf(&fil, "| %2u | %2u | %2u | %2u |", Current_Date.Date, Current_Time.Hours, Current_Time.Minutes, Current_Time.Seconds);
//
//
//			if(Sensors.BME280_Internal.Status == HAL_OK)
//			{
//				char TempStr[100];
//				sprintf(TempStr, " %8.3f | %8.3f | %8.3f | %8.1f |", Sensors.BME280_Internal.Temperature, Sensors.BME280_Internal.Pressure, Sensors.BME280_Internal.Humidity, Sensors.BME280_Internal.Altitude);
//				f_printf(&fil, TempStr);
//			}
//			else
//				for(uint8_t i = 0; i < 3; i++)
//					f_printf(&fil, "    N/A    |");
//
//
//			if(Sensors.BME280_External.Status == HAL_OK)
//			{
//				char TempStr[100];
//				sprintf(TempStr, " %8.3f | %8.3f | %8.3f | %8.1f |", Sensors.BME280_External.Temperature, Sensors.BME280_External.Pressure, Sensors.BME280_External.Humidity, Sensors.BME280_External.Altitude);
//				f_printf(&fil, TempStr);
//			}
//			else
//				for(uint8_t i = 0; i < 3; i++)
//					f_printf(&fil, "    N/A    |");
//
//
//			if(Sensors.MPU650.Status == HAL_OK)
//			{
//				char TempStr[100];
//				sprintf(TempStr, " %9.3f | %9.3f | %9.3f |", Sensors.MPU650.Accel_X, Sensors.MPU650.Accel_Y, Sensors.MPU650.Accel_Z);
//				f_printf(&fil, TempStr);
//				sprintf(TempStr, " %9.3f | %9.3f | %9.3f | %9.3f |", Sensors.MPU650.Gyro_X, Sensors.MPU650.Gyro_Y, Sensors.MPU650.Gyro_Z, Sensors.MPU650.Temperature);
//				f_printf(&fil, TempStr);
//			}
//			else
//				for(uint8_t i = 0; i < 7; i++)
//					f_printf(&fil, "      N/A     |");
//
//
//			if(Sensors.SI1145.Status == HAL_OK)
//			{
//				char TempStr[100];
//				sprintf(TempStr, " %6d | %6d | %6.2f |", Sensors.SI1145.VIS, Sensors.SI1145.IR, Sensors.SI1145.UV);
//				f_printf(&fil, TempStr);
//			}
//			else
//				for(uint8_t i = 0; i < 3; i++)
//					f_printf(&fil, "  N/A   |");
//
//
//			f_printf(&fil, "\n");
//			sd_status = f_sync(&fil);
//		}
//
//
//		// If an SD card error has occured, attempt to remount card and write to a new file
//		// After too many attempts the card is considered inoperable
//		if((sd_status != FR_OK) && (sd_error_cnt <= SD_MAX_RETRIES))
//		{
//			sd_status = f_mount(NULL, "", 1);
//			sd_status = f_mount(&fs, "", 1);
//			if(sd_status == FR_OK)
//			{
//				sd_error_cnt = 0;
//				sd_mounted = 1;
//				sd_file_cnt++;
//
//				char TempStr[12];
//				sprintf(TempStr, "Data%d.txt", sd_file_cnt);
//				sd_status = f_open(&fil, TempStr, FA_OPEN_ALWAYS | FA_WRITE);
//			}
//			else
//				sd_error_cnt++;
//
//			if(sd_error_cnt > SD_MAX_RETRIES)
//			{
//				UART1_TxBuf[0] = 0xD0;
//				UART1_TxBuf[1] = sd_status;
//				HAL_UART_Transmit_IT(&huart1, UART1_TxBuf, 2);
//				sd_mounted = 0;
//			}
//		}
//
//
//		// Cycle runtime logging
//		uint16_t Cycle_runtime = HAL_GetTick() - tickstart;
//		if(Cycle_runtime > Cycle_runtime_max)
//			Cycle_runtime_max = Cycle_runtime;
//		if(Cycle_runtime < Cycle_runtime_min)
//			Cycle_runtime_min = Cycle_runtime;
//		Cycle_runtime_last = Cycle_runtime;
//	}

    // LED flasher
    if (htim == &htim7) {
        // Status display for subsystems
//		  if((sd_mounted == 1) && (sd_status == FR_OK))
//			  HAL_GPIO_TogglePin(USER_LED1_GPIO_Port, USER_LED1_Pin);
//		  else
//			  HAL_GPIO_WritePin(USER_LED1_GPIO_Port, USER_LED1_Pin, GPIO_PIN_RESET);
//
//		  if(Sensors.BME280_Internal.Status == HAL_OK)
//			  HAL_GPIO_TogglePin(USER_LED4_GPIO_Port, USER_LED4_Pin);
//		  else
//			  HAL_GPIO_WritePin(USER_LED4_GPIO_Port, USER_LED4_Pin, GPIO_PIN_RESET);
//
//		  if(Sensors.BME280_External.Status == HAL_OK)
//			  HAL_GPIO_TogglePin(USER_LED2_GPIO_Port, USER_LED2_Pin);
//		  else
//			  HAL_GPIO_WritePin(USER_LED2_GPIO_Port, USER_LED2_Pin, GPIO_PIN_RESET);
//
//		  if(Sensors.MPU650.Status == HAL_OK)
//			  HAL_GPIO_TogglePin(USER_LED5_GPIO_Port, USER_LED5_Pin);
//		  else
//			  HAL_GPIO_WritePin(USER_LED5_GPIO_Port, USER_LED5_Pin, GPIO_PIN_RESET);
//
//		  if(Sensors.SI1145.Status == HAL_OK)
//			  HAL_GPIO_TogglePin(USER_LED3_GPIO_Port, USER_LED3_Pin);
//		  else
//			  HAL_GPIO_WritePin(USER_LED3_GPIO_Port, USER_LED3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
    }

    // Camera_1 recording restart
    if (htim == CAM_1.TIM) {
//		CAM_1.REC_STATUS = CAM_NREC;
//
//		if(CAM_1.Startup_delay > 0)
//			CAM_1.Startup_delay--;
//
//		if(CAM_1.Do_Restart)
//			CAM_START(&CAM_1);
    }

    // Camera_2 recording restart
    if (htim == CAM_2.TIM) {
//		CAM_2.REC_STATUS = CAM_NREC;
//
//		if(CAM_2.Startup_delay > 0)
//			CAM_2.Startup_delay--;
//
//		if(CAM_2.Do_Restart)
//			CAM_START(&CAM_2);
    }

    // Buzzer control
    if (htim == &htim14) {
        if ((rtc_time.seconds == 30) || (rtc_time.seconds == 0))
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
}

void process_adc_buffer(uint16_t *buffer) {

    uint32_t sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0, sum5 = 0;
    for (int i = 0; i < ADC_SAMPLES; ++i) {
        sum1 += buffer[i * 5];
        sum2 += buffer[1 + i * 5];
        sum3 += buffer[2 + i * 5];
        sum4 += buffer[3 + i * 5];
        sum5 += buffer[4 + i * 5];
    }

    temp_avg = sum1 / ADC_SAMPLES;
    vref_avg = sum2 / ADC_SAMPLES;
    bat_v = sum3 / ADC_SAMPLES;
    v4 = sum4 / ADC_SAMPLES * 0.01;
    v3_3 = sum5 / ADC_SAMPLES * 0.01;

    // VDDA can be calculated based on the measured vref and the calibration data
    vdda = (float) VREFINT_CAL_VREF * (float) *VREFINT_CAL_ADDR / vref_avg / 1000;

    // Knowing vdda and the resolution of adc - the actual voltage can be calculated
    vref = (float) vdda / ADC_RESOLUTION * vref_avg;

    temp = (float) (ta * (float) (sum1 / ADC_SAMPLES) + tb);

    //
    v3_3 = (vdda * v3_3) / ADC_RESOLUTION;
    i3_3 = v3_3 / 0.01;
    //
    v4 = (vdda * v4) / ADC_RESOLUTION;
    i4 = v4 / 0.01;
    //
    bat_v = (((vdda * bat_v) / ADC_RESOLUTION) * (12000 + 22000)) / 12000;
}

void calculate_calibration() {
    float x1 = (float) *TEMPSENSOR_CAL1_ADDR;
    float x2 = (float) *TEMPSENSOR_CAL2_ADDR;
    float y1 = (float) TEMPSENSOR_CAL1_TEMP;
    float y2 = (float) TEMPSENSOR_CAL2_TEMP;

    // Simple linear equation y = ax + b based on two points
    ta = (float) ((y2 - y1) / (x2 - x1));
    tb = (float) ((x2 * y1 - x1 * y2) / (x2 - x1));
}

//XXX why is this needed ????
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    process_adc_buffer(&adc_buffer[0]);
}

//XXX why do it in two steps ???
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    process_adc_buffer(&adc_buffer[ADC_SAMPLES * 5]);
}

void select_sensor(uint8_t sensor) {
    if (sensor == RTC_MOD) {
        HAL_GPIO_WritePin(S0_SENS_GPIO_Port, S0_SENS_Pin, RESET);
        HAL_GPIO_WritePin(S1_SENS_GPIO_Port, S1_SENS_Pin, RESET);
        HAL_GPIO_WritePin(S2_SENS_GPIO_Port, S2_SENS_Pin, RESET);
        HAL_GPIO_WritePin(MUX_EN_0_GPIO_Port, MUX_EN_0_Pin, SET);
    } else if (sensor == IMU) {
        HAL_GPIO_WritePin(S0_SENS_GPIO_Port, S0_SENS_Pin, SET);
        HAL_GPIO_WritePin(S1_SENS_GPIO_Port, S1_SENS_Pin, RESET);
        HAL_GPIO_WritePin(S2_SENS_GPIO_Port, S2_SENS_Pin, RESET);
        HAL_GPIO_WritePin(MUX_EN_0_GPIO_Port, MUX_EN_0_Pin, SET);
    } else if (sensor == BME680) {
        HAL_GPIO_WritePin(S0_SENS_GPIO_Port, S0_SENS_Pin, RESET);
        HAL_GPIO_WritePin(S1_SENS_GPIO_Port, S1_SENS_Pin, SET);
        HAL_GPIO_WritePin(S2_SENS_GPIO_Port, S2_SENS_Pin, RESET);
        HAL_GPIO_WritePin(MUX_EN_0_GPIO_Port, MUX_EN_0_Pin, SET);
    } else if (sensor == TEMP_OUT) {
        HAL_GPIO_WritePin(S0_SENS_GPIO_Port, S0_SENS_Pin, SET);
        HAL_GPIO_WritePin(S1_SENS_GPIO_Port, S1_SENS_Pin, SET);
        HAL_GPIO_WritePin(S2_SENS_GPIO_Port, S2_SENS_Pin, RESET);
        HAL_GPIO_WritePin(MUX_EN_0_GPIO_Port, MUX_EN_0_Pin, SET);
    } else if (sensor == TEMP_IN) {
        HAL_GPIO_WritePin(S0_SENS_GPIO_Port, S0_SENS_Pin, RESET);
        HAL_GPIO_WritePin(S1_SENS_GPIO_Port, S1_SENS_Pin, RESET);
        HAL_GPIO_WritePin(S2_SENS_GPIO_Port, S2_SENS_Pin, SET);
        HAL_GPIO_WritePin(MUX_EN_0_GPIO_Port, MUX_EN_0_Pin, SET);
    } else if (sensor == SD_CARD) {
        HAL_GPIO_WritePin(S0_SENS_GPIO_Port, S0_SENS_Pin, SET);
        HAL_GPIO_WritePin(S1_SENS_GPIO_Port, S1_SENS_Pin, RESET);
        HAL_GPIO_WritePin(S2_SENS_GPIO_Port, S2_SENS_Pin, SET);
        HAL_GPIO_WritePin(MUX_EN_0_GPIO_Port, MUX_EN_0_Pin, SET);
    } else if (sensor == MAG) {
        HAL_GPIO_WritePin(S0_SENS_GPIO_Port, S0_SENS_Pin, RESET);
        HAL_GPIO_WritePin(S1_SENS_GPIO_Port, S1_SENS_Pin, SET);
        HAL_GPIO_WritePin(S2_SENS_GPIO_Port, S2_SENS_Pin, SET);
        HAL_GPIO_WritePin(MUX_EN_0_GPIO_Port, MUX_EN_0_Pin, SET);
    } else {
        printf("Selected sensor with this ID doesn't not exists: %02x", sensor);
    }
}
void deselect_sensors(void) {
    HAL_GPIO_WritePin(S0_SENS_GPIO_Port, S0_SENS_Pin, RESET);
    HAL_GPIO_WritePin(S1_SENS_GPIO_Port, S1_SENS_Pin, RESET);
    HAL_GPIO_WritePin(S2_SENS_GPIO_Port, S2_SENS_Pin, RESET);
    HAL_GPIO_WritePin(MUX_EN_0_GPIO_Port, MUX_EN_0_Pin, RESET);
}
void select_camera_port(uint8_t cam_port) {
    if (cam_port == CAM0_ON) {
        HAL_GPIO_WritePin(S0_CAM_GPIO_Port, S0_CAM_Pin, RESET);
        HAL_GPIO_WritePin(S1_CAM_GPIO_Port, S1_CAM_Pin, RESET);
        HAL_GPIO_WritePin(MUX_EN_1_GPIO_Port, MUX_EN_1_Pin, SET);
    } else if (cam_port == CAM0_REC) {
        HAL_GPIO_WritePin(S0_CAM_GPIO_Port, S0_CAM_Pin, SET);
        HAL_GPIO_WritePin(S1_CAM_GPIO_Port, S1_CAM_Pin, RESET);
        HAL_GPIO_WritePin(MUX_EN_1_GPIO_Port, MUX_EN_1_Pin, SET);
    } else if (cam_port == CAM1_ON) {
        HAL_GPIO_WritePin(S0_CAM_GPIO_Port, S0_CAM_Pin, RESET);
        HAL_GPIO_WritePin(S1_CAM_GPIO_Port, S1_CAM_Pin, SET);
        HAL_GPIO_WritePin(MUX_EN_1_GPIO_Port, MUX_EN_1_Pin, SET);
    } else if (cam_port == CAM1_REC) {
        HAL_GPIO_WritePin(S0_CAM_GPIO_Port, S0_CAM_Pin, SET);
        HAL_GPIO_WritePin(S1_CAM_GPIO_Port, S1_CAM_Pin, SET);
        HAL_GPIO_WritePin(MUX_EN_1_GPIO_Port, MUX_EN_1_Pin, SET);
    } else {
        printf("Selected camera port with this ID doesn't not exists: %02x", cam_port);
    }
}
void deselect_camera_port(void) {
    HAL_GPIO_WritePin(S0_CAM_GPIO_Port, S0_CAM_Pin, RESET);
    HAL_GPIO_WritePin(S1_CAM_GPIO_Port, S1_CAM_Pin, RESET);
    HAL_GPIO_WritePin(MUX_EN_1_GPIO_Port, MUX_EN_1_Pin, RESET);
}
/* Returns FR_status after success or last failed step */
uint8_t Mount_open_SD_Card() {
    uint8_t sd_mounting_status;

    /* Mount SD Card */
    sd_mounting_status = f_mount(&fs, "", 1);
    if (sd_mounting_status != FR_OK)
        return sd_mounting_status;

    /* Open file to write */
    sd_mounting_status = f_open(&fil, "Data0.txt", FA_OPEN_ALWAYS | FA_WRITE);
    if (sd_mounting_status != FR_OK)
        return sd_mounting_status;

    /* Check freeSpace space */
    sd_mounting_status = f_getfree("", &fre_clust, &pfs);
    if (sd_mounting_status != FR_OK)
        return sd_mounting_status;
    totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
    freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);

    /* free space is less than 1kb */
    if (freeSpace < 1) {
        sd_mounting_status = 0xDF;
        return sd_mounting_status;
    }

    /* Writing header text */
    f_printf(&fil, "STM32 SD Card data logger\n");
    f_printf(&fil, "RTC time and sensor readings (SI VIS and SI IR are raw values)\n");
    f_printf(&fil, "SD-CARD-MEMORY = %d \n SD-CARD-FREE-MEMORY = %d\n", totalSpace, freeSpace);
    f_printf(&fil, "| Dt | Hr | Mn | Sc |");
    f_printf(&fil, " BME_in T | BME_in P | BME_in H | BME_in A |");
    f_printf(&fil, " BME_ex T | BME_ex P | BME_ex H | BME_in A |");
    f_printf(&fil, " MPU Acc_X | MPU Acc_Y | MPU Acc_Z | MPU Gyr_X | MPU Gyr_Y | MPU Gyr_Z | MPU Tmpr  |");
    f_printf(&fil, " SI VIS | SI IR  | SI UV  |\n");
    sd_mounting_status = f_sync(&fil);
    return sd_mounting_status;
}

void ON_CAM(CAM_struct *CAM) {
    // Checks if CAM is OFF
    if (CAM_OFF == CAM->CAM_STATUS) {
        select_camera_port(CAM->CAM_ON_OFF);
        HAL_Delay(3000);
        deselect_camera_port();
    }
    // Start heartbeat monitoring
    CAM->Do_EXTI = 1;
}
void START_CAM(CAM_struct *CAM) {
    // Checks if CAM is ON and not recording
    if ((CAM->REC_STATUS != CAM_RECORDING) && (CAM_ON == CAM->CAM_STATUS)) {
        select_camera_port(CAM->REC_CAM);
        HAL_Delay(100);
        deselect_camera_port();
    }

    //ON_CAM(CAM);

    // Start filming restart routine
    CAM->Do_Restart = 1;
}

//void STOP_CAM(CAM_struct *CAM){
//	// Checks if CAM is recording
//	if(CAM->REC_STATUS == CAM_REC){
//		HAL_GPIO_WritePin(CAM->OK_PORT, CAM->OK_PIN, GPIO_PIN_RESET);
//		HAL_Delay(30);
//		HAL_GPIO_WritePin(CAM->OK_PORT, CAM->OK_PIN, GPIO_PIN_SET);
//		CAM->REC_STATUS = CAM_NREC;
//	}
//	// Stop filming restart routine
//	CAM->Do_Restart = 0;
//}
//
//void OFF_CAM(CAM_struct *CAM){
//	// checks if CAM is ON
//	if(HAL_GPIO_ReadPin(CAM->OK_PORT, CAM->OK_PIN)){
//		HAL_GPIO_WritePin(CAM->ON_PORT, CAM->ON_PIN, GPIO_PIN_RESET);
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(CAM->ON_PORT, CAM->ON_PIN, GPIO_PIN_SET);
//	}
//	// Stop heartbeat monitoring
//	CAM->Do_EXTI = 0;
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart4) {
        BOARDTRX_UART_RX_CB();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart4) {
        BOARDTRX_UART_RXERROR_CB();
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
