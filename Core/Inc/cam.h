#ifndef __CAM_H
#define __CAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "stm32g0xx_hal.h"

typedef struct CAM_struct {
	uint16_t CAM_HB_Pin;
	GPIO_TypeDef *CAM_HB_Port;
	uint8_t CAM_ON_OFF;
	uint8_t REC_CAM;
	uint8_t REC_STATUS;
	uint8_t Do_Restart;
	uint8_t Do_EXTI;
	uint8_t Startup_delay;
	uint8_t CAM_STATUS;
	HAL_StatusTypeDef status; /**< Sensor status, true if still capable to communicate */
} CAM_struct;

void deselect_camera_port(void);
void select_camera_port(uint8_t cam_port);
void START_CAM(CAM_struct *CAM);
void OFF_CAM(CAM_struct *CAM);
void STOP_CAM(CAM_struct *CAM);
void ON_CAM(CAM_struct *CAM);

#define CAM_NOK 0
#define CAM_OK 1
#define CAM_NREC 0
#define CAM_REC 1
#define CAM_STARTUP_TIME 6	// x*5 seconds
#define CAM_OFF 0
#define CAM_ON 1
#define CAM_RECORDING 2

#define MUX_EN_1_Pin GPIO_PIN_7
#define MUX_EN_1_GPIO_Port GPIOA
#define S1_CAM_Pin GPIO_PIN_0
#define S1_CAM_GPIO_Port GPIOB
#define S0_CAM_Pin GPIO_PIN_1
#define S0_CAM_GPIO_Port GPIOB
#define CAM_HB_0_Pin GPIO_PIN_15
#define CAM_HB_0_GPIO_Port GPIOA
#define CAM_HB_1_Pin GPIO_PIN_8
#define CAM_HB_1_GPIO_Port GPIOB

#define CAM_HB_0_Pin GPIO_PIN_15
#define CAM_HB_0_GPIO_Port GPIOA
#define CAM_HB_1_Pin GPIO_PIN_8
#define CAM_HB_1_GPIO_Port GPIOB

#define CAM0_ON 0U
#define CAM0_REC 1U
#define CAM1_ON 2U
#define CAM1_REC 3U

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __CAM_H */
