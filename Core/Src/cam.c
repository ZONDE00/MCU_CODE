#include "cam.h"

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

void STOP_CAM(CAM_struct *CAM) {
	// Checks if CAM is recording
//	if(CAM->REC_STATUS == CAM_REC){
//		HAL_GPIO_WritePin(CAM->OK_PORT, CAM->OK_PIN, GPIO_PIN_RESET);
//		HAL_Delay(30);
//		HAL_GPIO_WritePin(CAM->OK_PORT, CAM->OK_PIN, GPIO_PIN_SET);
//		CAM->REC_STATUS = CAM_NREC;
//	}
	// Stop filming restart routine
	CAM->Do_Restart = 0;
}

void OFF_CAM(CAM_struct *CAM) {
	// checks if CAM is ON
//	if(HAL_GPIO_ReadPin(CAM->OK_PORT, CAM->OK_PIN)){
//		HAL_GPIO_WritePin(CAM->ON_PORT, CAM->ON_PIN, GPIO_PIN_RESET);
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(CAM->ON_PORT, CAM->ON_PIN, GPIO_PIN_SET);
//	}
	// Stop heartbeat monitoring
	CAM->Do_EXTI = 0;
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
		printf("Selected camera port with this ID doesn't not exists: %02x",
				cam_port);
	}
}
void deselect_camera_port(void) {
	HAL_GPIO_WritePin(S0_CAM_GPIO_Port, S0_CAM_Pin, RESET);
	HAL_GPIO_WritePin(S1_CAM_GPIO_Port, S1_CAM_Pin, RESET);
	HAL_GPIO_WritePin(MUX_EN_1_GPIO_Port, MUX_EN_1_Pin, RESET);
}
