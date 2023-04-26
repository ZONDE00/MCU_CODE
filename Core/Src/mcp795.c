#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "mcp795.h"
#include <main.h>
#include <string.h>

int mcp795_init(mcp795_t *dev, SPI_HandleTypeDef *hspi, uint8_t sensor_cs){
	uint8_t id[9] = {0};
	dev->hspi = hspi;
	dev->sensor_cs = sensor_cs;
	if(mcp795_read_id(dev, id, 8) != HAL_OK){
		 return HAL_TIMEOUT;
	 }
	 if(strcmp((char *)id, RTC_ID) != 0){
		 return UNRECOGNIZED_ID;
	 }
	return 0;
}

int mcp795_read_status_register(mcp795_t *dev, uint8_t rtc_status){
	int status = 0;
	uint8_t command[2] = {0};
	uint8_t received[2] = {0};
	command[0] = SRREAD;
	command[1] = 0x00;
	select_sensor(dev->sensor_cs);
	status = HAL_SPI_TransmitReceive(dev->hspi, command, received, 2, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		return -1;
	}
	rtc_status = received[1];
	deselect_sensors();
	return status;
}

int mcp795_read_id(mcp795_t *dev, uint8_t *id, uint16_t len){
	int status = 0;
	uint8_t command[2] = {0};
	command[0] = IDREAD;
	command[1] = 0x00;
	select_sensor(dev->sensor_cs);
	status = HAL_SPI_Transmit(dev->hspi, command, 2, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	HAL_SPI_Receive(dev->hspi, id, len, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	deselect_sensors();
	return status;
}
int mcp795_write_id(mcp795_t *dev, uint8_t *id, uint16_t len){
	int status = 0;
	uint8_t command[2] = {0};
	uint8_t send_id[16] = {0};
	/* unclock ID area */
	mcp795_WR_unlock(dev);
	command[0] = IDWRITE;
	command[1] = 0x00;
	for(uint8_t i = 0; i < len; i++){
		send_id[i] = id[i];
	}
	select_sensor(dev->sensor_cs);
	status = HAL_SPI_Transmit(dev->hspi, command, 2, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	status = HAL_SPI_Transmit(dev->hspi, send_id, 8, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	command[1] = 0x08;
	status = HAL_SPI_Transmit(dev->hspi, command, 2, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	status = HAL_SPI_Transmit(dev->hspi, &send_id[8], 8, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	deselect_sensors();
	return status;
}

int mcp795_WR_unlock(mcp795_t *dev){
	int status = 0;
	uint8_t unlock[2] = {0};
	unlock[0] = EEWREN;
	select_sensor(dev->sensor_cs);
	HAL_SPI_Transmit(dev->hspi, unlock, 1, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	deselect_sensors();
	unlock[0] = UNLOCK;
	unlock[1] = 0x55;
	select_sensor(dev->sensor_cs);
	HAL_SPI_Transmit(dev->hspi, unlock, 2, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	deselect_sensors();
	unlock[1] = 0xAA;
	select_sensor(dev->sensor_cs);
	HAL_SPI_Transmit(dev->hspi, unlock, 2, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	deselect_sensors();
	return 0;
}

int mcp795_read_time(mcp795_t *dev, rtc_time_t *rtc_time){
	int status = 0;
	uint8_t command[10] = {0};
	uint8_t received[10] = {0};
	command[0] = READ;
	command[1] = 0x00;
	select_sensor(dev->sensor_cs);
	status = HAL_SPI_TransmitReceive(dev->hspi, command, received, 10, 100);
	deselect_sensors();
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		return HAL_TIMEOUT;
	}

	rtc_time->milliseconds = ((received[2] & 0xf0) >> 4) * 10 + (received[2] & 0x0f);
	rtc_time->seconds = ((received[3] & 0x70) >> 4) * 10 + (received[3] & 0x0f);
	rtc_time->minutes = ((received[4] & 0x70) >> 4) * 10 + (received[4] & 0x0f);
	rtc_time->hours = (((received[5] & 0x30) >> 4) * 10) + (received[5] & 0x0f);
	rtc_time->days =  received[6] & 0x07;
	rtc_time->date = (((received[7] & 0x30) >> 4) * 10) + (received[7] & 0x0f);
	rtc_time->month = (((received[8] & 0x10) >> 4) * 10) + (received[8] & 0x0f);
	rtc_time->year = ((received[9] & 0xf0) >> 4) * 10 + (received[9] & 0x0f);
	return status;
}
int mcp795_set_time(mcp795_t *dev, rtc_time_t *rtc_time){
	int status = 0;
	uint8_t command[2] = {0};
	uint8_t time[8] = {0};
	command[0] = READ;
	command[1] = 0x00;
	select_sensor(dev->sensor_cs);
	HAL_SPI_Transmit(dev->hspi, command, 2, 100);
	HAL_SPI_Receive(dev->hspi, time, 8, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	deselect_sensors();
	time[0] = ((rtc_time->milliseconds / 10) << 4) | rtc_time->milliseconds % 10;
	time[1] = (time[1] & 0x80 ) | (((rtc_time->seconds / 10) << 4) & 0x70) | rtc_time->seconds % 10;
	time[2] = (time[2] & 0x80 ) | (((rtc_time->minutes / 10) << 4) & 0x70) | rtc_time->minutes % 10;
	time[3] = 0 << 8 | 0 << 7 | ((rtc_time->hours / 10) << 4) | (rtc_time->hours % 10);
	time[4] = (time[4] & 0xf8 ) | (rtc_time->days & 0x07);
	time[5] = (time[5] & 0xc0 ) | ((rtc_time->date / 10) << 4) | (rtc_time->date % 10);
	time[6] = (time[6] & 0xe0 ) | ((rtc_time->month / 10) << 4) | (rtc_time->month % 10);
	time[7] = ((rtc_time->year / 10) << 4) | rtc_time->year % 10;
	command[0] = WRITE;
	command[1] = 0x00;
	select_sensor(dev->sensor_cs);
	HAL_SPI_Transmit(dev->hspi, command, 2, 100);
	HAL_SPI_Transmit(dev->hspi, time, 8, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		deselect_sensors();
		return HAL_TIMEOUT;
	}
	deselect_sensors();
	return status;
}
int mcp795_start_counting(mcp795_t *dev){
	int status = 0;
	uint8_t command[3] = {0};
	command[0] = WRITE;
	command[1] = 0x01;
	command[2] = 0b10000000;
	select_sensor(dev->sensor_cs);
	HAL_SPI_Transmit(dev->hspi, command, 3, 100);
	if (status != HAL_OK){
		/* Couldn't communicate with sensor */
		return HAL_TIMEOUT;
	}
	deselect_sensors();
	return status;
}

