#ifndef __MCP795_H
#define __MCP795_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   Device descriptor for MCP795 RTC.
 */
typedef struct {
    SPI_HandleTypeDef *hspi; 	/**< SPI bus the sensor is connected to */
    uint8_t sensor_cs;			/**< CS ID for MUX */
} mcp795_t;


typedef enum{
	Unknown_day = 0,
	Monday = 1,
	Tuesday = 2,
	Wednesday = 3,
	Thursday = 4,
	Friday = 5,
	Saturday = 6,
	Sunday = 7
} day_of_the_week_t;

typedef enum{
	Unknown_month = 0,
	January = 1,
	February = 2,
	March = 3,
	April = 4,
	May = 5,
	June = 6,
	July = 7,
	August = 8,
	September = 9,
	October = 10,
	November = 11,
	December = 12
} month_t;

static inline const char *day_of_the_week(day_of_the_week_t day){
    static const char *week_day[] = {"Unknown_day","Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
    return week_day[day];
}
static inline const char *month(month_t month){
    static const char *months[] = {"Unknown_month","January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
    return months[month];
}

typedef struct {
	uint8_t year;
	uint8_t month;
	uint8_t date;
	day_of_the_week_t days;
	uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t milliseconds;
} rtc_time_t;

#define RTC_ID "Kristers"
//#define RTC_ID "Rodrigoo"
#define UNRECOGNIZED_ID 6U
/**
 * @name   MCP795 instruction set
 * @{
 */
#define EEREAD 		0b00000011 	// Read data from EE memory array beginning at selected address
#define EEWRITE 	0b00000010 	// Write data to EE memory array beginning at selected address
#define EEWRDI 		0b00000100 	// Reset the write enable latch (disable write operations)
#define EEWREN 		0b00000110 	// Set the write enable latch (enable write operations)
#define SRREAD 		0b00000101 	// Read STATUS register
#define SRWRITE  	0b00000001  // Write STATUS register
#define READ 		0b00010011 	// Read RTCC/SRAM array beginning at selected address
#define WRITE 		0b00010010 	// Write RTCC/SRAM data to memory array beginning at selected address
#define UNLOCK 		0b00010100 	// Unlock ID Locations
#define IDWRITE 	0b00110010 	// Write to the ID Locations
#define IDREAD 		0b00110011 	// Read the ID Locations
#define CLRWDT 		0b01000100 	// Clear Watchdog TImer
#define CLRRAM 		0b01010100  // Clear RAM Location to ‘0’

/**
 * @name   MCP795 register addresses
 * @{
 */
/* TIME AND CONFIGURATION REGISTERS */
#define HUNDREDTHS_OF_SECONDS 0X00
/**
 * @brief   Initialize the MCP795 sensor driver.
 *
 * @note The SPI bus is expected to have been initialized when mcp795_init is called.
 *
 * @param[in]  dev          pointer to sensor device descriptor
 * @param[in]  hspi         SPI bus the sensor is connected to
 * @param[in]  cs           GPIO pin the chip select signal is connected to
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int mcp795_init(mcp795_t *dev, SPI_HandleTypeDef *hspi, uint8_t sensor_cs);
int mcp795_read_id(mcp795_t *dev, uint8_t *id, uint16_t len);
int mcp795_write_id(mcp795_t *dev, uint8_t *id, uint16_t len);
int mcp795_WR_unlock(mcp795_t *dev);
/**
 * @brief   Set configuration register of an MCP795 sensor
 *
 * @param[in]  dev          pointer to sensor device descriptor
 * @param[in]  config       configuration byte
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int mcp795_set_config(mcp795_t *dev, uint8_t config);
int mcp795_read_status_register(mcp795_t *dev, uint8_t rtc_status);
int mcp795_read_time(mcp795_t *dev, rtc_time_t *rtc_time);
int mcp795_set_time(mcp795_t *dev, rtc_time_t *rtc_time);
int mcp795_start_counting(mcp795_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __MCP795_H */

