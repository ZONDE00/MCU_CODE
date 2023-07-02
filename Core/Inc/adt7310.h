#ifndef __ADT7310_H
#define __ADT7310_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   Device descriptor for ADT7310 sensors.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <main.h>
#include "stm32g0xx_hal.h"

typedef struct {
	SPI_HandleTypeDef *hspi; /**< SPI bus the sensor is connected to */
	uint8_t sensor_cs; /**< CS ID for MUX */
	bool initialized; /**< sensor status, true if sensor is initialized */
	bool high_res; /**< Sensor resolution, true if configured to 16 bit resolution */
	HAL_StatusTypeDef status; /**< Sensor status, true if still capable to communicate */
} adt7310_t;

/**
 * @name    ADT7310 configuration bits
 * @{
 */
#define ADT7310_CONF_FAULT_QUEUE_MASK  (0x03)
#define ADT7310_CONF_FAULT_QUEUE_SHIFT (0)
#define ADT7310_CONF_FAULT_QUEUE(x) (((x) << ADT7310_CONF_FAULT_QUEUE_SHIFT) & ADT7310_CONF_FAULT_QUEUE_MASK)
#define ADT7310_CONF_CT_POL_MASK  (0x04)
#define ADT7310_CONF_CT_POL_SHIFT (2)
#define ADT7310_CONF_CT_POL(x) (((x) << ADT7310_CONF_CT_POL_SHIFT) & ADT7310_CONF_CT_POL_MASK)
#define ADT7310_CONF_INT_POL_MASK  (0x08)
#define ADT7310_CONF_INT_POL_SHIFT (3)
#define ADT7310_CONF_INT_POL(x) (((x) << ADT7310_CONF_INT_POL_SHIFT) & ADT7310_CONF_INT_POL_MASK)
#define ADT7310_CONF_INTCT_MODE_MASK  (0x10)
#define ADT7310_CONF_INTCT_MODE_SHIFT (4)
#define ADT7310_CONF_INTCT_MODE(x) (((x) << ADT7310_CONF_INTCT_MODE_SHIFT) & ADT7310_CONF_INTCT_MODE_MASK)
#define ADT7310_CONF_OPERATION_MODE_MASK  (0x60)
#define ADT7310_CONF_OPERATION_MODE_SHIFT (5)
#define ADT7310_CONF_OPERATION_MODE(x) (((x) << ADT7310_CONF_OPERATION_MODE_SHIFT) & ADT7310_CONF_OPERATION_MODE_MASK)
#define ADT7310_CONF_RESOLUTION_MASK  (0x80)
#define ADT7310_CONF_RESOLUTION_SHIFT (7)
#define ADT7310_CONF_RESOLUTION(x) (((x) << ADT7310_CONF_RESOLUTION_SHIFT) & ADT7310_CONF_RESOLUTION_MASK)

/**
 * @brief   Continuous operation mode
 */
#define ADT7310_MODE_CONTINUOUS (ADT7310_CONF_OPERATION_MODE(0))
/**
 * @brief   One shot
 */
#define ADT7310_MODE_ONE_SHOT   (ADT7310_CONF_OPERATION_MODE(1))
/**
 * @brief   1 sample per second
 */
#define ADT7310_MODE_1SPS       (ADT7310_CONF_OPERATION_MODE(2))
/**
 * @brief   Shut down (powersave)
 */
#define ADT7310_MODE_SHUTDOWN   (ADT7310_CONF_OPERATION_MODE(3))
/** @} */

/**
 * @brief   Set configuration register of an ADT7310 sensor
 *
 * @param[in]  dev          pointer to sensor device descriptor
 * @param[in]  config       configuration byte, see macros in adt7310.h
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int adt7310_set_config(adt7310_t *dev, uint8_t config);

/**
 * @brief   Initialize the ADT7310 sensor driver.
 *
 * @note The SPI bus is expected to have been initialized when adt7310_init is called.
 *
 * @param[in]  dev          pointer to sensor device descriptor
 * @param[in]  spi          SPI bus the sensor is connected to
 * @param[in]  clk          SPI bus speed
 * @param[in]  cs           GPIO pin the chip select signal is connected to
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int adt7310_init(adt7310_t *dev, SPI_HandleTypeDef *hspi, uint8_t sensor_cs);

/**
 * @brief   Read raw temperature register value
 *
 * @note The three least-significant bits of the value register are used for
 *       flags if the sensor is configured for 13 bit mode.
 *
 * @param[in]  dev          pointer to sensor device descriptor
 *
 * @return                  raw sensor value on success
 * @return                  INT16_MIN on error
 */
int16_t adt7310_read_raw(const adt7310_t *dev);

/**
 * @brief   Read temperature value from sensor and convert to degrees Celsius.
 *
 * @param[in]  dev          pointer to sensor device descriptor
 *
 * @return                  floating point representation of temperature in degrees Celsius
 * @return                  NaN on errors
 */
float adt7310_read_float(const adt7310_t *dev);

HAL_StatusTypeDef adt7310_resetDevice(adt7310_t *dev);
int adt7310_resetspi(const adt7310_t *dev);
int adt7310_configspi(const adt7310_t *dev);
int adt7310_read_reg(const adt7310_t *dev, const uint8_t addr,
		const uint16_t len, uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif /* ADT7310_H */
/** @} */
