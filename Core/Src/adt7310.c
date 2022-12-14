#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "adt7310.h"
#include <main.h>
#include <string.h>

/* SPI command byte parameters */
#define ADT7310_CMD_READ       (0x40)
#define ADT7310_CMD_WRITE      (0x00)
#define ADT7310_CMD_ADDR_SHIFT (3)
#define ADT7310_CMD_CONTINUOUS (0x04)

/* ****************** *
 * Register addresses *
 * ****************** */
#define ADT7310_REG_STATUS (0x00)
#define ADT7310_REG_CONFIG (0x01)
#define ADT7310_REG_VALUE  (0x02)
#define ADT7310_REG_ID     (0x03)
#define ADT7310_REG_TCRIT  (0x04)
#define ADT7310_REG_THYST  (0x05)
#define ADT7310_REG_THIGH  (0x06)
#define ADT7310_REG_TLOW   (0x07)

/* ************** *
 * Register sizes *
 * ************** */
#define ADT7310_REG_SIZE_STATUS (1)
#define ADT7310_REG_SIZE_CONFIG (1)
#define ADT7310_REG_SIZE_VALUE  (2)
#define ADT7310_REG_SIZE_ID     (1)
#define ADT7310_REG_SIZE_TCRIT  (2)
#define ADT7310_REG_SIZE_THYST  (1)
#define ADT7310_REG_SIZE_THIGH  (2)
#define ADT7310_REG_SIZE_TLOW   (2)

/* Register bit masks */
/** @brief Manufacturer ID */
#define ADT7310_REG_ID_MASK_MANUFACTURER_ID  (0xF8)
/** @brief Silicon version */
#define ADT7310_REG_ID_MASK_SILICON_VERSION  (0x07)

/** @brief Expected manufacturer ID */
#define ADT7310_EXPECTED_MANUF_ID (0xC0)

/** @brief 13 bit temperature mask */
#define ADT7310_REG_VALUE_MASK_13BIT  (0xF8)

/** @brief Number of fractional bits in the raw readings */
#define ADT7310_VALUE_FRAC_BITS (7)

/** @brief Scale factor for converting raw temperature readings to degrees
 * Celsius, floating point number */
#define ADT7310_TEMPERATURE_LSB_FLOAT (1.f/((float)((int)1 << ADT7310_VALUE_FRAC_BITS)))

/**
 * @brief Read a register from the sensor
 *
 * @param[in]  dev    device descriptor
 * @param[in]  addr   register address
 * @param[in]  len    register size
 * @param[out] buf    destination buffer
 *
 * @return            0 on success
 * @return            -1 on communication errors
 */
static int adt7310_read_reg(const adt7310_t *dev, const uint8_t addr, const uint16_t len, uint8_t *buf){
    int status = 0;
    uint8_t command = ADT7310_CMD_READ | (addr << ADT7310_CMD_ADDR_SHIFT);
    /* Acquire exclusive access to the bus. */
    select_sensor(dev->sensor_cs);
    /* Perform the transaction */
    status = HAL_SPI_TransmitReceive(dev->hspi, &command, buf, len, 100);
    if (status != HAL_OK){
    	/* Couldn't communicate with sensor */
		return -1;
    }
    /* Release the bus for other threads. */
    deselect_sensors();

    return status;
}

/**
 * @brief Write a register value to the sensor
 *
 * @param[in]  dev    device descriptor
 * @param[in]  addr   register address
 * @param[in]  len    register size
 * @param[in]  buf    source buffer
 *
 * @return            0 on success
 * @return            -1 on communication errors
 */
static int adt7310_write_reg(const adt7310_t *dev, const uint8_t addr, const uint16_t len, uint8_t *buf){
    int status = 0;
    uint8_t command[len+1];
    memset(command, 0, (len+1)*sizeof(uint8_t));
    memcpy(&command[1], buf, len);
    command[0] = ADT7310_CMD_WRITE | (addr << ADT7310_CMD_ADDR_SHIFT);
    /* Acquire exclusive access to the bus. */
    select_sensor(dev->sensor_cs);
    /* Perform the transaction */
    status = HAL_SPI_Transmit(dev->hspi, command, len + 1, 100);
    if (status != HAL_OK) {
            /* SPI bus error */
            return -1;
    }
    /* Release the bus for other threads. */
    deselect_sensors();

    return status;
}

int adt7310_init(adt7310_t *dev, SPI_HandleTypeDef *hspi, uint8_t sensor_cs){
    int status;
    uint8_t reg = 0;
    /* write device descriptor */
    dev->hspi = hspi;
    dev->sensor_cs = sensor_cs;
    dev->initialized = false;
    dev->high_res = false;

    /* CS */
    select_sensor(dev->sensor_cs);

    /* Read ID register from device */
    status = HAL_SPI_TransmitReceive(dev->hspi, (uint8_t*)ADT7310_REG_ID, &reg, ADT7310_REG_SIZE_ID, 100);
    if (status != HAL_OK){
        /* SPI bus error */
        return -1;
    }
    if ((reg & ADT7310_REG_ID_MASK_MANUFACTURER_ID) != ADT7310_EXPECTED_MANUF_ID){
        /* Wrong part ID */
        return -2;
    }

    /* Set a configuration, go to shut down mode to save power until the sensor is needed. */
    if (adt7310_set_config(dev, ADT7310_MODE_SHUTDOWN) != 0){
        /* communication error */
        return -3;
    }

    dev->initialized = true;
    return 0;
}

int adt7310_set_config(adt7310_t *dev, uint8_t config){
	if (config & ADT7310_CONF_RESOLUTION_MASK){
        dev->high_res = true;
    }
    return adt7310_write_reg(dev, ADT7310_REG_CONFIG, ADT7310_REG_SIZE_CONFIG, &config);
}

int16_t adt7310_read_raw(const adt7310_t *dev){
    int status;
    int16_t raw;

    /* Read the temperature value register */
    status = adt7310_read_reg(dev, ADT7310_REG_VALUE, ADT7310_REG_SIZE_VALUE, (uint8_t*)&raw);
    if (status < 0) {
        /* communication error */
        return INT16_MIN;
    }
    return raw;
}

int32_t adt7310_read(const adt7310_t *dev){
    int16_t raw = adt7310_read_raw(dev);
    if (raw == INT16_MIN) {
        return INT32_MIN;
    }
    if (!dev->high_res) {
        /* filter out the flag bits */
        raw &= ADT7310_REG_VALUE_MASK_13BIT;
    }
    return ((((int32_t)raw) * 1000) >> ADT7310_VALUE_FRAC_BITS);
}

float adt7310_read_float(const adt7310_t *dev){
    int16_t raw = adt7310_read_raw(dev);
    if (raw == INT16_MIN){
        /* cppcheck-suppress duplicateExpression
         * (reason: we want to create a NaN here) */
        return (0.0f / 0.0f); /* return NaN */
    }
    if (!dev->high_res){
        /* filter out the flag bits */
        raw &= ADT7310_REG_VALUE_MASK_13BIT;
    }
    return (((float) raw) * ADT7310_TEMPERATURE_LSB_FLOAT);
}
