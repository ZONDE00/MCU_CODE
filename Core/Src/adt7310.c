#include "adt7310.h"

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
#define ADT7310_REG_VALUE_MASK_13BIT  (0xFFF8)

/** @brief Number of fractional bits in the raw readings */
#define ADT7310_VALUE_FRAC_16_BITS (7)
#define ADT7310_VALUE_FRAC_13_BITS (4)

/** @brief Scale factor for converting raw temperature readings to degrees - 16 bit
 * Celsius, floating point number */
#define ADT7310_TEMPERATURE_LSB_FLOAT_16_BIT (1.f/((float)((int)1 << ADT7310_VALUE_FRAC_16_BITS)))

/** @brief Scale factor for converting raw temperature readings to degrees - 13 bit
 * Celsius, floating point number */
#define ADT7310_TEMPERATURE_LSB_FLOAT_13_BIT (1.f/((float)((int)1 << ADT7310_VALUE_FRAC_13_BITS)))
/**
 * @brief Read a register from the sensor
 *
 * @param[in]  dev    device descriptor
 * @param[in]  addr   register address
 * @param[in]  len    register size
 * @param[out] buf    destination buffer
 *
 * @return            0 on success
 * @return            3 on communication errors
 */
int adt7310_read_reg(const adt7310_t *dev, const uint8_t addr,
		const uint16_t len, uint8_t *buf) {
	int status = 0;
	uint8_t command = ADT7310_CMD_READ | (addr << ADT7310_CMD_ADDR_SHIFT);
	/* Acquire exclusive access to the bus. */
	if (adt7310_configspi(dev) != 0) {
		return HAL_ERROR;
	}

	select_sensor(dev->sensor_cs);

	/* Perform the transaction */
	status = HAL_SPI_Transmit(dev->hspi, &command, 1, 100);
	status = HAL_SPI_Receive(dev->hspi, buf, len, 100);
	/* Release the bus. */
	deselect_sensors();

	if (adt7310_resetspi(dev) != 0) {
		return HAL_ERROR;
	}

	if (status != HAL_OK) {
		/* Couldn't communicate with sensor */
		return HAL_TIMEOUT;
	}
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
static int adt7310_write_reg(const adt7310_t *dev, const uint8_t addr,
		const uint16_t len, uint8_t *buf) {
	int status = 0;
	uint8_t command = ADT7310_CMD_WRITE | (addr << ADT7310_CMD_ADDR_SHIFT);
	/* Acquire exclusive access to the bus. */
	if (adt7310_configspi(dev) != 0) {
		return HAL_ERROR;
	}
	select_sensor(dev->sensor_cs);
	/* Perform the transaction */
	status = HAL_SPI_Transmit(dev->hspi, &command, 1, 11);
	status = HAL_SPI_Transmit(dev->hspi, buf, len, 11);
	/* Release the bus for other threads. */
	deselect_sensors();
	if (adt7310_resetspi(dev) != 0) {
		return HAL_ERROR;
	}
	if (status != HAL_OK) {
		/* SPI bus error */
		return -1;
	}
	return status;
}

HAL_StatusTypeDef adt7310_resetDevice(adt7310_t *dev){
	dev->hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
	dev->hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
	if (HAL_SPI_Init(dev->hspi) != HAL_OK) {
		return HAL_ERROR;
	}

	select_sensor(dev->sensor_cs);

	uint8_t tmpData[4];
	memset(tmpData, 255, 4);
	HAL_SPI_Transmit(dev->hspi, tmpData, 4, 100);

	deselect_sensors();

	return HAL_OK;
}

int adt7310_resetspi(const adt7310_t *dev) {
	//dummy commands
	uint8_t command = 0;
	uint8_t received = 255;
	uint8_t status = HAL_OK;

	dev->hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
	dev->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
	if (HAL_SPI_Init(dev->hspi) != HAL_OK) {
		return HAL_ERROR;
	}
	//setup correctly SPI [DUMMY COMMAND]
	status = HAL_SPI_TransmitReceive(dev->hspi, &command, &received, 1, 100);
	if (status != HAL_OK) {
		return HAL_TIMEOUT;
	}
	return HAL_OK;
}

int adt7310_configspi(const adt7310_t *dev) {
	//dummy commands
	uint8_t command = 0;
	uint8_t received = 255;
	uint8_t status = HAL_OK;
	dev->hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
	dev->hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
	if (HAL_SPI_Init(dev->hspi) != HAL_OK) {
		return HAL_ERROR;
	}
	//setup correctly SPI [DUMMY COMMAND]
	status = HAL_SPI_TransmitReceive(dev->hspi, &command, &received, 1, 100);
	if (status != HAL_OK) {
		return HAL_TIMEOUT;
	}
	return HAL_OK;
}

int adt7310_init(adt7310_t *dev, SPI_HandleTypeDef *hspi, uint8_t sensor_cs) {
	int status;
	uint8_t received = 255;
	/* write device descriptor */
	dev->hspi = hspi;
	dev->sensor_cs = sensor_cs;
	dev->initialized = false;
	dev->high_res = false;

	adt7310_resetDevice(dev);

	HAL_Delay(1);

	status = adt7310_read_reg(dev, ADT7310_REG_ID, 1, &received);

	if (status != HAL_OK) {
		/* Couldn't communicate with sensor */
		return HAL_TIMEOUT;
	}

	if ((received & ADT7310_REG_ID_MASK_MANUFACTURER_ID)
			!= ADT7310_EXPECTED_MANUF_ID) {
		/* Wrong part ID */
		return -2;
	}

	/* Set a configuration, go to shut down mode to save power until the sensor is needed. */
	if (adt7310_set_config(dev, ADT7310_MODE_SHUTDOWN) != 0) {
		/* communication error */
		return -3;
	}

	dev->initialized = true;
	return HAL_OK;
}

int adt7310_set_config(adt7310_t *dev, uint8_t config) {
	if (config & ADT7310_CONF_RESOLUTION_MASK) {
		dev->high_res = true;
	}
	return adt7310_write_reg(dev, ADT7310_REG_CONFIG, ADT7310_REG_SIZE_CONFIG,
			&config);
}

int16_t adt7310_read_raw(const adt7310_t *dev) {
	int status;
	int16_t raw;
	/* Read the temperature value register */
	status = adt7310_read_reg(dev, ADT7310_REG_VALUE, ADT7310_REG_SIZE_VALUE,
			(uint8_t*) &raw);
	raw = (raw >> 8) + (raw << 8);
	if (status < 0) {
		/* communication error */
		return INT16_MIN;
	}
	return raw;
}

float adt7310_read_float(const adt7310_t *dev) {
	int16_t raw = adt7310_read_raw(dev);
	float c = 0.0;
	if (raw == INT16_MIN) {
		/* cppcheck-suppress duplicateExpression
		 * (reason: we want to create a NaN here) */
		return (0.0f / 0.0f); /* return NaN */
	}
	if (!dev->high_res) {
		/* filter out the flag bits */
		raw &= ADT7310_REG_VALUE_MASK_13BIT;
		c = (raw >> 3) * ADT7310_TEMPERATURE_LSB_FLOAT_13_BIT;
	} else {
		c = (float) raw * ADT7310_TEMPERATURE_LSB_FLOAT_16_BIT;
	}
	return c;
}
