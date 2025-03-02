#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/types.h>

#include "bmp280-iio.h"

/**
 * Used as a sanity check during sensor initialization.
 * If we are really talking with a real BMP280 sensor, then reading from the
 * BMP280_ID_REG register will return us BMP280_ID.
 */
#define BMP280_ID 0x58
#define BMP280_ID_REG 0xD0

/**
 * Register addresses for sampling control and configuration.
 */
#define BMP280_CTRL_MEAS_REG_ADDRESS 0xf4
#define BMP280_CONFIG_REG_ADDRESS 0xf5

/**
 * Register addresses for temperature reading.
 */
#define BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS 0x88
#define BMP280_TEMP_RAW_REG_ADDRESS 0xfa

/**
 * Register addresses for pressure reading.
 */
#define BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS 0x8e
#define BMP280_PRESS_RAW_REG_ADDRESS 0xf7

/**
 * IIO channel macro for calibration values.
 * For triggered buffer reads, `scan_index` sets the position of this channel
 * data within the sample. `scan_type` tells that the channel data is unsigned,
 * takes up 16 bits without any padding, and follows the CPU's endianness.
 */
#define BMP280_CALIBR_CHANNNEL(channel_type, index, scan_idx, reg_address) { \
    .type = (channel_type),						\
    .indexed = 1,							\
    .channel = (index),						        \
    .address = (reg_address),						\
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
    .scan_index = (scan_idx),						\
    .scan_type = {							\
      .sign = 'u',							\
      .realbits = 16,							\
      .storagebits = 16,						\
      .shift = 0,							\
      .endianness = IIO_CPU,						\
    },									\
    .output = 0,							\
}

/**
 * IIO channels.
 * We make the following channels available:
 *     * Three temperature calibration values.
 *     * One raw temperature value.
 *     * One final, processed temperature value.
 *     * Nine pressure calibration values.
 *     * One raw pressure value.
 *     * One final, processed pressure value.
 * Within `/sys/bus/iio/devices/iio:deviceX/`, these will be:
 * `in_temp{0-3}_raw`, `in_temp_input`, `in_pressure{0-9}_raw`, and
 * `in_pressure_input`, respectively.
 */
static const struct iio_chan_spec bmp280_iio_channels[] = {
  // Temperature calibration values, refered to as dig_T1 to dig_T3 on the
  // datasheet. Corresponding sysfs files: `in_temp{0-2}_raw`.
  // Note: each calibration value is 16 bits, thus the address deltas.
  BMP280_CALIBR_CHANNNEL(IIO_TEMP, 0, 0,
			 BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS),
  BMP280_CALIBR_CHANNNEL(IIO_TEMP, 1, 1,
			 BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS + 2),
  BMP280_CALIBR_CHANNNEL(IIO_TEMP, 2, 2,
			 BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS + 4),
  // Raw temperature value, as directly read from the sensor.
  // Corresponding sysfs file: `in_temp3_raw`
  {
    .type = IIO_TEMP,
    .indexed = 1,
    .channel = 3,
    .address = BMP280_TEMP_RAW_REG_ADDRESS,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    .scan_index = 3,
    // Channel data is signed (2 complement), takes up 20 bits within a 32 bits
    // field, with the 4 LS bits being padding bits, and follows the host
    // CPU's endianness.
    .scan_type = {
      .sign = 's',
      .realbits = 20,
      .storagebits = 32,
      .shift = 4,
      .endianness = IIO_CPU,
    },
    .output = 0,
  },
  // Final "processed" temperature value.
  // Corresponding sysfs file: `in_temp_processed`
  {
    .type = IIO_TEMP,
    .indexed = 0,
    .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
    .scan_index = 4,
    // Channel data is signed (2 complement), takes up 32 bits,
    // and follows the host CPU's endianness.
    .scan_type = {
      .sign = 's',
      .realbits = 32,
      .storagebits = 32,
      .shift = 0,
      .endianness = IIO_CPU,
    },
    .output = 0,
  },
  // Pressure calibration values, refered to as dig_P1 to dig_P9 on the
  // datasheet. Corresponding sysfs files: `in_pressure{0-8}_raw`.
  // Note: each calibration value is 16 bits, thus the address deltas.
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 0, 5,
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 1, 6,
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 2),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 2, 7,
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 4),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 3, 8,
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 6),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 4, 9,
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 8),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 5, 10,
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 10),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 6, 11,
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 12),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 7, 12,
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 14),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 8, 13,
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 16),
  // Raw pressure value, as directly read from the sensor.
  // Corresponding sysfs file: `in_pressure9_raw`
  {
    .type = IIO_PRESSURE,
    .indexed = 1,
    .channel = 9,
    .address = BMP280_PRESS_RAW_REG_ADDRESS,
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
    .scan_index = 14,
    // Channel data is signed (2 complement), takes up 20 bits within a 32 bits
    // field, with the 4 LS bits being padding bits, and follows the host
    // CPU's endianness.
    .scan_type = {
      .sign = 's',
      .realbits = 20,
      .storagebits = 32,
      .shift = 4,
      .endianness = IIO_CPU,
    },
    .output = 0,
  },
  // Final "processed" pressure value.
  // Corresponding sysfs file: `in_pressure_processed`
  {
    .type = IIO_PRESSURE,
    .indexed = 0,
    .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
    .scan_index = 15,
    // Channel data is signed (2 complement), takes up 32 bits,
    // and follows the host CPU's endianness.
    .scan_type = {
      .sign = 's',
      .realbits = 32,
      .storagebits = 32,
      .shift = 0,
      .endianness = IIO_CPU,
    },
    .output = 0,
  },
};

static int bmp280_iio_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask);

/**
 * IIO hooks. We only need to read from the device.
 */
static const struct iio_info bmp280_iio_info = {
  .read_raw = bmp280_iio_read_raw,
};

// BMP280 I2C communication methods
static int initialize_bmp280(struct i2c_client *client);
static u16 read_bmp280_calibration_value(struct i2c_client *client, u8 reg_address);
static s32 read_bmp280_raw_temperature(struct i2c_client *client);
static s32 read_bmp280_processed_temperature(struct i2c_client *client);
static s32 read_bmp280_raw_pressure(struct i2c_client *client);
static s32 read_bmp280_processed_pressure(struct i2c_client *client);

/**
 * Sets up an IIO device and registers it with the IIO subsystem.
 */
int register_bmp280_iio_device(struct i2c_client *client) {
  int status = initialize_bmp280(client);
  if (status) {
    return status;
  }
  // Allocate IIO device structure.
  // devm_* methods do not require corresponding free/unregister calls.
  // When client->dev is removed, the reverse operation happens automatically.
  struct iio_dev *indio_dev =
    devm_iio_device_alloc(&client->dev,
			  /*sizeof_priv*/sizeof(struct i2c_client *));
  if (!indio_dev) {
    return -ENOMEM;
  }
  indio_dev->name = client->name;
  indio_dev->info = &bmp280_iio_info;
  indio_dev->modes = INDIO_DIRECT_MODE;
  indio_dev->channels = bmp280_iio_channels;
  indio_dev->num_channels = ARRAY_SIZE(bmp280_iio_channels);
  // Make the I2C client available from our IIO device structure.
  struct i2c_client **priv_data = iio_priv(indio_dev);
  *priv_data = client;
  // Register device with the IIO subsystem
  status = devm_iio_device_register(&client->dev, indio_dev);
  if (status) {
    pr_err("Failed to register with IIO subsystem");
    return status;
  }
  return 0;
}

/**
 * IIO driver's read method.
 * This method identifies which of the IIO sysfs files is being read from,
 * and assembles the result from sensor specific methos declared above.
 */
static int bmp280_iio_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask) {
  struct i2c_client **priv_data = iio_priv(indio_dev);
  struct i2c_client *client = *priv_data;
  switch (mask) {
  case IIO_CHAN_INFO_RAW:
    if ((chan->type == IIO_TEMP && 0 <= chan->channel && chan->channel <= 2) ||
	(chan->type == IIO_PRESSURE && 0 <= chan->channel && chan->channel <= 8)) {
      // One of the calibration values
      *val = read_bmp280_calibration_value(client, chan->address);
    } else if (chan->type == IIO_TEMP && chan->channel == 3) {
      // Raw temperature value
      *val = read_bmp280_raw_temperature(client);
    } else if (chan->type == IIO_PRESSURE && chan->channel == 9) {
      // Raw pressure value
      *val = read_bmp280_raw_pressure(client);
    } else {
      pr_err("Unexpected IIO raw channel type/number combination %d/%d\n",
	     chan->type, chan->channel);
      return -EINVAL;
    }
    return IIO_VAL_INT;
  case IIO_CHAN_INFO_PROCESSED:
    if (chan->type == IIO_TEMP) {
      // Processed temperature value. *val contains the temperature in
      // 100ths of C. So we set *val2 and return IIO_VAL_FRACTIONAL such
      // that the IIO core produces a fractional value to userspace.
      *val = read_bmp280_processed_temperature(client);
      *val2 = 100;
      return IIO_VAL_FRACTIONAL;
    } else if (chan->type == IIO_PRESSURE) {
      *val = read_bmp280_processed_pressure(client);
      return IIO_VAL_INT;
    } else {
      pr_err("Unexpected IIO processed channel type %d\n", chan->type);
      return -EINVAL;
    }
  default:
    pr_err("Unexpected IIO read mask %ld\n", mask);
    return -EINVAL;
  }
}

/**
 * Performs a device id sanity check, then initializes the BMP280 sensor.
 * We are using the following configuration:
 *     * maximum temperature and pressure oversampling (x16): this gives us
 * 20 bits of resolution.
 *     * Normal power mode: the sensor will continuously collecting samples.
 *     * 1000ms standby mode: samples are collected once per second.
 *     * No filtering: disable data smoothing over time.
 *     * No 3-wire SPI: we only use I2C
 */
static int initialize_bmp280(struct i2c_client *client) {
  // Try to read the sensor ID, and verify if it matches the expected BMP280 ID.
  u8 sensor_id = i2c_smbus_read_byte_data(client, BMP280_ID_REG);
  if (sensor_id != BMP280_ID) {
    pr_err("Unexpected sensor id 0x%02x. Expecting 0x%02x\n",
	   sensor_id, BMP280_ID);
    return -1;
  }
  // Maximum temperature oversampling (x16)
  u8 osrs_t = 0x5;
  // Maximum pressure oversampling (x16)
  u8 osrs_p = 0x5;
  // Normal power mode
  u8 mode = 0x3;
  // Standby time. In normal power mode, take a measurement every 1000ms (1s)
  u8 t_sb = 0x5;
  // No filtering
  u8 filter = 0x0;
  // No 3-wire SPI interface. We only use I2C
  u8 spi3w_en = 0x0;
  // These options are combined into the ctrl_meas and config registers
  u8 ctrl_meas = (osrs_t << 5) | (osrs_p << 2) | mode;
  u8 config = (t_sb << 5) | (filter << 2) | spi3w_en;
  i2c_smbus_write_byte_data(client, BMP280_CONFIG_REG_ADDRESS, config);
  i2c_smbus_write_byte_data(client, BMP280_CTRL_MEAS_REG_ADDRESS, ctrl_meas);
  return 0;
}

/**
 * Reads one of BMP280's calibration values.
 * These are 16 bit values, from 0x88 to 0xa1 on the sensor register bank.
 */
static u16 read_bmp280_calibration_value(struct i2c_client *client, u8 reg_address) {
  return i2c_smbus_read_word_data(client, reg_address);
}

/**
 * Reads the raw temperature value from the sensor.
 * It takes up the 20 MS bits of three consecutive 8 bit registers.
 * We read the three registers at once so we don't run the risk of the sensor
 * updating them while we are reading.
 */
static s32 read_bmp280_raw_temperature(struct i2c_client *client) {
  u8 values[3] = {0, 0, 0};
  i2c_smbus_read_i2c_block_data(client, BMP280_TEMP_RAW_REG_ADDRESS,
				/*length=*/3, values);
  s32 val1 = values[0];
  s32 val2 = values[1];
  s32 val3 = values[2];
  // The LS 4 bits of val3 are irrelevant. We do not right shift on this method,
  // we just return the raw value, as read from the sensor.
  return ((val1 << 16) | (val2 << 8) | val3);
}

/**
 * Computes the final temperature, in units of 1/100 degrees Celcius.
 * We do this using the calibration values and the conversion algorithm
 * described in the datasheet.
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 * (Section 3.11.3 - Compensation formula)
 */
static s32 read_bmp280_processed_temperature(struct i2c_client *client) {
  s32 dig_T1 = read_bmp280_calibration_value(
        client, BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS);
  s32 dig_T2 = read_bmp280_calibration_value(
        client, BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS + 2);
  s32 dig_T3 = read_bmp280_calibration_value(
        client, BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS + 4);
  s32 raw_temp = read_bmp280_raw_temperature(client);
  // dig_T1, dig_T2, and dig_T3 should be treated as unsigned, signed, and
  // signed, 16 bits numbers, respectively.
  if(dig_T2 > 32767) {
    dig_T2 -= 65536;
  }
  if(dig_T3 > 32767) {
    dig_T3 -= 65536;
  }
  // LS 4 bits of raw temperature are ignored.
  raw_temp >>= 4;
  // This rather cryptic set of operations is described in the datasheet
  s32 var1 = (((raw_temp >> 3) - (dig_T1 << 1)) * dig_T2) >> 11;
  s32 var2 = (((((raw_temp >> 4) - dig_T1) * ((raw_temp >> 4) - dig_T1)) >> 12)
	      * dig_T3) >> 14;
  return ((var1 + var2) * 5 + 128) >> 8;
}

/**
 * Reads the raw pressure value from the sensor.
 * It takes up the 20 MS bits of three consecutive 8 bit registers.
 * We read the three registers at once so we don't run the risk of the sensor
 * updating them while we are reading.
 */
static s32 read_bmp280_raw_pressure(struct i2c_client *client) {
  return 0;
}

static s32 read_bmp280_processed_pressure(struct i2c_client *client) {
  return 0;
}
