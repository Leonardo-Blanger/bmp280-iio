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
 * BMP280 context structure.
 * dig_T and dig_P are the sensor's calibration values, which are constant for
 * any given sensor, so we only read them once and keep store them here.
 */
struct bmp280_ctx {
  struct i2c_client *client;
  s32 dig_T[4];
  s64 dig_P[10];
};

/**
 * IIO channel macro for calibration values.
 * For triggered buffer reads, `scan_index` sets the position of this channel
 * data within the sample. `scan_type` tells that the channel data takes up 16
 * bits without any padding, and follows the CPU's endianness.
 */
#define BMP280_CALIBR_CHANNNEL(_type, _index, _scan_index, _sign, _address) { \
    .type = (_type),							\
    .indexed = 1,							\
    .channel = (_index),						\
    .address = (_address),						\
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
    .scan_index = (_scan_index),					\
    .scan_type = {							\
      .sign = (_sign),							\
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
  BMP280_CALIBR_CHANNNEL(IIO_TEMP, 0, 0, 'u',
			 BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS),
  BMP280_CALIBR_CHANNNEL(IIO_TEMP, 1, 1, 's',
			 BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS + 2),
  BMP280_CALIBR_CHANNNEL(IIO_TEMP, 2, 2, 's',
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
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 0, 5, 'u',
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 1, 6, 's',
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 2),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 2, 7, 's',
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 4),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 3, 8, 's',
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 6),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 4, 9, 's',
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 8),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 5, 10, 's',
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 10),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 6, 11, 's',
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 12),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 7, 12, 's',
			 BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS + 14),
  BMP280_CALIBR_CHANNNEL(IIO_PRESSURE, 8, 13, 's',
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
    // Channel data is unsigned, takes up 32 bits,
    // and follows the host CPU's endianness.
    .scan_type = {
      .sign = 'u',
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
static int setup_bmp280(struct i2c_client *client, struct bmp280_ctx *bmp280);
static int initialize_bmp280(struct bmp280_ctx *bmp280);
static int read_bmp280_calibration_values(struct bmp280_ctx *bmp280);
static int read_bmp280_raw_temperature(struct bmp280_ctx *bmp280,
				       s32 *raw_temp);
static int read_bmp280_raw_pressure(struct bmp280_ctx *bmp280, s32 *raw_press);
static int read_bmp280_processed_temperature(struct bmp280_ctx *bmp280,
					     s32 *temp);
static int read_bmp280_processed_pressure(struct bmp280_ctx *bmp280,
					  u32 *press);

/**
 * Sets up an IIO device and registers it with the IIO subsystem.
 */
int register_bmp280_iio_device(struct i2c_client *client) {
  // Allocate IIO device structure.
  // devm_* methods do not require corresponding free/unregister calls.
  // When client->dev is removed, the reverse operation happens automatically.
  struct iio_dev *indio_dev =
    devm_iio_device_alloc(&client->dev,
			  /*sizeof_priv*/sizeof(struct bmp280_ctx));
  if (!indio_dev) {
    return -ENOMEM;
  }
  indio_dev->name = client->name;
  indio_dev->info = &bmp280_iio_info;
  indio_dev->modes = INDIO_DIRECT_MODE;
  indio_dev->channels = bmp280_iio_channels;
  indio_dev->num_channels = ARRAY_SIZE(bmp280_iio_channels);
  struct bmp280_ctx *bmp280 = iio_priv(indio_dev);
  int status = setup_bmp280(client, bmp280);
  if (status) {
    pr_err("Failed to setup BMP280 device.");
    return status;
  }
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
  struct bmp280_ctx *bmp280 = iio_priv(indio_dev);
  switch (mask) {
  case IIO_CHAN_INFO_RAW:
    if (chan->type == IIO_TEMP && 0 <= chan->channel && chan->channel < 3) {
      // One of the constant temperature calibration values.
      *val = bmp280->dig_T[chan->channel + 1];
    } else if (chan->type == IIO_PRESSURE &&
	       0 <= chan->channel && chan->channel < 9) {
      // One of the constant pressure calibration values.
      *val = bmp280->dig_P[chan->channel + 1];
    } else if (chan->type == IIO_TEMP && chan->channel == 3) {
      // Raw temperature value
      s32 raw_temp;
      int status = read_bmp280_raw_temperature(bmp280, &raw_temp);
      if (status) {
	return status;
      }
      *val = raw_temp;
    } else if (chan->type == IIO_PRESSURE && chan->channel == 9) {
      // Raw pressure value
      s32 raw_press;
      int status = read_bmp280_raw_pressure(bmp280, &raw_press);
      if (status) {
	return status;
      }
      *val = raw_press;
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
      s32 temp;
      int status = read_bmp280_processed_temperature(bmp280, &temp);
      if (status) {
	return status;
      }
      *val = temp;
      *val2 = 100;
    } else if (chan->type == IIO_PRESSURE) {
      // Processed pressure value. *val contains the pressure in
      // 1/256 of Pascal. So we set *val2 and return IIO_VAL_FRACTIONAL such
      // that the IIO core produces a fractional value to userspace.
      u32 press;
      int status = read_bmp280_processed_pressure(bmp280, &press);
      if (status) {
	return status;
      }
      *val = press;
      *val2 = 256;
    } else {
      pr_err("Unexpected IIO processed channel type %d\n", chan->type);
      return -EINVAL;
    }
    return IIO_VAL_FRACTIONAL;
  default:
    pr_err("Unexpected IIO read mask %ld\n", mask);
    return -EINVAL;
  }
}

/**
 * Calls sensor initialization functions, then reads the constant calibration
 * values from the sensor and sets the BMP280 context structure.
 */
static int setup_bmp280(struct i2c_client *client, struct bmp280_ctx *bmp280) {
  // Make the I2C client available from the context structure
  bmp280->client = client;
  // Initialize sensor
  int status = initialize_bmp280(bmp280);
  if (status) {
    return status;
  }
  status = read_bmp280_calibration_values(bmp280);
  if (status) {
    return status;
  }
  return 0;
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
static int initialize_bmp280(struct bmp280_ctx *bmp280) {
  // Try to read the sensor ID, and verify if it matches the expected BMP280 ID.
  u8 sensor_id = i2c_smbus_read_byte_data(bmp280->client, BMP280_ID_REG);
  if (sensor_id != BMP280_ID) {
    pr_err("Unexpected sensor id 0x%02x. Expecting 0x%02x\n",
	   sensor_id, BMP280_ID);
    return -ENODEV;
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
  i2c_smbus_write_byte_data(bmp280->client, BMP280_CONFIG_REG_ADDRESS, config);
  i2c_smbus_write_byte_data(bmp280->client, BMP280_CTRL_MEAS_REG_ADDRESS,
			    ctrl_meas);
  return 0;
}

/**
 * Reads the BMP280 constant calibration values, and stores them in the context
 * structure's dig_T and dig_P fields.
 * These are 16 bit values, stored from 0x88 to 0xa1 on the sensor register
 * bank.
 */
static int read_bmp280_calibration_values(struct bmp280_ctx *bmp280) {
  // Read all temperature calibration values, then all pressure calibration
  // values, to minimize the number of I2C reads during setup.
  // Note: `n_read` is specified in bytes.
  u16 temp_calib_buffer[3];
  s32 read = i2c_smbus_read_i2c_block_data(
      bmp280->client, BMP280_TEMP_CALIBRATION_BASE_REG_ADDRESS, 
      /*n_read=*/3 * 2, (u8 *)temp_calib_buffer);
  if (read != 3 * 2) {
    pr_err("Expected 6 temperature calibration bytes. Read %d instead\n", read);
    return -EIO;
  }
  u16 press_calib_buffer[9];
  read = i2c_smbus_read_i2c_block_data(
      bmp280->client, BMP280_PRESS_CALIBRATION_BASE_REG_ADDRESS,
      /*n_read=*/9 * 2, (u8 *)press_calib_buffer);
  if (read != 9 * 2) {
    pr_err("Expected 6 pressure calibration bytes. Read %d instead\n", read);
    return -EIO;
  }
  // ignoring index zero to match the indexes starting from 1 in the
  // datasheet algorithm.
  bmp280->dig_T[0] = 0;
  bmp280->dig_P[0] = 0;
  // dig_T1 and dig_P1 should be treated as unsigned 16 bits, whereas all other
  // calibration values should be treated as signed 16 bits.
  for (int i = 1; i <= 3; i++) {
    bmp280->dig_T[i] = temp_calib_buffer[i-1];
    if(i != 1 && bmp280->dig_T[i] > 32767) {
      bmp280->dig_T[i] -= 65536;
    }
  }
  for (int i = 1; i <= 9; i++) {
    bmp280->dig_P[i] = press_calib_buffer[i-1];
    if(i != 1 && bmp280->dig_P[i] > 32767) {
      bmp280->dig_P[i] -= 65536;
    }
  }
  return 0;
}

/**
 * Reads the raw temperature value from the sensor.
 * It takes up the 20 MS bits of three consecutive 8 bit registers.
 * We read the three registers at once so we don't run the risk of the sensor
 * updating them while we are reading.
 */
static int read_bmp280_raw_temperature(struct bmp280_ctx *bmp280,
				       s32 *raw_temp) {
  u8 values[3] = {0, 0, 0};
  s32 read = i2c_smbus_read_i2c_block_data(
      bmp280->client, BMP280_TEMP_RAW_REG_ADDRESS, /*length=*/3, values);
  if (read != 3) {
    pr_err("Expected to read 3 raw temperature bytes. Read %d instead\n", read);
    return -EIO;
  }
  s32 val1 = values[0];
  s32 val2 = values[1];
  s32 val3 = values[2];
  // The LS 4 bits of val3 are irrelevant. We do not right shift on this method,
  // we just return the raw value, as read from the sensor.
  *raw_temp = ((val1 << 16) | (val2 << 8) | val3);
  return 0;
}

/**
 * Reads the raw pressure value from the sensor.
 * It takes up the 20 MS bits of three consecutive 8 bit registers.
 * We read the three registers at once so we don't run the risk of the sensor
 * updating them while we are reading.
 */
static int read_bmp280_raw_pressure(struct bmp280_ctx *bmp280,
				    s32 *raw_press) {
  u8 values[3] = {0, 0, 0};
  s32 read = i2c_smbus_read_i2c_block_data(
      bmp280->client, BMP280_PRESS_RAW_REG_ADDRESS, /*length=*/3, values);
  if (read != 3) {
    pr_err("Expected to read 3 raw pressure bytes. Read %d instead\n", read);
    return -EIO;
  }
  s32 val1 = values[0];
  s32 val2 = values[1];
  s32 val3 = values[2];
  // The LS 4 bits of val3 are irrelevant. We do not right shift on this method,
  // we just return the raw value, as read from the sensor.
  *raw_press = ((val1 << 16) | (val2 << 8) | val3);
  return 0;
}

/**
 * `t_fine` is an intermediate temperature value, required by both the final
 * processed temperature, as well as for pressure computation. See the
 * datasheet for details.
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 * (Section 3.11.3 - Compensation formula)
 */
static s32 compute_bmp280_t_fine(s32 raw_temp, const s32 dig_T[]) {
  // This rather cryptic set of operations is described in the datasheet
  s32 var1 = (((raw_temp >> 3) - (dig_T[1] << 1)) * dig_T[2]) >> 11;
  s32 var2 = (((((raw_temp >> 4) - dig_T[1]) * ((raw_temp >> 4) - dig_T[1])) >> 12)
	      * dig_T[3]) >> 14;
  return var1 + var2;
}

/**
 * Computes the final temperature, in units of 1/100 degrees Celcius.
 * We do this using the calibration values and the conversion algorithm
 * described in the datasheet.
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 * (Section 3.11.3 - Compensation formula)
 */
static int read_bmp280_processed_temperature(struct bmp280_ctx *bmp280, s32 *temp) {
  s32 raw_temp;
  int status = read_bmp280_raw_temperature(bmp280, &raw_temp);
  if (status) {
    return status;
  }
  // LS 4 bits of raw temperature are ignored.
  raw_temp >>= 4;
  *temp = (compute_bmp280_t_fine(raw_temp, bmp280->dig_T) * 5 + 128) >> 8;
  return 0;
}

/**
 * Computes the final pressure, as an unsigned 32 bit integer,
 * in units of 1 / 256 Pascal.
 * We do this using the calibration values and the conversion algorithm
 * described in the datasheet.
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 * (Section 3.11.3 - Compensation formula)
 */
static int read_bmp280_processed_pressure(struct bmp280_ctx *bmp280, u32 *press) {
  // We need both the raw temperature, and the raw pressure values to compute
  // the final pressure. Pressure registers come before. We read all of them
  // at once to avoid the risk of the sensor chaning either of them in between
  // reads.
  u8 values[6];
  int read = i2c_smbus_read_i2c_block_data(
      bmp280->client, BMP280_PRESS_RAW_REG_ADDRESS, /*length=*/6, values);
  if (read != 6) {
    pr_err("Expected to read 6 temperature/pressure bytes. Read %d instead\n",
	   read);
    return -EIO;
  }
  s32 p1 = values[0];
  s32 p2 = values[1];
  s32 p3 = values[2];
  s32 raw_press = (p1 << 16) | (p2 << 8) | p3;
  s32 t1 = values[3];
  s32 t2 = values[4];
  s32 t3 = values[5];
  s32 raw_temp = (t1 << 16) | (t2 << 8) | t3;
  // LS 4 bits of raw temperature and pressure are ignored.
  raw_press >>= 4;
  raw_temp >>= 4;
  s64 t_fine = compute_bmp280_t_fine(raw_temp, bmp280->dig_T);
  s64 var1 = t_fine - 128000;
  s64 var2 = var1 * var1 * bmp280->dig_P[6];
  var2 = var2 + ((var1 * bmp280->dig_P[5]) << 17);
  var2 = var2 + (bmp280->dig_P[4] << 35);
  var1 = (((var1 * var1 * bmp280->dig_P[3]) >> 8) +
	  ((var1 * bmp280->dig_P[2]) << 12));
  var1 = ((((s64)1) << 47) + var1) * bmp280->dig_P[1] >> 33;
  if (var1 == 0) {
    *press = 0;
    return 0;
  }
  s64 p = 1048576 - raw_press;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (bmp280->dig_P[9] * (p >> 13) * (p >> 13)) >> 25;
  var2 = (bmp280->dig_P[8] * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (bmp280->dig_P[7] << 4);
  *press = (u32)p;
  return 0;
}
