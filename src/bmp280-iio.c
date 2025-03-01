#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/types.h>

MODULE_AUTHOR("Leonardo Blanger");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("An IIO compatible, I2C driver for the Bosch BMP280 "
		   "temperature and pressure sensor.");

/**
 * Expected I2C address. Can be configured as a module parameter if your sensor
 * somehow has a different address.
 * E.g. `sudo insmod bmp280-iio.ko bmp280_i2c_address=<addr>`
 */
static unsigned short bmp280_i2c_address = 0x76;
module_param(bmp280_i2c_address, ushort, S_IRUGO);
MODULE_PARM_DESC(bmp280_i2c_address, "I2C address for the BMP280 sensor");

/**
 * Used as a sanity check during driver probing.
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
 * Traditional device table matching approach.
 * Listed here for completness only, since we rely mostly on the device tree.
 */
static const struct i2c_device_id bmp280_iio_i2c_driver_ids[] = {
  {
    .name = "leonardo,bmp280-iio",
  },
  { /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, bmp280_iio_i2c_driver_ids);

/**
 * Device Tree (OF = open firmware) based matching ids.
 */
static const struct of_device_id bmp280_iio_of_driver_ids[] = {
  {
    .compatible = "leonardo,bmp280-iio",
  },
  { /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, bmp280_iio_of_driver_ids);

static int bmp280_iio_probe(struct i2c_client *client);
static void bmp280_iio_remove(struct i2c_client *client);

static struct i2c_driver bmp280_iio_driver = {
  .probe = bmp280_iio_probe,
  .remove = bmp280_iio_remove,
  .id_table = bmp280_iio_i2c_driver_ids,
  .driver = {
    .name = "leonardo,bmp280-iio",
    .of_match_table = of_match_ptr(bmp280_iio_of_driver_ids),
  },
};

/**
 * This macro gets replaced by a module __init/__exit function pair,
 * that does nothing other than registering/unregistering the I2C driver.
*/
module_i2c_driver(bmp280_iio_driver);

/**
 * IIO channel macro for calibration values.
 * For triggered buffer reads, `scan_index` sets the position of this channel's
 * data within the sample.
 * `scan_type` tells that the channel data is unsigned, takes up 16 bits without
 * any padding, and follows the CPU's endianness.
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
 * We make one channel available for each of the three calibration values,
 * plus one chanel for the raw, unprocessed, temperature value, and one channel
 * for the final, processed, temperature value.
 * Within `/sys/bus/iio/devices/iio:deviceX/`, these will be:
 * `in_temp{0-3}_raw` and `in_temp_input`, respectively.
 */
static const struct iio_chan_spec bmp280_iio_channels[] = {
  // Temperature calibration values, refered to as dig_T1 to dig_T3 on the
  // datasheet. Corresponding sysfs files: `in_temp0_raw` to `in_temp2_raw.
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
      .realbits = 24,
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

/**
 * I2C driver probe.
 * Performs sanity checks on the client address, and sensor ID register,
 * then sets up an IIO device and registers it with the IIO subsystem.
 */
static int bmp280_iio_probe(struct i2c_client *client) {
  pr_info("Probing the i2c driver.\n");
  if (client->addr != bmp280_i2c_address) {
    pr_err("Probed with unexpected I2C address 0x%02x. Expecting 0x%02x\n",
	   client->addr, bmp280_i2c_address);
    return -1;
  }
  // Try to read the sensor ID, and verify if it matches the expected BMP280 ID.
  u8 sensor_id = i2c_smbus_read_byte_data(client, BMP280_ID_REG);
  if (sensor_id != BMP280_ID) {
    pr_err("Unexpected sensor id 0x%02x. Expecting 0x%02x\n",
	   sensor_id, BMP280_ID);
    return -1;
  }
  // Set up IIO device structure.
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
  int status = devm_iio_device_register(&client->dev, indio_dev);
  if (status) {
    pr_err("Failed to register with IIO subsystem");
    return status;
  }
  pr_info("Probed i2c driver successfully.\n");
  return 0;
}

/**
 * I2C driver remove.
 * We do not need to undo anything manually here, since we only used
 * "device managed" operations during probe, (i.e. devm_*() functions).
 * Those actions are automatically undone when client->dev is removed.
 */
static void bmp280_iio_remove(struct i2c_client *client) {
  pr_info("Removing the i2c driver.\n");
}

int read_bmp280_calibration_value(struct i2c_client *client,
				  int calib_index,
				  u16 *value);
int read_bmp280_raw_temperature(struct i2c_client *client, u32 *value);
int read_bmp280_processed_temperature(struct i2c_client *client, s32 *value);

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
    if (0 <= chan->channel && chan->channel <= 2) {
      // One of the three calibration values
      u16 calib_value = 0;
      int status =
	read_bmp280_calibration_value(client, chan->channel, &calib_value);
      if (status) {
	return status;
      }
      *val = calib_value;
    } else if(chan->channel == 3) {
      // The raw temperature value.
      u32 raw_temp = 0;
      int status = read_bmp280_raw_temperature(client, &raw_temp);
      if (status) {
	return status;
      }
      *val = raw_temp;
    } else {
      pr_err("Unexpected IIO channel number %d\n", chan->channel);
      return -EINVAL;
    }
    break;
  case IIO_CHAN_INFO_PROCESSED:
    s32 processed_temp = 0;
    int status = read_bmp280_processed_temperature(client, &processed_temp);
    if (status) {
      return status;
    }
    *val = processed_temp;
    break;
  default:
    pr_err("Unexpected IIO read mask %ld\n", mask);
    return -EINVAL;
  }
  return IIO_VAL_INT;
}

int read_bmp280_calibration_value(struct i2c_client *client,
				  int calib_index,
				  u16 *value) {
  *value = 10 + calib_index;
  return 0;
}

int read_bmp280_raw_temperature(struct i2c_client *client, u32 *value) {
  *value = 111;
  return 0;
}

int read_bmp280_processed_temperature(struct i2c_client *client, s32 *value) {
  *value = 22;
  u8 sensor_id = i2c_smbus_read_byte_data(client, BMP280_ID_REG);
  pr_info("Sensor ID: 0x%02x\n", sensor_id);
  return 0;
}
