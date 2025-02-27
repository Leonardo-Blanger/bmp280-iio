#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/printk.h>

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
  uint8_t sensor_id = i2c_smbus_read_byte_data(client, BMP280_ID_REG);
  if (sensor_id != BMP280_ID) {
    pr_err("Unexpected sensor id 0x%02x. Expecting 0x%02x\n",
	   sensor_id, BMP280_ID);
    return -1;
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
