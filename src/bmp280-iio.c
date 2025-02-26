#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/printk.h>

MODULE_AUTHOR("Leonardo Blanger");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("An IIO compatible, I2C driver for the Bosch BMP280 "
		   "temperature and pressure sensor.");

/* Traditional device table matching approach.
   Listed here for completness only, since we rely mostly on the device tree. */
static const struct i2c_device_id bmp280_iio_i2c_driver_ids[] = {
  {
    .name = "leonardo,bmp280-iio",
  },
  { /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, bmp280_iio_i2c_driver_ids);

/* Device Tree (OF = open firmware) based matching ids. */
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

/* This macro gets replaced by an __init/__exit function pair,
   that does nothing other than registering/unregistering the I2C driver. */
module_i2c_driver(bmp280_iio_driver);

static int bmp280_iio_probe(struct i2c_client *client) {
  pr_info("Probing the i2c driver.\n");
  return 0;
}

static void bmp280_iio_remove(struct i2c_client *client) {
  pr_info("Removing the i2c driver.\n");
}
