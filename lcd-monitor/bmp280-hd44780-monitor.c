#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/iio/consumer.h>
#include <linux/init.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/of.h>

MODULE_AUTHOR("Leonardo Blanger");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMP280 live monitoring module "
		   "using the HD44780 character LCD display.");

/**
 * The bmp280-iio module is a dependency, and needs to be loaded before.
 */
MODULE_SOFTDEP("pre: bmp280-iio");

struct bmp280_hd44780_monitor {
  struct iio_channel *temperature_channel;
  struct iio_channel *pressure_channel;
};

static int bmp280_hd44780_monitor_probe(struct platform_device *pdev) {
  pr_info("Probing bmp280-hd44780-monitor platform driver.\n");
  // Allocate driver context instance
  struct bmp280_hd44780_monitor *monitor =
    devm_kzalloc(&pdev->dev, sizeof(struct bmp280_hd44780_monitor), GFP_KERNEL);
  if (!monitor) {
    return -ENOMEM;
  }
  // Attempt to retrieve temperature channel as a device property
  monitor->temperature_channel =
    devm_iio_channel_get(&pdev->dev, "temperature");
  if (IS_ERR(monitor->temperature_channel)) {
    pr_err("Failed to acquire IIO temperature channel with error %ld. "
	   "Aborting probe.\n", PTR_ERR(monitor->temperature_channel));
    return PTR_ERR(monitor->temperature_channel);
  }
  // Attempt to retrieve pressure channel as a device property
  monitor->pressure_channel =
    devm_iio_channel_get(&pdev->dev, "pressure");
  if (IS_ERR(monitor->pressure_channel)) {
    pr_err("Failed to acquire IIO pressure channel with error %ld. "
	   "Aborting probe.\n", PTR_ERR(monitor->pressure_channel));
    return PTR_ERR(monitor->pressure_channel);
  }
  // Make our context structure available from the platform driver
  platform_set_drvdata(pdev, monitor);
  pr_info("Successfully probed bmp280-hd44780-monitor platform driver.\n");
  return 0;
}

static void bmp280_hd44780_monitor_remove(struct platform_device *pdev) {
  pr_info("Successfully removed bmp280-hd44780-monitor platform driver.\n");
}

static const struct of_device_id bmp280_hd44780_monitor_of_driver_ids[] = {
  {
    .compatible = "leonardo,bmp280-hd44780-monitor",
  },
  { /* sentinel */ },
};

static struct platform_driver bmp280_hd44780_monitor_driver = {
  .probe = bmp280_hd44780_monitor_probe,
  .remove_new = bmp280_hd44780_monitor_remove,
  .driver = {
    .name = "bmp280-hd44780-monitor",
    .of_match_table = of_match_ptr(bmp280_hd44780_monitor_of_driver_ids),
  },
};

static int __init bmp280_hd44780_monitor_init(void) {
  int status = platform_driver_register(&bmp280_hd44780_monitor_driver);
  if (status) {
    pr_err("Failed to register bmp280-hd44780-monitor platform driver: %d\n",
	   status);
    return status;
  }
  pr_info("Loaded bmp280-hd44780-monitor.\n");
  return 0;
}

static void __exit bmp280_hd44780_monitor_exit(void) {
  platform_driver_unregister(&bmp280_hd44780_monitor_driver);
  pr_info("Removed bmp280-hd44780-monitor.\n");
}

module_init(bmp280_hd44780_monitor_init);
module_exit(bmp280_hd44780_monitor_exit);
