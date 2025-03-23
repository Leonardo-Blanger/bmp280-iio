#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/printk.h>

MODULE_AUTHOR("Leonardo Blanger");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMP280 live monitoring module "
		   "using the HD44780 character LCD display.");

/**
 * The bmp280-iio module is a dependency, and needs to be loaded before.
 */
MODULE_SOFTDEP("pre: bmp280-iio");

static int __init bmp280_hd44780_monitor_init(void) {
  pr_info("Loaded bmp280-hd44780-monitor.\n");
  return 0;
}

static void __exit bmp280_hd44780_monitor_exit(void) {
  pr_info("Removed bmp280-hd44780-monitor.\n");
}

module_init(bmp280_hd44780_monitor_init);
module_exit(bmp280_hd44780_monitor_exit);
