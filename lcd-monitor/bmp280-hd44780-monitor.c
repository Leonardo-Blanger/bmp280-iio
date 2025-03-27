#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/iio/consumer.h>
#include <linux/iio/types.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/workqueue.h>

MODULE_AUTHOR("Leonardo Blanger");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMP280 live monitoring module "
		   "using the HD44780 character LCD display.");

/**
 * The bmp280-iio module is a dependency, and needs to be loaded before.
 */
MODULE_SOFTDEP("pre: bmp280-iio");

struct hd44780;
extern struct hd44780 *hd44780_get(int index);
extern void hd44780_put(struct hd44780 *hd44780);
extern int hd44780_reset_display(struct hd44780 *hd44780);
extern ssize_t hd44780_write(struct hd44780 *hd44780, const char *msg, size_t length);

/**
 * Monitor context structure.
 *
 * There is one instance of this allocated for each probed driver.
 *
 * Must be initialized before being used, and de-initialized after no longer
 * needed, through calls to monitor_init and monitor_teardown, respectively.
 */
struct bmp280_hd44780_monitor {
  struct mutex monitor_mutex;
  // References to BMP280 IIO channels
  struct iio_channel *temperature_channel;
  struct iio_channel *pressure_channel;
  // Work structure for the periodic data refresh
  struct delayed_work dwork;
  // ID of display we are writing to. Default to 0.
  s32 display_index;
  // How often do we update the display with new values. Default to 2 seconds.
  u32 refresh_period_ms;
  // Whether we are running or not.
  bool running;
};

static void bmp280_hd44780_monitor_work(struct work_struct *work);

/**
 * Initializes a monitor context structure.
 *
 * A call to this function must eventually be followed by a call to
 * monitor_teardown.
 *
 * Initializes the structure mutex, the monitor worker, and assign default
 * values to monitor parameters.
 */
static void monitor_init(struct bmp280_hd44780_monitor *monitor) {
  mutex_init(&monitor->monitor_mutex);
  // Set up workqueue entry for our running worker function
  INIT_DELAYED_WORK(&monitor->dwork, &bmp280_hd44780_monitor_work);
  // Assign default parameter values
  monitor->display_index = 0;
  monitor->refresh_period_ms = 2000;
  monitor->running = true;
}

/**
 * Monitor context structure teardown.
 *
 * Counterpart to monitor_init. Cancels (synchronously) the monitor worker, and
 * destroys the mutex.
 */
static void monitor_teardown(struct bmp280_hd44780_monitor *monitor) {
  cancel_delayed_work_sync(&monitor->dwork);
  mutex_destroy(&monitor->monitor_mutex);
}

#define REFRESH_PERIOD_SEC 2

static void bmp280_hd44780_monitor_work(struct work_struct *work) {
  struct delayed_work *dwork = container_of(work, struct delayed_work, work);
  struct bmp280_hd44780_monitor *monitor =
    container_of(dwork, struct bmp280_hd44780_monitor, dwork);
  int status = 0;
  int temperature = 0, temperature_val2 = 0;
  status = iio_read_channel_attribute(monitor->temperature_channel,
				      &temperature, &temperature_val2,
				      IIO_CHAN_INFO_PROCESSED);
  if (status < 0) {
    pr_err("Failed to read temperature value from IIO channel: %d\n", status);
    return;
  }
  if (status != IIO_VAL_FRACTIONAL) {
    pr_err("Unexpected IIO temperature channel type: %d\n", status);
    return;
  }
  int pressure = 0, pressure_val2 = 0;
  status = iio_read_channel_attribute(monitor->pressure_channel,
				      &pressure, &pressure_val2,
				      IIO_CHAN_INFO_PROCESSED);
  if (status < 0) {
    pr_err("Failed to read pressure value from IIO channel: %d\n", status);
    return;
  }
  if (status != IIO_VAL_FRACTIONAL) {
    pr_err("Unexpected IIO pressure channel type: %d\n", status);
    return;
  }
  int temperature_int = temperature / temperature_val2;
  int temperature_100ths =
    (100 * (temperature % temperature_val2)) / temperature_val2;
  int pressure_int = pressure / pressure_val2;
  int pressure_100ths = (100 * (pressure % pressure_val2)) / pressure_val2;

  struct hd44780 *display = hd44780_get(0);
  if (IS_ERR(display)) {
    pr_err("Failed to retrieve display: %ld\n", PTR_ERR(display));
  } else {
    hd44780_write(display, "Hello World!", 12);
    hd44780_put(display);
    display = NULL;
  }

  pr_info("Current temperature: %d.%02d C -- Current pressure: %d.%02d P\n",
	  temperature_int, temperature_100ths, pressure_int, pressure_100ths);
  unsigned long delay = msecs_to_jiffies(REFRESH_PERIOD_SEC * 1000);
  schedule_delayed_work(dwork, delay);
}

static ssize_t
bmp280_hd44780_monitor_parameter_show(struct device *dev,
				      struct device_attribute *attr, char *buf);
static ssize_t
bmp280_hd44780_monitor_parameter_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count);

/**
 * Sysfs device attribute files for runtime configuration.
 */
static DEVICE_ATTR(monitor_display_index, 0644,
		   bmp280_hd44780_monitor_parameter_show,
		   bmp280_hd44780_monitor_parameter_store);
static DEVICE_ATTR(monitor_refresh_period_ms, 0644,
		   bmp280_hd44780_monitor_parameter_show,
		   bmp280_hd44780_monitor_parameter_store);
static DEVICE_ATTR(monitor_running, 0644,
		   bmp280_hd44780_monitor_parameter_show,
		   bmp280_hd44780_monitor_parameter_store);

/**
 * Sysfs device attribute show function.
 */
static ssize_t
bmp280_hd44780_monitor_parameter_show(struct device *dev,
				      struct device_attribute *attr, char *buf) {
  struct bmp280_hd44780_monitor *monitor = dev_get_drvdata(dev);
  if (mutex_lock_interruptible(&monitor->monitor_mutex)) {
    return -ERESTARTSYS;
  }
  ssize_t ret = 0;
  if (attr == &dev_attr_monitor_display_index) {
    ret = snprintf(buf, 12, "%d", monitor->display_index);
  } else if (attr == &dev_attr_monitor_refresh_period_ms) {
    ret = snprintf(buf, 11, "%u", monitor->refresh_period_ms);
  } else if (attr == &dev_attr_monitor_running) {
    ret = snprintf(buf, 2, "%d", monitor->running ? 1 : 0);
  } else {
    ret = -EINVAL;
  }
  mutex_unlock(&monitor->monitor_mutex);
  return ret;
}

/**
 * Sysfs device attribute store function.
 */
static ssize_t
bmp280_hd44780_monitor_parameter_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count) {
  // 25 characters is enough for any 64 bit value.
  if (count > 25) {
    pr_err("Attempt to write unexpected long value to sysfs attribute.\n");
    return -EINVAL;
  }
  struct bmp280_hd44780_monitor *monitor = dev_get_drvdata(dev);
  if (mutex_lock_interruptible(&monitor->monitor_mutex)) {
    return -ERESTARTSYS;
  }
  // Copy buffer to local, null-terminated string.
  char str[26];
  memset(str, 0, sizeof(str));
  memcpy(str, buf, count);
  ssize_t ret = 0;
  if (attr == &dev_attr_monitor_display_index) {
    // base=0 means autodetect base
    ret = kstrtos32(str, /*base=*/0, &monitor->display_index);
  } else if (attr == &dev_attr_monitor_refresh_period_ms) {
    // base=0 means autodetect base
    ret = kstrtou32(str, /*base=*/0, &monitor->refresh_period_ms);
  } else if (attr == &dev_attr_monitor_running) {
    s32 value = monitor->running;
    // base=0 means autodetect base
    ret = kstrtos32(str, /*base=*/0, &value);
    monitor->running = (value != 0);
  } else {
    ret = -EINVAL;
  }
  mutex_unlock(&monitor->monitor_mutex);
  if (ret == 0) {
    ret = count;
  }
  return ret;
}

/**
 * Monitor platform driver probe method.
 *
 * Allocates and initializes an instance of our monitor, retrieves references to
 * the BMP280 IIO channels, and start our worker thread (as a system default
 * workqueue entry).
 */
static int bmp280_hd44780_monitor_probe(struct platform_device *pdev) {
  pr_info("Probing bmp280-hd44780-monitor platform driver.\n");
  // Allocate driver context instance
  struct bmp280_hd44780_monitor *monitor =
    devm_kzalloc(&pdev->dev, sizeof(struct bmp280_hd44780_monitor), GFP_KERNEL);
  if (!monitor) {
    return -ENOMEM;
  }
  monitor_init(monitor);
  int ret = 0;
  // Attempt to retrieve temperature channel as a device property
  monitor->temperature_channel =
    devm_iio_channel_get(&pdev->dev, "temperature");
  if (IS_ERR(monitor->temperature_channel)) {
    pr_err("Failed to acquire IIO temperature channel with error %ld. "
	   "Aborting probe.\n", PTR_ERR(monitor->temperature_channel));
    ret = PTR_ERR(monitor->temperature_channel);
    goto out_fail;
  }
  // Attempt to retrieve pressure channel as a device property
  monitor->pressure_channel =
    devm_iio_channel_get(&pdev->dev, "pressure");
  if (IS_ERR(monitor->pressure_channel)) {
    pr_err("Failed to acquire IIO pressure channel with error %ld. "
	   "Aborting probe.\n", PTR_ERR(monitor->pressure_channel));
    ret = PTR_ERR(monitor->pressure_channel);
    goto out_fail;
  }
  // Make our context structure available from this device
  dev_set_drvdata(&pdev->dev, monitor);
  // Setup sysfs files for driver runtime control
  ret = device_create_file(&pdev->dev, &dev_attr_monitor_display_index);
  if (ret) {
    goto out_fail;
  }
  ret = device_create_file(&pdev->dev, &dev_attr_monitor_refresh_period_ms);
  if (ret) {
    goto out_fail;
  }
  ret = device_create_file(&pdev->dev, &dev_attr_monitor_running);
  if (ret) {
    goto out_fail;
  }
  // Start our monitor worker thread
  unsigned long delay = msecs_to_jiffies(REFRESH_PERIOD_SEC * 1000);
  if (!schedule_delayed_work(&monitor->dwork, delay)) {
    pr_err("Failed to schedule worker thread. Aborting probe.\n");
    ret = -EFAULT;
    goto out_fail;
  }
  pr_info("Successfully probed bmp280-hd44780-monitor platform driver.\n");
  return 0;
 out_fail:
  device_remove_file(&pdev->dev, &dev_attr_monitor_display_index);
  device_remove_file(&pdev->dev, &dev_attr_monitor_refresh_period_ms);
  device_remove_file(&pdev->dev, &dev_attr_monitor_running);
  monitor_teardown(monitor);
  return ret;
}

/**
 * Monitor platform driver remove function.
 *
 * Stops the driver worker thread.
 */
static void bmp280_hd44780_monitor_remove(struct platform_device *pdev) {
  device_remove_file(&pdev->dev, &dev_attr_monitor_display_index);
  device_remove_file(&pdev->dev, &dev_attr_monitor_refresh_period_ms);
  device_remove_file(&pdev->dev, &dev_attr_monitor_running);
  struct bmp280_hd44780_monitor *monitor = dev_get_drvdata(&pdev->dev);
  monitor_teardown(monitor);
  pr_info("Successfully removed bmp280-hd44780-monitor platform driver.\n");
}

/**
 * Device Tree based matching ids (OF = open firmware).
 *
 * Used for auto loading the kernel module, and for driver matching.
 */
static const struct of_device_id bmp280_hd44780_monitor_of_driver_ids[] = {
  {
    .compatible = "leonardo,bmp280-hd44780-monitor",
  },
  { /* sentinel */ },
};

/**
 * HD44780 LCD monitor platform driver for the BMP280 IIO driver.
 */
static struct platform_driver bmp280_hd44780_monitor_driver = {
  .probe = bmp280_hd44780_monitor_probe,
  .remove_new = bmp280_hd44780_monitor_remove,
  .driver = {
    .name = "bmp280-hd44780-monitor",
    .of_match_table = of_match_ptr(bmp280_hd44780_monitor_of_driver_ids),
  },
};

/**
 * Module init function.
 *
 * Registers the platform driver.
 */
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

/**
 * Module exit function.
 *
 * Unregisters the platform driver registered during init.
 */
static void __exit bmp280_hd44780_monitor_exit(void) {
  platform_driver_unregister(&bmp280_hd44780_monitor_driver);
  pr_info("Removed bmp280-hd44780-monitor.\n");
}

module_init(bmp280_hd44780_monitor_init);
module_exit(bmp280_hd44780_monitor_exit);
