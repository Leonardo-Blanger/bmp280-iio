#ifndef BMP280_IIO_H_
#define BMP280_IIO_H_

#include <linux/i2c.h>

int register_bmp280_iio_device(struct i2c_client *client);

#endif  // BMP280_IIO_H_
