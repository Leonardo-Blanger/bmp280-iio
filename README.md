# IIO Compatible Linux Kernel Driver for the BMP280 Temperature and Pressure Sensor

> **Disclaimer:** This is a hobby project I developed after having studied Linux driver development for the past few weeks. I came up with this project as a way to explore concepts such as I2C comunication and the the IIO subsystem in practice. In summary, the main purpose for this project is self learning, and it was absolutely **not** tested for "mission critical" use cases.

This project provides a Linux kernel driver for the Bosch BMP280 temperature and pressure sensor. It exposes the sensor data through the Linux IIO (Industrial I/O) subsystem, making it easy to integrate with existing tools and libraries. It also supports IIO triggered buffers. 

This project was designed with full support for Device Tree Overlays, so it can be reconfigured to work on any platform by simply editing the overlay file. That being said, it was only tested on a Raspberry Pi 5.

## Overview

The [BMP280](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf) is a digital barometric pressure and temperature sensor that communicates over either SPI or I2C. This driver specifically supports the **I2C interface**.  It was developed and tested on a Raspberry Pi 5, but should be adaptable to other Linux systems with I2C support by edditing the Device Tree Overlay file.

Instead of a simple character device, this driver uses the [IIO framework](https://docs.kernel.org/driver-api/iio/index.html). This offers several advantages:

*   **Standardized Interface:**  IIO provides a consistent way to access sensor data, regardless of the underlying hardware.  Tools that understand IIO can automatically work with this driver.
*   **Sysfs Integration:** Sensor readings and configuration options are exposed through the sysfs filesystem, making them easily accessible from userspace.
*   **Data Buffering:** IIO supports data buffering, which can be useful for applications that need to collect data at high frequencies.
*   **Trigger Support:**  The IIO framework allows setting up triggers, often used in combination with data buffers. Triggers are events that cause the driver to read a sample of data and push it into a data buffer.

## Prerequisites

### **Raspberry Pi and Accessories**
Tested only on Raspberry Pi 5., although it should work on other platforms if you are familiar enough with that platform's Device Tree. It will also need a breadboard and four male to female jumper wires.

### **BMP280 Sensor**

You can easily find it online. It should look something like this:

TODO: Insert a photo.

We will use the I2C protocol, so we will only use the pins labeled SDA (that's the data line), SCL (clock), VCC (3.3v power), and GND (ground). If your BMP280 does not come with soldered pins, I found I could get away by just pinning it down into a breadboard with the male side of jumper wires.

TODO: Insert a photo

Ensure it's properly connected to your Raspberry Pi's I2C-1 bus. These should be the GPIO2 pin for data, and GPIO3 pin for clock. Note that there is an I2C-0 bus as well (GPIO0 and GPIO1), but that is typically reserved for the system. You can see more information on [this page](https://pinout.xyz/pinout/i2c). You also need pins for 3.3v power and ground. My arrangement looks like this:

TODO: Insert a photo

### System Setup

## Building the Driver

1.  **Clone the Repository:**

    ```bash
    git clone https://github.com/Leonardo-Blanger/bmp280-iio.git
    cd bmp280-iio
    ```

2.  **Build the Device Tree Overlay:**

	```bash
	make dtbo
	```
	This will generate our `bmp280-iio.dtbo` Device Tree blob overlay.

3.  **Build the Module:**

    ```bash
    make
    ```
    This will generate our `bmp280-iio.ko` kernel module.

## Installing and Loading the Driver

## Accessing Sensor Data

## Example

## License

This project is licensed under the MIT License - see the LICENSE file for details.

TODO: Create an MIT license file.

## References

TODO: Link all the resources I used and/or might be useful.
