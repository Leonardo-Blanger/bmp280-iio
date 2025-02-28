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

<img src="images/sensor-closeup.jpg" height=512>

We will use the I2C protocol, so we will only use the pins labeled SDA (that's the data line), SCL (clock), VCC (3.3v power), and GND (ground). If your BMP280 does not come with soldered pins, I found I could get away by just pinning it down into a breadboard with the male side of jumper wires.

<img src="images/sensor-connections.jpg" height=512>

Ensure it's properly connected to your Raspberry Pi's I2C-1 bus. These should be the GPIO2 pin for data, and GPIO3 pin for clock. Note that there is an I2C-0 bus as well (GPIO0 and GPIO1), but that is typically reserved for the system. You can see more information on [this page](https://pinout.xyz/pinout/i2c). You also need pins for 3.3v power and ground. My arrangement looks like this:

<img src="images/full-arrangement.jpg" height=512>

## System Setup

* **I2C Enabled:** Make sure I2C is enabled on your Raspberry Pi. You can do this using `raspi-config` (Interface Options -> I2C -> Enable) or by manually editing `/boot/firmware/config.txt` and ensuring `dtparam=i2c_arm=on` is present (and not commented out). Reboot after enabling.

* **`i2c-dev` module**: This is useful in order to make sure you wired up your sensor the right way. The `i2c-dev` kernel module gives userspace access to I2C. For each I2C bus on the system, this module exposes a `/dev/ic2-X` chrdev file. It comes with Raspberry Pi's OS, and you can load it with:

    ```bash
    sudo modprobe i2c-dev
    ```

    To make it load automatically on boot, add `i2c-dev` to `/etc/modules`. On my installation, it's already there.

	For our example, after enabling the bus and loading this module, you should have an `/dev/i2c-1` file.

* **`i2c-tools`**: The i2c-tools package provides a set of command-line utilities that make it easier to communicate with I2C devices from userspace using the `i2c-dev` exposed files. You can install it on the Raspberry Pi OS using:

	```bash
	sudo apt update
	sudo apt install i2c-tools
	```

	If you wired everything correctly, running `i2c-detect -l` will display all the I2C buses on the system. The main bus exposed through the GPIO pins should show up as `i2c-1`. If not, you probably didn't enable it (see above), or didn't reboot after enabling it.

	Finally, running `i2c-detect -y 1` will show a table of devices sitting on our I2C bus. There should only be one filled entry in this table, with the address number associated with our BMP280 sensor. For instance, my sensor has address 0x76. Take note of this number. 

## Building the Driver

1.  **Linux Kernel Headers:** You will need the kernel headers for your running kernel. Install them using:

    ```bash
    sudo apt update
    sudo apt install raspberrypi-kernel-headers
    ```

	You should have a directory with your kernel headers and config files at `/usr/lib/modules/$(uname -r)/build`. This is enough to build kernel modules, we don't need the full kernel sources.

2.  **Build Dependencies:** You will need a set of dependencies to build kernel modules and device tree overlays. On the Raspberry Pi OS, you can install them with:

	```bash
	sudo apt update
	sudo apt install make build-essential device-tree-compiler
	```

3.  **Clone the Repository:**

    ```bash
    git clone https://github.com/Leonardo-Blanger/bmp280-iio.git
    cd bmp280-iio
    ```

4.  **Build the Device Tree Overlay:**

	```bash
	make dtbo
	```
	This will generate our `bmp280-iio.dtbo` Device Tree blob overlay.

5.  **Build the Module:**

    ```bash
    make modules
    ```
    This will generate our `bmp280-iio.ko` kernel module.

	Note: Running `make` withou arguments builds both the dt overlay, and the module. Running `make clean` deletes all build files from your repo.

## Installing and Loading the Driver

1.  **Copy the Module (Optional but Recommended):**

    ```bash
    sudo make modules_install
    ```
    This will place your module in the correct kernel modules directory. You might need to manually run `sudo depmod` afterwards. For some reason, on my installation, it does not run automatically.

2.  **Load the Module:**

    ```bash
    sudo insmod bmp280-iio.ko
    ```
    Alternatively, if you used `make modules_install`, you can load it with:

    ```bash
    sudo modprobe bmp280-iio
    ```

3.  **Verify Installation:**
    *   Check for the IIO device:

        ```bash
        ls /sys/bus/iio/devices/
        ```
        You should see a device named something like `iio:deviceX` (where X is a number). This represents your BMP280 sensor. You can check for the device name using:

		```bash
		cat '/sys/bus/iio/devices/iio:device<X>/name'
		```
		The name should be `bmp280-iio`. This is useful in case you already had other IIO device directories.

    *   Check for kernel messages:

        ```bash
        dmesg | grep bmp280-iio
        ```
        You should see the following messages indicating the driver loaded and detected the sensor:

		TODO: Write this once we settle on the log format.

4.  **Automatic Loading on Boot (Optional):**
    *   To automatically load the driver on boot, add `bmp280-iio` to the `/etc/modules` file.
	*   TODO: Find out how to make the dtoverlay load on boot.

## Accessing Sensor Data

## Example

## License

This project is licensed under the MIT License - see the LICENSE file for details.

TODO: Create an MIT license file.

## References

TODO: Link all the resources I used and/or might be useful.
