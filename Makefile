obj-m += src/bmp280-iio.o

all: dtbo module

dtbo: bmp280-iio.dts
	dtc -@ -I dts -O dtb -o bmp280-iio.dtbo bmp280-iio.dts
	echo "Built Device Tree Overlay"
module:
	make -C /usr/lib/modules/$(shell uname -r)/build M=$(PWD) modules
	echo "Built Kernel Modules"
clean:
	rm bmp280-iio.dtbo
	make -C /usr/lib/modules/$(shell uname -r)/build M=$(PWD) clean
