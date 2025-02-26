dt: bmp280-iio.dts
	dtc -@ -I dts -O dtb -o bmp280-iio.dtbo bmp280-iio.dts
clean:
	rm bmp280-iio.dtbo
