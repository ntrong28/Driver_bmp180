Linux kernel driver for BMP180 sensor

NOTE: Thực hiện các bước chuẩn bị trước khi cài đặt driver cho cảm biến BMP180.

1.Cập nhật hệ thống:

sudo apt update
sudo apt upgrade -y

2.Cài đặt kernel headers:

sudo apt install raspberrypi-kernel-headers -y

3.Sau khi cài đặt xong, khởi động lại Raspberry Pi:

sudo reboot

4.Kiểm tra version kernel và headers:

uname -r
ls /lib/modules/$(uname -r)/build

Thư mục build phải tồn tại thì mới có thể build driver kernel.

5.Bật giao tiếp I2C:

sudo raspi-config

Chọn:

Interfacing Options

I2C

Enable

Sau đó reboot:

sudo reboot

6.Kiểm tra thiết bị BMP180 kết nối đúng:

sudo apt install -y i2c-tools
i2cdetect -y 1

Thiết bị thường sẽ hiện tại địa chỉ 0x77 trên ma trận.

7.Cài file Device Tree Overlay cho driver BMP180:
Tạo file bmp180_overlay.dts: xem code kèm trong file
Biên dịch overlay:
dtc -@ -I dts -O dtb -o bmp180_overlay.dtbo bmp180_overlay.dts
Sao chép file overlay vào thư mục boot:
sudo cp bmp180_overlay.dtbo /boot/overlays/

8.Chỉnh sửa file cấu hình:
sudo nano /boot/config.txt
Thêm dòng ở cuối file config.txt
dtoverlay=bmp180_overlay
Khởi động lại Raspberry Pi:
sudo reboot

9.Biên dịch driver kernel BMP180:
Tạo Makefile: xem code kèm trong file
Dùng lệnh:

make: Gọi Makefile của kernel, chỉ cho nó biết nơi tìm mã nguồn của module của bạn (M=$(shell pwd)), và yêu cầu nó biên dịch các module (modules)Sau khi hoàn tất, sẻ có file .ko để chèn vào kernel 
make install: Dùng để cài đặt module kernel đã biên dịch vào kernel.
make uninstall: Dùng để gỡ cài đặtmodule kernel đã biên dịch vào kernel.
make clean: Dùng để dọn dẹp các file được tạo ra trong quá trình biên dịch.

Lưu ý: 
	- Dùng lệnh: ls -l /sys/bus/i2c/devices/1-0077/driver kiểm tra hiện tại địa chỉ 0x77 đang được gán cho driver mặt định của máy (pmb280) hay không!
	- Nếu có dùng lệnh: "make unload" để gỡ bỏ module kernel khỏi kernel để gán địa chỉ đó cho driver đang tạo.

10.Ứng dụng
Dùng lệnh:
make gcc: để biên dịch file bmp180_run.c
sudo ./run: để chạy chương trình
