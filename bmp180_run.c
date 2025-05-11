#include <stdio.h> 
#include <stdlib.h> 
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h> 
#include <errno.h> 
#include <math.h>

'''
Họ và tên		            MSSV
Nguyễn Thanh Trọng  	    22146431
Huỳnh Xuân Trường  	        22146435
Tạ Văn Trường 		        22146439
Nguyễn Thái Tuấn	        22146443
'''

#define DEVICE_PATH "/dev/bmp180" 

#define BMP180_IOCTL_MAGIC 'b' // Số "magic" để phân biệt các lệnh IOCTL của driver này.
#define BMP180_IOCTL_SET_OSS            _IOW(BMP180_IOCTL_MAGIC, 1, int) // Lệnh IOCTL để cài đặt chế độ oversampling (OSS)
#define BMP180_IOCTL_GET_TEMPERATURE    _IOR(BMP180_IOCTL_MAGIC, 2, int32_t) // Lệnh IOCTL để đọc nhiệt độ
#define BMP180_IOCTL_GET_PRESSURE       _IOR(BMP180_IOCTL_MAGIC, 3, int32_t) // Lệnh IOCTL để đọc áp suất
#define BMP180_IOCTL_RESET_MODULE       _IO(BMP180_IOCTL_MAGIC, 4) // Lệnh IOCTL để reset cảm biến

#define PRESSURE_AT_SEA_LEVEL_PA    100500.0 // Áp suất chuẩn tại mực nước biển tính bằng Pascal (Pa) -> Áp suất tham chiếu 

/*
Hàm tính toán độ cao so với mực nước biển
p: áp suất đo được từ cảm biến (Pa)
p0: áp suất tham chiếu tại mực nước biển (Pa)
*/
float bmp180_calculate_altitude(int32_t p, float p0)
{
    if( p <= 0 || p0 <= 0) return NAN; // Kiểm tra đầu vào
    float ps = (float)p / p0; 
    ps = 44330.0 * (1.0 - powf(ps, 1.0 / 5.255));
    return ps;
}

/*
Hàm tính toán áp suất tại mực nước biển dựa vào áp suất đo được
p: áp suất đo được từ cảm biến (Pa)
altitude: độ cao hiện tại so với mực nước biển (m)
Trả về áp suất tại mực nước biển tính (Pa)
*/
float bmp180_calculate_pressure_sea_level(int32_t p, float altitude)
{
    if( p <= 0) return NAN; // Kiểm tra áp suất đầu vào
    float mau_so = 1.0 - (altitude/44330.0); 
    if(mau_so <= 0) return NAN; 
    float pasl = (float)p / mau_so;
    return pasl; //pressure at sea level
}

int main() {
    int fd, ret; 
    int oss_mode = 3; //Đặt chế độ oss mode            
    int32_t temp, press; 
    float altitude, sealevel;


    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) { 
        perror("Failed to open the device\n"); 
        return errno; 
    }

    if(oss_mode < 0 || oss_mode > 3){
        printf("OSS Invalid -> OSS is a number between 0 - 3\n"); // Báo lỗi oss_mode không hợp lệ
        close(fd);
        return 1; 
    }

    // In ra chế độ OSS đang được sử dụng.
    if(oss_mode == 0)       printf("Pressure sampling accuracy mode: Ultra Low Power\n");
    else if(oss_mode == 1)  printf("Pressure sampling accuracy mode: Standard\n");
    else if(oss_mode == 2)  printf("Pressure sampling accuracy mode: High Resolution\n");
    else if(oss_mode == 3)  printf("Pressure sampling accuracy mode: Ultra High Resolution\n");


    // Gửi lệnh IOCTL để cài đặt chế độ OSS cho driver
    ret = ioctl(fd, BMP180_IOCTL_SET_OSS, &oss_mode);
    if (ret < 0) { 
        printf("Failed to set OSS mode. Error: %d\n", ret);
        //perror("ioctl BMP180_IOCTL_SET_OSS");
        printf("Module using Standard mode by default (or last successfully set mode)\n"); 
    }

    // Đọc nhiệt độ từ cảm biến bằng IOCTL
    ret = ioctl(fd, BMP180_IOCTL_GET_TEMPERATURE, &temp);
    if (ret < 0) { 
        perror("Failed to read Temperature"); 
        close(fd); 
        return errno; 
    }
    printf("Temperature: %.2f °C\n", (float)temp / 10.0); // In nhiệt độ. Giá trị 'temp' từ driver là nhiệt độ đơn vị 0.1°C.

    // Đọc áp suất từ cảm biến bằng IOCTL
    ret = ioctl(fd, BMP180_IOCTL_GET_PRESSURE, &press);
    if (ret < 0) {
        perror("Failed to read Pressure");
        close(fd);
        return errno;
    }
    printf("Pressure: %.2f hPa\n", (float)press / 100.0); // In áp suất. Giá trị 'press' từ driver là Pa, chia 100 để đổi sang hPa

    // Tính toán và đọc độ cao.
    altitude = bmp180_calculate_altitude(press, PRESSURE_AT_SEA_LEVEL_PA);
    if(!isnan(altitude)) // Kiểm tra xem kết quả có phải là NAN không
        printf("Altitude: %.2f m (relative to P0=%.1f Pa)\n", altitude, PRESSURE_AT_SEA_LEVEL_PA); // In độ cao
    else
        printf("Failed to calculate Altitude (pressure data might be invalid)\n"); // Thông báo lỗi

    // Tính toán và đọc áp suất tại mực nước biển.
    sealevel = bmp180_calculate_pressure_sea_level(press, altitude);
    if(!isnan(sealevel)) // Kiểm tra xem kết quả có phải là NAN không
        printf("Calculated Sea Level Pressure: %.2f hPa (based on current P and calculated altitude)\n", sealevel / 100.0); // In áp suất tại mực nước biển (đổi sang hPa).
    else
        printf("Failed to calculate Sea Level Pressure\n"); // Thông báo lỗi.

    close(fd);
    return 0;
}