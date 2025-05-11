#include <linux/init.h>     
#include <linux/module.h>  
#include <linux/i2c.h>
#include <linux/device.h>   
#include <linux/fs.h>     
#include <linux/uaccess.h>   
#include <linux/delay.h>    
#include <linux/mutex.h>     
#include <linux/math64.h>   

'''
Họ và tên		            MSSV
Nguyễn Thanh Trọng  	    22146431
Huỳnh Xuân Trường  	        22146435
Tạ Văn Trường 		        22146439
Nguyễn Thái Tuấn	        22146443
'''

// Định nghĩa tên cho driver, class và device.
#define DRIVER_NAME "bmp180_driver"
#define CLASS_NAME "bmp180"
#define DEVICE_NAME "bmp180"

//===========//     REGISTER CALIBRATION     //===========//
// Địa chỉ các thanh ghi chứa dữ liệu hiệu chuẩn (calibration data) của cảm biến BMP180.
#define BMP180_CALIB_AC1     0xAA
#define BMP180_CALIB_AC2     0xAC
#define BMP180_CALIB_AC3     0xAE
#define BMP180_CALIB_AC4     0xB0
#define BMP180_CALIB_AC5     0xB2
#define BMP180_CALIB_AC6     0xB4
#define BMP180_CALIB_B1      0xB6
#define BMP180_CALIB_B2      0xB8
#define BMP180_CALIB_MB      0xBA
#define BMP180_CALIB_MC      0xBC
#define BMP180_CALIB_MD      0xBE

//===========//     REGISTER COMMAND     //===========//
// Địa chỉ các thanh ghi điều khiển và dữ liệu của BMP180.
#define BMP180_RESET_REG            0xE0        // Thanh ghi để gửi lệnh reset
#define BMP180_CONTROL_REG          0xF4        // Thanh ghi điều khiển, dùng để gửi lệnh bắt đầu đo nhiệt độ/áp suất
#define BMP180_TEMPERATURE_REG      0xF6        // Thanh ghi chứa giá trị nhiệt độ thô (MSB, LSB)
#define BMP180_PRESSURE_REG         0xF6        // Thanh ghi chứa giá trị áp suất thô (MSB, LSB, XLSB) (chung địa chỉ với nhiệt độ nhưng đọc khác)
#define BMP180_CHIPID_REG           0xD0        // Thanh ghi chứa ID của chip (mặc định là 0x55)

// Các giá trị lệnh để ghi vào thanh ghi điều khiển hoặc thanh ghi reset.
#define BMP180_READTEMP_CMD         0x2E        // Lệnh yêu cầu đo nhiệt độ
#define BMP180_READPRESSURE_CMD     0x34        // Lệnh yêu cầu đo áp suất (cần kết hợp với OSS)
#define BMP180_RESET_CMD            0xB6        // Lệnh reset cảm biến

//===========//     I2C ADDRESS     //===========//
// Địa chỉ I2C mặc định của cảm biến BMP180.
#define bmp180_i2c_address   0x77

// Định nghĩa các lệnh IOCTL (Input/Output Control)
#define BMP180_IOCTL_MAGIC 'b'
#define BMP180_IOCTL_SET_OSS            _IOW(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_GET_TEMPERATURE    _IOR(BMP180_IOCTL_MAGIC, 2, int32_t)
#define BMP180_IOCTL_GET_PRESSURE       _IOR(BMP180_IOCTL_MAGIC, 3, int32_t)
#define BMP180_IOCTL_RESET_MODULE       _IO(BMP180_IOCTL_MAGIC, 4) 


typedef struct
{
    s16 ac1;    s16 ac2;    s16 ac3;   
    u16 ac4;    u16 ac5;   u16 ac6;
    s16 b1;     s16 b2;     s16 mb;     
    s16 mc;     s16 md;
} bmp180_cal_data_t;

typedef enum 
{
    BMP180_ULTRA_LOW_POWER  = 0, 
    BMP180_STANDARD       = 1, 
    BMP180_HIGHRES        = 2, 
    BMP180_ULTRA_HIGHRES   = 3 
} bmp180_oss_mode_t;

// Biến toàn cục cho driver
static struct i2c_client *bmp180_client = NULL; 
static struct class* bmp180_class = NULL;       
static struct device* bmp180_device = NULL;     
static int major_number;                        
static bmp180_cal_data_t cal_data;              
static bmp180_oss_mode_t oss_mode = BMP180_STANDARD;
static s32 cal_data_b5;                         

static int bmp180_read_regs(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
    int ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf); 
    if(ret < 0) // Kiểm tra lỗi trả về từ hàm i2c_smbus_read_i2c_block_data
    {
        dev_err(&client->dev, "Read error %d bytes from reg 0x%02X: %d\n", len, reg, ret);
        return ret;
    }
    if(ret != len) // Kiểm tra xem số byte đọc được có đúng như yêu cầu không
    {
        dev_err(&client->dev, "Read %d bytes from reg 0x%02X, expected %d\n", ret, reg, len);
        return -EIO;
    }
    return 0; 
}

static int bmp180_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
    int ret = i2c_smbus_write_byte_data(client, reg, value);
    if(ret < 0) // Kiểm tra lỗi
    {
        dev_err(&client->dev, "Write error %d value to reg 0x%02X: %d\n", value, reg, ret);
        return ret;
    }
    return 0;
}

static int bmp180_read_s16(struct i2c_client *client, u8 reg, s16 *value) {
    u8 buf[2]; 
    int ret = bmp180_read_regs(client, reg, buf, 2); 
    if (ret < 0) return ret; // Trả về lỗi nếu đọc thất bại
    *value = (s16)((buf[0] << 8) | buf[1]); // Kết hợp MSB (buf[0]) và LSB (buf[1]) thành giá trị s16
    return 0; 
}

static int bmp180_read_u16(struct i2c_client *client, u8 reg, u16 *value)
{
    u8 buf[2];
    int ret = bmp180_read_regs(client, reg, buf, 2); 
    if(ret < 0) return ret; // Trả về lỗi nếu đọc thất bại
    *value = (u16)((buf[0] << 8) | buf[1]); // Kết hợp MSB và LSB thành giá trị u16
    return 0; 
}

static int bmp180_read_calibration_data(struct i2c_client *client)
{
    if(bmp180_read_s16(client, BMP180_CALIB_AC1, &cal_data.ac1) < 0) return -EIO;
    if(bmp180_read_s16(client, BMP180_CALIB_AC2, &cal_data.ac2) < 0) return -EIO;
    if(bmp180_read_s16(client, BMP180_CALIB_AC3, &cal_data.ac3) < 0) return -EIO;
    if(bmp180_read_u16(client, BMP180_CALIB_AC4, &cal_data.ac4) < 0) return -EIO;
    if(bmp180_read_u16(client, BMP180_CALIB_AC5, &cal_data.ac5) < 0) return -EIO;
    if(bmp180_read_u16(client, BMP180_CALIB_AC6, &cal_data.ac6) < 0) return -EIO;
    if(bmp180_read_s16(client, BMP180_CALIB_B1, &cal_data.b1) < 0) return -EIO;
    if(bmp180_read_s16(client, BMP180_CALIB_B2, &cal_data.b2) < 0) return -EIO;
    if(bmp180_read_s16(client, BMP180_CALIB_MB, &cal_data.mb) < 0) return -EIO;
    if(bmp180_read_s16(client, BMP180_CALIB_MC, &cal_data.mc) < 0) return -EIO;
    if(bmp180_read_s16(client, BMP180_CALIB_MD, &cal_data.md) < 0) return -EIO;

    dev_info(&client->dev, "Reading Calibration data Successfully\n"); 
    return 0;
}

static int bmp180_check_calibration_data(struct i2c_client *client)
{
    int cal_check = 0; //0: đúng, 1: sai
    // s16
    if(cal_data.ac1 == 0 || cal_data.ac1 == -1) cal_check = 1;
    if(cal_data.ac2 == 0 || cal_data.ac2 == -1) cal_check = 1;
    if(cal_data.ac3 == 0 || cal_data.ac3 == -1) cal_check = 1;
    if(cal_data.b1 == 0 || cal_data.b1 == -1)   cal_check = 1;
    if(cal_data.b2 == 0 || cal_data.b2 == -1)   cal_check = 1;
    if(cal_data.mb == -1)                       cal_check = 1;
    if(cal_data.mc == 0 || cal_data.mc == -1)   cal_check = 1;
    if(cal_data.md == 0 || cal_data.md == -1)   cal_check = 1;
    //  u16
    if(cal_data.ac4 == 0 || cal_data.ac4 == 0xFFFF) cal_check = 1;
    if(cal_data.ac5 == 0 || cal_data.ac5 == 0xFFFF) cal_check = 1;
    if(cal_data.ac6 == 0 || cal_data.ac6 == 0xFFFF) cal_check = 1;
    
    dev_info(&client->dev, "Checking Calibration data Successfully\n"); 
    if(cal_check == 1) 
    {
        dev_err(&client->dev, "Calibration data Invalid \n"); 
        printk(KERN_ERR   "-->  AC1: %d\n", cal_data.ac1);
        printk(KERN_ERR   "-->  AC2: %d\n", cal_data.ac2);
        printk(KERN_ERR   "-->  AC3: %d\n", cal_data.ac3);
        printk(KERN_ERR   "-->  AC4: %u\n", cal_data.ac4);
        printk(KERN_ERR   "-->  AC5: %u\n", cal_data.ac5);
        printk(KERN_ERR   "-->  AC6: %u\n", cal_data.ac6);
        printk(KERN_ERR   "-->  B1: %d\n", cal_data.b1);
        printk(KERN_ERR   "-->  B2: %d\n", cal_data.b2);
        printk(KERN_ERR   "-->  MB: %d\n", cal_data.mb);
        printk(KERN_ERR   "-->  MC: %d\n", cal_data.mc);
        printk(KERN_ERR   "-->  MD: %d\n", cal_data.md);
        return -EIO; 
    }
    return 0;
}

static int bmp180_read_raw_temperature(s32 *raw_temperature)
{
    int ret;
    u8 buf[2]; 
    if (!bmp180_client) return -ENODEV;

    // Ghi lệnh yêu cầu đo nhiệt độ (0x2E) vào thanh ghi điều khiển (0xF4).
    ret = bmp180_write_reg(bmp180_client, BMP180_CONTROL_REG, BMP180_READTEMP_CMD);
    if(ret < 0) return ret; // Trả về lỗi nếu ghi thất bại

    msleep(5); // Chờ 5ms cho quá trình chuyển đổi nhiệt độ hoàn tất

    ret = bmp180_read_regs(bmp180_client, BMP180_TEMPERATURE_REG, buf, 2);
    if(ret < 0) return ret; 

    *raw_temperature = ((buf[0] << 8) | buf[1]);
    return 0; 
}

static int bmp180_read_raw_pressure(s32 *raw_pressure)
{
    int ret;
    u8 buf[3]; //(MSB, LSB, XLSB)
    if (!bmp180_client) return -ENODEV; 

    // Ghi lệnh yêu cầu đo áp suất vào thanh ghi điều khiển (0xF4).
    // Lệnh là 0x34 cộng với (oss_mode << 6).
    ret = bmp180_write_reg(bmp180_client, BMP180_CONTROL_REG, BMP180_READPRESSURE_CMD + (oss_mode << 6));  
    if(ret < 0) return ret; 

    // Chờ cho quá trình chuyển đổi áp suất hoàn tất
    switch (oss_mode) {
        case BMP180_ULTRA_LOW_POWER:
            msleep(5); 
            break;
        case BMP180_HIGHRES:
            msleep(14); 
            break;
        case BMP180_ULTRA_HIGHRES:
            msleep(26); 
            break;
        default:
            msleep(8); 
            break;
    }

    ret = bmp180_read_regs(bmp180_client, BMP180_PRESSURE_REG, buf, 3);
    if(ret < 0) return ret; 

    *raw_pressure = (s32) (((u32)buf[0] << 16) | ((u32)buf[1] << 8) | (u32)buf[2]) >> (8 - oss_mode);
    return 0; 
}

static int bmp180_calculating_true_temperature(s32 raw_temp, s32 *true_temperature)
{
    s64 x1, x2; 
    x1 = ( ((s64)raw_temp - cal_data.ac6) * cal_data.ac5 ) >> 15;
    if ((x1 + cal_data.md) == 0) return -ERANGE; 
    x2 = div_s64( (s64)cal_data.mc << 11 , (x1 + cal_data.md) );
    cal_data_b5 = (s32)(x1 + x2);

    *true_temperature = (s32)(cal_data_b5 + 8) >> 4;
    return 0; 
}

static int bmp180_calculating_true_pressure(s32 raw_pressure, s32 *true_pressure)
{
    s64 x1_64, x2_64, x3_64, b3_64, b6_64, p_64, temp_64;
    s32 b3; 
    u32 b4, b7;

    b6_64 = cal_data_b5 - 4000;
    x1_64 = ( (s64)cal_data.b2 * ((b6_64 * b6_64) >> 12 ) ) >> 11;
    x2_64 = ( (s64)cal_data.ac2 * b6_64 ) >> 11;
    x3_64 = x1_64 + x2_64;
    b3_64 = ( (((s64)cal_data.ac1 * 4 + x3_64) << oss_mode) + 2) / 4;
    b3 = (s32)b3_64;

    x1_64 = ( (s64)cal_data.ac3 * b6_64 ) >> 13;
    x2_64 = ( (s64)cal_data.b1 * ((b6_64 * b6_64) >> 12)) >> 16;
    x3_64 = ( (x1_64 + x2_64) + 2 ) >> 2;

    temp_64 = (u64)cal_data.ac4 * (u64)(x3_64 + 32768); 
    b4 = (u32)(temp_64 >> 15);


    b7 = ( (u32)raw_pressure - b3) * (50000 >> oss_mode);
    
    if (b4 == 0) return -ERANGE;
    if(b7 < 0x80000000) p_64 = div_s64( (s64)b7 * 2 , (s32)b4 );
    else p_64 = div_s64( (s64)b7 , (s32)b4 ) * 2;


    x1_64 = (p_64 >> 8) * (p_64 >> 8);
    x1_64 = (x1_64 * 3038) >> 16;
    x2_64 = (-7357 * p_64) >> 16;

    p_64 += (x1_64 + x2_64 + 3791) >> 4;

    *true_pressure = (s32)p_64; 
    return 0;
}


static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0; 
    int oss_value; 
    s32 true_temp, true_press, raw_temp, raw_press;

    // Kiểm tra xem driver đã được khởi tạo và i2c_client có hợp lệ không
    if(!bmp180_client) 
    {
        printk(KERN_ERR   "Module does not exist\n"); 
        return -ENODEV;
    }

    // Xử lý dựa trên mã lệnh IOCTL
    switch (cmd) {
        case BMP180_IOCTL_SET_OSS: // Lệnh đặt chế độ OSS
            if (copy_from_user(&oss_value, (int __user *)arg, sizeof(oss_value))) {
                ret = -EFAULT; 
                break;
            }
            // Kiểm tra xem giá trị OSS có hợp lệ
            if (oss_value < BMP180_ULTRA_LOW_POWER || oss_value > BMP180_ULTRA_HIGHRES) {
                ret = -EINVAL;
                break;
            }
            oss_mode = (bmp180_oss_mode_t)oss_value; // Cập nhật chế độ OSS toàn cục
            dev_info(&bmp180_client->dev, "OSS mode set to %d\n", oss_mode);
            break;

        case BMP180_IOCTL_GET_TEMPERATURE: // Lệnh lấy nhiệt độ
            // Đọc nhiệt độ thô
            ret = bmp180_read_raw_temperature(&raw_temp);
            if(ret == 0)
            {
                // Tính toán nhiệt độ thực
                ret = bmp180_calculating_true_temperature(raw_temp, &true_temp);
            }
            if(ret == 0)
            {
                // Sao chép giá trị nhiệt độ thực ra user space
                if (copy_to_user((int32_t __user *)arg, &true_temp, sizeof(true_temp))) {
                    ret = -EFAULT; 
                }
            }
            break;

        case BMP180_IOCTL_GET_PRESSURE: // Lệnh lấy áp suất
            // Đọc nhiệt độ thô
            ret = bmp180_read_raw_temperature(&raw_temp);
            if(ret == 0) // Nếu thành công
            {
                // Tính nhiệt độ thực
                ret = bmp180_calculating_true_temperature(raw_temp, &true_temp); // true_temp không dùng trực tiếp ở đây, nhưng B5 được cập nhật
            }
            if(ret == 0) 
            {
                // Đọc áp suất thô
                ret = bmp180_read_raw_pressure(&raw_press);
            }
            if(ret == 0) 
            {
                // Tính áp suất thực
                ret = bmp180_calculating_true_pressure(raw_press, &true_press);
            }
            if(ret == 0) 
            {
                // Sao chép giá trị áp suất thực ra user space.
                if (copy_to_user((int32_t __user *)arg, &true_press, sizeof(true_press))) {
                    ret = -EFAULT; 
                }
            }
            break;

        case BMP180_IOCTL_RESET_MODULE:
            // Ghi lệnh reset (0xB6) vào thanh ghi reset (0xE0)
            ret = bmp180_write_reg(bmp180_client, BMP180_RESET_REG, BMP180_RESET_CMD);
            if (ret < 0) {
            } else {
                dev_info(&bmp180_client->dev, "BMP180 reset command sent.\n");
                msleep(10);
            }
            break;

        default: // Lệnh IOCTL không hợp lệ
            ret = -EINVAL; 
            break;
    }
    return ret; 
}


static int bmp180_open(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device opened\n"); 
    return 0;
}

static int bmp180_release(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device closed\n"); 
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,       
    .open = bmp180_open,        
    .unlocked_ioctl = bmp180_ioctl, 
    .release = bmp180_release,  
};

static int bmp180_i2c_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 chip_ID_raw; 
    int ret; 

    dev_info(&client->dev, "Probing for BMP180 device...\n");


    if (bmp180_client != NULL) {
        dev_err(&client->dev, "Driver Already Registered for another client -> Busy\n");
        return -EBUSY; 
    }
    bmp180_client = client; 


    // Thanh ghi Chip ID (0xD0) -> chip ID 0x55.
    chip_ID_raw = i2c_smbus_read_byte_data(client, BMP180_CHIPID_REG);
    if (chip_ID_raw < 0) { 
        dev_err(&client->dev, "Failed to read Chip ID: %d\n", chip_ID_raw);
        ret = chip_ID_raw; 
        goto probe_fail_client; 
    }
    if (chip_ID_raw != 0x55) {
        dev_err(&client->dev, "Chip ID Invalid (0x%02X != 0x55)\n", chip_ID_raw);
        ret = -ENODEV; 
        goto probe_fail_client; 
    }
    dev_info(&client->dev, "Matching Chip ID 0x55. BMP180 detected.\n");


    // Đọc dữ liệu hiệu chuẩn từ cảm biến.
    ret = bmp180_read_calibration_data(client);
    if (ret < 0) {
        dev_err(&client->dev, "Can't Read Calibration Data: %d\n", ret);
        goto probe_fail_client;
    }

    // Kiểm tra tính hợp lệ của dữ liệu hiệu chuẩn.
    ret = bmp180_check_calibration_data(client);
    if (ret < 0) {
        dev_err(&client->dev, "Calibration Data Invalid: %d\n", ret);
        goto probe_fail_client;
    }

    // Đăng ký character device để tạo device file trong /dev
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        dev_err(&client->dev, "Failed to register a major number: %d\n", major_number);
        ret = major_number; 
        goto probe_fail_client;
    }
    dev_info(&client->dev, "Registered character device with major number %d\n", major_number);

    // Tạo device class trong sysfs (/sys/class/CLASS_NAME)
    bmp180_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(bmp180_class)) { 
        ret = PTR_ERR(bmp180_class); 
        dev_err(&client->dev, "Failed to create device class: %d\n", ret);
        goto probe_fail_chrdev; 
    }
    dev_info(&client->dev, "Device class '%s' created successfully\n", CLASS_NAME);

    // Tạo device file trong /dev (/dev/bmp180)
    bmp180_device = device_create(bmp180_class, &client->dev, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(bmp180_device)) { 
        ret = PTR_ERR(bmp180_device);
        dev_err(&client->dev, "Failed to create device file: %d\n", ret);
        goto probe_fail_class; 
    }
    dev_info(&client->dev, "Device file '/dev/%s' created successfully\n", DEVICE_NAME);

    printk(KERN_INFO "BMP180 driver probed and initialized successfully\n");
    return 0; 

// goto để xử lý lỗi và dọn dẹp tài nguyên
probe_fail_class:
    class_destroy(bmp180_class); // Hủy class
    bmp180_class = NULL;
probe_fail_chrdev:
    unregister_chrdev(major_number, DEVICE_NAME); // Hủy đăng ký character device
    major_number = 0; 
probe_fail_client:
    bmp180_client = NULL; // Giải phóng con trỏ client
    return ret;
}

static void bmp180_i2c_driver_remove(struct i2c_client *client)
{
    printk(KERN_INFO "Removing BMP180 Driver...\n");
    
    if (bmp180_device) {
        device_destroy(bmp180_class, MKDEV(major_number, 0)); // Xóa device file
        bmp180_device = NULL;
    }
    if (bmp180_class) {
        class_destroy(bmp180_class); // Xóa device class
        bmp180_class = NULL;
    }
    if (major_number > 0) {
        unregister_chrdev(major_number, DEVICE_NAME); // Hủy đăng ký character device
        major_number = 0;
    }
    
    bmp180_client = NULL; // Đặt lại con trỏ client

    printk(KERN_INFO "BMP180 driver removed successfully\n");
}

static const struct of_device_id bmp180_of_match[] = {
    { .compatible = "bosch,bmp180", }, 
    { },
};


MODULE_DEVICE_TABLE(of, bmp180_of_match);

// Cấu trúc i2c_driver
static struct i2c_driver bmp180_i2c_driver = {
    .driver = {
        .name   = DRIVER_NAME, // Tên của driver
        .owner  = THIS_MODULE, // Chủ sở hữu module
        .of_match_table = of_match_ptr(bmp180_of_match), // Bảng matching
    },
    .probe      = bmp180_i2c_driver_probe,  
    .remove     = bmp180_i2c_driver_remove, 
};

static int __init bmp180_i2c_driver_init(void)
{
    printk(KERN_INFO "Initializing BMP180 I2C driver\n");
    return i2c_add_driver(&bmp180_i2c_driver); 
}


static void __exit bmp180_i2c_driver_exit(void)
{
    printk(KERN_INFO "Exiting BMP180 I2C driver\n");
    i2c_del_driver(&bmp180_i2c_driver); 
}

// Đăng ký hàm khởi tạo và hàm dọn dẹp module.
module_init(bmp180_i2c_driver_init);
module_exit(bmp180_i2c_driver_exit);


MODULE_AUTHOR("Truong");
MODULE_DESCRIPTION("BMP180 I2C Client Driver with IOCTL Interface"); 
MODULE_LICENSE("GPL"); 
