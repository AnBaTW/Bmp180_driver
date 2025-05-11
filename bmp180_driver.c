#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/math64.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of.h> // Include for Device Tree

#define DRIVER_NAME "bmp180_driver"
#define DEVICE_NAME "bmp180"
#define CLASS_NAME  "bmp180"

#define BMP180_ADDR         0x77
#define MEASURE_CONTROL_REG 0xF4
#define TEMP_DATA           0x2E
#define PRESS_DATA          0x34
#define MSB_REG             0xF6
#define AC1_REG             0xAA

// IOCTL commands
#define BMP180_IOCTL_MAGIC 'm'
#define BMP180_IOCTL_READ_TEMP _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_PRESS _IOR(BMP180_IOCTL_MAGIC, 2, int)
#define BMP180_IOCTL_WRITE_SP _IOR(BMP180_IOCTL_MAGIC, 3, int)

/* Forward declarations */
static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id);
static void bmp180_remove(struct i2c_client *client);
static int bmp180_open(struct inode *inode, struct file *file);
static int bmp180_release(struct inode *inode, struct file *file);
static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

/* Device Tree match table */
static const struct of_device_id bmp180_of_match[] = {
    { .compatible = "bosch,bmp180_driver", }, // Corrected compatible string
    { },
};
MODULE_DEVICE_TABLE(of, bmp180_of_match);

/* I2C device ID table */
static const struct i2c_device_id bmp180_id[] = {
    {"bmp180", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, bmp180_id);

/* Define file operations structure first */
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = bmp180_open,
    .unlocked_ioctl = bmp180_ioctl,
    .release = bmp180_release,
};

static struct i2c_client *bmp180_client;
static struct class* bmp180_class = NULL;
static struct device* bmp180_device = NULL;
static int device_open_count = 0;
static int major_number;

static s16 AC1, AC2, AC3, B1, B2, MB, MC, MD;
static u16 AC4,AC5,AC6;
static u64 B4, B7;
static s64 X1, X2, X3, B5, B6, B3;
static s64 T;
static s64 P;
static u64 T_u;
static u16 frac;
static int sampling = 0;

static int bmp180_read_data(struct i2c_client *client, int sampling_mode)
{
    u8 buf[3];
    s64 UT;
    s64 UP;
    int ret;

    ret = i2c_smbus_write_byte_data(client, MEASURE_CONTROL_REG, TEMP_DATA);
    if (ret < 0) {
        printk(KERN_ERR "Failed to write BMP180\n");
        return ret;
    }
    usleep_range(4500, 5000);
    if (i2c_smbus_read_i2c_block_data(client, MSB_REG, sizeof(buf), buf) < 0) {
        printk(KERN_ERR "Failed to read temperature data\n");
        return -EIO;
    }
    UT = ((buf[0] << 8) + buf[1]);

    ret = i2c_smbus_write_byte_data(client, MEASURE_CONTROL_REG, PRESS_DATA);
    if (ret < 0) {
        printk(KERN_ERR "Failed to write BMP180\n");
        return ret;
    }
    switch (sampling_mode) {
    case 0:
        usleep_range(4500, 5000);
        break;
    case 1:
        usleep_range(7500, 8000);
        break;
    case 2:
        usleep_range(13500, 14000);
        break;
    case 3:
        usleep_range(25500, 26000);
        break;
    }
    if (i2c_smbus_read_i2c_block_data(client, MSB_REG, sizeof(buf), buf) < 0) {
        printk(KERN_ERR "Failed to read bmp180 data\n");
        return -EIO;
    }
    UP = ((buf[0] << 16) + (buf[1] << 8) + buf[2]) >> (8 - sampling_mode);

    X1 = (UT - AC6) * AC5 >> 15;
    X2 = div64_s64((MC << 11), (X1 + MD));
    B5 = X1 + X2;
    T = ((B5 + 8) >> 4);

    B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((s64)AC1 * 4 + X3) << sampling_mode) + 2) >> 2;

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (u64)(X3 + 32768)) >> 15;
    B7 = ((u64)UP - B3) * (50000 >> sampling_mode);
    if (B7 < 0x80000000)
        P = div64_s64((B7 << 1),B4);
    else
        P = (div64_s64(B7,B4)) << 1;

    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;
    P = P + ((X1 + X2 + 3791) >> 4);
    return 0;
}

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    
    int ret;
    u8 buf[22];
    bmp180_client = client;
    //printk(KERN_ERR "bmp180: probe function called!\n");
    // Create a char device
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ERR "Failed to register a major number\n");
        return major_number;
    }

    // Updated class_create 
    bmp180_class = class_create(THIS_MODULE,CLASS_NAME);
    if (IS_ERR(bmp180_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to register device class\n");
        return PTR_ERR(bmp180_class);
    }

    bmp180_device = device_create(bmp180_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(bmp180_device)) {
        class_destroy(bmp180_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to create the device\n");
        return PTR_ERR(bmp180_device);
    }
  
    printk(KERN_INFO "BMP180 driver installed\n");

    if (i2c_smbus_read_i2c_block_data(client, AC1_REG, sizeof(buf), buf) < 0) {
        printk(KERN_ERR "Failed to read bmp180 data\n");
        return -EIO;
    }
    AC1 = (buf[0] << 8) | buf[1];
    AC2 = (buf[2] << 8) | buf[3];
    AC3 = (buf[4] << 8) | buf[5];
    AC4 = (buf[6] << 8) | buf[7];
    AC5 = (buf[8] << 8) | buf[9];
    AC6 = (buf[10] << 8) | buf[11];
    B1 = (buf[12] << 8) | buf[13];
    B2 = (buf[14] << 8) | buf[15];
    MB = (buf[16] << 8) | buf[17];
    MC = (buf[18] << 8) | buf[19];
    MD = (buf[20] << 8) | buf[21];

    ret = bmp180_read_data(client, 1);
    if (ret < 0) {
        return ret;
    }
    //Change from 0.1 C to 1 C
    T_u = T;
    frac = do_div(T_u,10);
    printk(KERN_INFO "Temp: %lld.%u C, Press: %lld Pa \n", div64_s64(T,10),frac, P);
    return 0;
}

static void bmp180_remove(struct i2c_client *client)
{
    device_destroy(bmp180_class, MKDEV(major_number, 0));
    class_unregister(bmp180_class);
    class_destroy(bmp180_class);
    unregister_chrdev(major_number, DEVICE_NAME);

    printk(KERN_INFO "BMP180 driver removed\n");
}

static int bmp180_open(struct inode *inode, struct file *file)
{
    if (device_open_count > 0)
        return -EBUSY;

    device_open_count++;
    try_module_get(THIS_MODULE);
    return 0;
}

static int bmp180_release(struct inode *inode, struct file *file)
{
    device_open_count--;
    module_put(THIS_MODULE);
    return 0;
}

static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret;
    int data;
    switch (cmd) {
        case BMP180_IOCTL_WRITE_SP:
            if (copy_from_user(&sampling, (int __user *)arg, sizeof(sampling))) {
                return -EFAULT;
            }
            break;
        case BMP180_IOCTL_READ_PRESS:
            ret = bmp180_read_data(bmp180_client, sampling);
            if (ret < 0) {
                return ret;
            }
            data = P;
            break;
        case BMP180_IOCTL_READ_TEMP:
            ret = bmp180_read_data(bmp180_client, sampling);
            if (ret < 0) {
                return ret;
            }
            data = T;
            break;
        default:
            return -EINVAL;
    }
    if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
        return -EFAULT;
    }

    return 0;
}

/* I2C driver structure */
static struct i2c_driver bmp180_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = bmp180_of_match,
    },
    .probe = bmp180_probe,
    .remove = bmp180_remove,
    .id_table = bmp180_id,
};

static int __init bmp180_init(void)
{
    printk(KERN_INFO "Initializing BMP180 driver\n");
    return i2c_add_driver(&bmp180_driver);
}

static void __exit bmp180_exit(void)
{
    printk(KERN_INFO "Exiting BMP180 driver\n");
    i2c_del_driver(&bmp180_driver);
}

module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_AUTHOR("Ignotus");
MODULE_DESCRIPTION("BMP180 I2C Client Driver");
MODULE_LICENSE("GPL");