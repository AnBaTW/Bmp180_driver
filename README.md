## DEVELOPMENT TEAM MEMBERS ##
NAME: NGUYEN BA AN
MSSV: 22146075

NAME: TRAN DINH AN
MSSV: 22146069

<<---------------------------------------------------------------------------->>
                        BMP180_Driver Documentation

## OVERVIEW: 
The bmp180_driver is a lightweight software module designed to interface 
with the BMP180 digital sensor from Bosch, which measures atmospheric pressure and ambient temperature. 
This driver communicates with the sensor via the I2C protocol and is compatible with 
a wide range of microcontrollers such as STM32, AVR, ESP32, and more.

## USE CASES: 
-Environmental monitoring systems
-Altitude measurement base on asmospheric pressure data
-Weather stations in IoT applications
-Any embedded system requiring atmospheric data

## FEATURE:
-Automatically initialize and configure the BMP180 sensor.
-Read temperature and pressure values (in both raw and human-readable formats).
-Optionally calculate altitude based on pressure readings.
-Automatically handle internal calibration coefficients stored in the sensor’s EEPROM.

## SETUP DRIVER TUTORIAL
**Important: Ensure the I2C peripheral in raspi-config is enabled before proceeding with the following steps.

-Step 1: Go to /boot directory in your RaspberryPi
-Step 2: Check the .dtb file located in /boot. (Make sure that the filename matches the model/version of your Raspberry Pi.)
-Step 3: Run this command on terminal: sudo dtc -I dtb -O dts -o bcm2710-rpi-zero-2-w.dts bcm2710-rpi-zero-2-w.dtb . This action will create a .dts file
-Step 4: Open the .dts file, scroll until see aliase, find the i2c1 . Then copy numbers next to it. CTR + F and find the numbers. You'll be send to i2c1 node defination.
-Step 5: Add the following script into i2c1 node (Add it before the };):
        bmp180@77 {
            compatible = "bosch,bmp180";
            reg = <0x77>; // I2C address
        };
-Step 6: Save the .dts file and compile it back to .dtb by run : sudo dtc -I dts -O dtb -o bcm2710-rpi-zero-2-w.dtb bcm2710-rpi-zero-2-w.dts
-Step 7: cd into the directory contain file driver and Makefile
Step 8: Run this command on terminal: make

**Congratuation, BMP180_driver is ready for use!
## UNINSTALL: 
- In the directory contain Makefile, run this command on terminal: make clean

## API 
This driver provides only one ioctl() fucntion for communicate with BMP180

COMMAND LIST:
 - BMP180_IOCTL_READ_TEMP : Read calculated temperature data 
 - BMP180_IOCTL_READ_PRESS : Read calculated pressure data
 - BMP180_IOCTL_WRITE_SP : Write data (sampling mode) into BMP180

 EXAMPLE USE:
  //Read data
    if(ioctl(fd,BMP180_IOCTL_READ_PRESS,&data) < 0){
        printf("Failed to read the data\n");
        close(fd);
        return errno;
    }
    printf("Press: %ld\n", data);

 //Write data
    if(ioctl(fd,BMP180_IOCTL_WRITE_SP,&sm) < 0){
        printf("Failed to write the data\n");
        close(fd);
        return errno;
    }
## Limitation
- Haven't support alitude measurement yet. Please calculate it by the following formula.

 A = 44330*(1-pow((P/(101325)),1/5.255)); (P is pressure data)

 ## Reference
 https://www.alldatasheet.vn/datasheet-pdf/pdf/1132068/BOSCH/BMP180.html