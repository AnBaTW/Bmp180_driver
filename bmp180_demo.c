#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#define DEVICE_PATH "/dev/bmp180"
#define BMP180_IOCTL_MAGIC 'm'
#define BMP180_IOCTL_READ_TEMP _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_PRESS _IOR(BMP180_IOCTL_MAGIC, 2, int)
#define BMP180_IOCTL_WRITE_SP _IOR(BMP180_IOCTL_MAGIC, 3, int)

int sampling_mode = 0;
int main(){
    int fd;
    int data;
    fd = open(DEVICE_PATH,O_RDONLY);
    if (fd < 0){
        printf("Failed to open the device\n");
        return errno;
    }
    //wirte data
    if(ioctl(fd,BMP180_IOCTL_WRITE_SP,&sampling_mode) < 0){
        printf("Failed to write the data\n");
        close(fd);
        return errno;
    }
    //Read data
    if(ioctl(fd,BMP180_IOCTL_READ_PRESS,&data) < 0){
        printf("Failed to read the data\n");
        close(fd);
        return errno;
    }
    printf("Press: %ld\n", data);
    close(fd);
    return 0;
}