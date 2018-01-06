
#include <stdio.h>
#include <unistd.h>

#include <math.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <arpa/inet.h>
#include <sys/select.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>

#include "tinyosc.h"

#define I2C_ADDR 0x28

#define BNO055_ID 0xA0

#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0X20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0X21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0X22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0X23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0X24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0X25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0X26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0X27
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F

#define OPERATION_MODE_CONFIG 0x00
#define OPERATION_MODE_NDOF 0x0C

#define POWER_MODE_NORMAL 0x00
#define POWER_MODE_LOWPOWER 0x01
#define POWER_MODE_SUSPEND 0x02


int main(int argc, char *argv[])
{
   int dev;
   if (wiringPiSetup() == -1) {
       printf("wiringPi init failed\n");
       return -1;
   }

   pullUpDnControl(8, PUD_UP);
   pullUpDnControl(9, PUD_UP);
   if ( (dev = wiringPiI2CSetup(I2C_ADDR)) == -1) {
       printf("error init i2c at 0x%04x\n",I2C_ADDR);
       return -1;
   }
   else
       printf("i2c init at 0x%04x OK, dev = %i\n", I2C_ADDR, dev);


   // sanity check
   char id = (char) wiringPiI2CReadReg8(dev, BNO055_CHIP_ID_ADDR);
   printf("chip id: 0x%02X (expected: 0x%02X)\n", id, BNO055_ID);

   // set config mode
   wiringPiI2CWriteReg8(dev, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
   usleep(30 * 1000);

   // reset
   wiringPiI2CWriteReg8(dev, BNO055_SYS_TRIGGER_ADDR, 0x20);
   while (wiringPiI2CReadReg8(dev, BNO055_CHIP_ID_ADDR) != BNO055_ID)
   {
      usleep(10 * 1000);
   }
   usleep(50 * 1000);

   // normal power mode
   wiringPiI2CWriteReg8(dev, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
   usleep(10 * 1000);

   wiringPiI2CWriteReg8(dev, BNO055_PAGE_ID_ADDR, 0);

   wiringPiI2CWriteReg8(dev, BNO055_SYS_TRIGGER_ADDR, 0x00);
   usleep(10 * 1000);

   wiringPiI2CWriteReg8(dev, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
   usleep(30 * 1000);
   usleep(20 * 1000);

   // osc init
   char buffer[2048]; // declare a 2Kb buffer to read packet data into

   printf("Starting write tests:\n");
   int len = 0;
   len = tosc_writeMessage(buffer, sizeof(buffer), "/address", "fs",
      1.0f, "hello world");
   tosc_printOscBuffer(buffer, len);
   // send(socket_fd, buffer, len, 0);
   printf("done.\n");

   while (1)
   {
      int wl = (int)wiringPiI2CReadReg8(dev, BNO055_QUATERNION_DATA_W_LSB_ADDR);
      int wm = (int)wiringPiI2CReadReg8(dev, BNO055_QUATERNION_DATA_W_MSB_ADDR);
      int xl = (int)wiringPiI2CReadReg8(dev, BNO055_QUATERNION_DATA_X_LSB_ADDR);
      int xm = (int)wiringPiI2CReadReg8(dev, BNO055_QUATERNION_DATA_X_MSB_ADDR);
      int yl = (int)wiringPiI2CReadReg8(dev, BNO055_QUATERNION_DATA_Y_LSB_ADDR);
      int ym = (int)wiringPiI2CReadReg8(dev, BNO055_QUATERNION_DATA_Y_MSB_ADDR);
      int zl = (int)wiringPiI2CReadReg8(dev, BNO055_QUATERNION_DATA_Z_LSB_ADDR);
      int zm = (int)wiringPiI2CReadReg8(dev, BNO055_QUATERNION_DATA_Z_MSB_ADDR);

      float w = ((float)(wl | (wm << 8)) - 32768.0f) / 32768.0f;
      float x = ((float)(xl | (xm << 8)) - 32768.0f) / 32768.0f;
      float y = ((float)(yl | (ym << 8)) - 32768.0f) / 32768.0f;
      float z = ((float)(zl | (zm << 8)) - 32768.0f) / 32768.0f;
      //printf("w = %6.2f x = %6.2f y = %6.2f z = %6.2f\n", w, x, y, z);

      float sinr = +2.0 * (w * x + y * z);
      float cosr = +1.0 - 2.0 * (x * x + y * y);
      float roll = atan2f(sinr, cosr) / M_PI * 180;
      float sinp = +2.0 * (w * y - z * x);
      float pitch;
      if(fabs(sinp) >= 1)
         pitch = copysignf(M_PI * 0.5f, sinp);
      else
         pitch = asinf(sinp);
      pitch = pitch / M_PI * 180;
      float siny = 2 * (w * z + x * y);
      float cosy = 1 - 2 * (y * y + z * z);
      float yaw = atan2f(siny, cosy) / M_PI * 180;

      printf("roll = %7.2f, pitch = %7.2f, yaw = %7.2f\n", roll, pitch, yaw);
   }
   return 0;
}
