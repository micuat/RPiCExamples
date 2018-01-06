
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <math.h>

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <arpa/inet.h>
#include <sys/select.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>

#include "tinyosc.h"

#define BNO_I2C_ADDR 0x28

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

#define MPR_I2C_ADDR 0x5A

#define MPR121_ID 0x00

#define MPR121_CHIP_ID_ADDR 0x00

#define MPR121_I2CADDR_DEFAULT 0x5A
#define MPR121_TOUCHSTATUS_L   0x00
#define MPR121_FILTDATA_0L     0x04
#define MPR121_BASELINE_0      0x1E
#define MPR121_MHDR            0x2B
#define MPR121_NHDR            0x2C
#define MPR121_NCLR            0x2D
#define MPR121_FDLR            0x2E
#define MPR121_MHDF            0x2F
#define MPR121_NHDF            0x30
#define MPR121_NCLF            0x31
#define MPR121_FDLF            0x32
#define MPR121_NHDT            0x33
#define MPR121_NCLT            0x34
#define MPR121_FDLT            0x35
#define MPR121_TOUCHTH_0       0x41
#define MPR121_RELEASETH_0     0x42
#define MPR121_DEBOUNCE        0x5B
#define MPR121_CONFIG1         0x5C
#define MPR121_CONFIG2         0x5D
#define MPR121_ECR             0x5E
#define MPR121_SOFTRESET       0x80

int main(int argc, char *argv[])
{
   int devBno;
   int devMpr;
   if (wiringPiSetup() == -1) {
       printf("wiringPi init failed\n");
       return -1;
   }

   pullUpDnControl(8, PUD_UP);
   pullUpDnControl(9, PUD_UP);
   if ( (devBno = wiringPiI2CSetup(BNO_I2C_ADDR)) == -1) {
       printf("error init i2c at 0x%04x\n",BNO_I2C_ADDR);
       return -1;
   }
   else
       printf("i2c init at 0x%04x OK, devBno = %i\n", BNO_I2C_ADDR, devBno);


   // sanity check
   char id = (char) wiringPiI2CReadReg8(devBno, BNO055_CHIP_ID_ADDR);
   printf("chip id: 0x%02X (expected: 0x%02X)\n", id, BNO055_ID);

   // set config mode
   wiringPiI2CWriteReg8(devBno, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
   usleep(30 * 1000);

   // reset
   wiringPiI2CWriteReg8(devBno, BNO055_SYS_TRIGGER_ADDR, 0x20);
   while (wiringPiI2CReadReg8(devBno, BNO055_CHIP_ID_ADDR) != BNO055_ID)
   {
      usleep(10 * 1000);
   }
   usleep(50 * 1000);

   // normal power mode
   wiringPiI2CWriteReg8(devBno, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
   usleep(10 * 1000);

   wiringPiI2CWriteReg8(devBno, BNO055_PAGE_ID_ADDR, 0);

   wiringPiI2CWriteReg8(devBno, BNO055_SYS_TRIGGER_ADDR, 0x00);
   usleep(10 * 1000);

   wiringPiI2CWriteReg8(devBno, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
   usleep(30 * 1000);
   usleep(20 * 1000);

   // MPR121 init
   if ( (devMpr = wiringPiI2CSetup(MPR_I2C_ADDR)) == -1) {
    printf("error init i2c at 0x%04x\n",MPR_I2C_ADDR);
      return -1;
   }
   else
      printf("i2c init at 0x%04x OK, devMpr = %i\n", MPR_I2C_ADDR, devMpr);
   // https://github.com/adafruit/Adafruit_CircuitPython_MPR121/blob/master/adafruit_mpr121.py
   wiringPiI2CWriteReg8(devMpr, MPR121_SOFTRESET, 0x63);
   wiringPiI2CWriteReg8(devMpr, MPR121_ECR, 0x00);
   // skip read CONFIG2
   int touch = 12, release = 6;
   for(int i = 0; i < 12; i ++) {
      wiringPiI2CWriteReg8(devMpr, MPR121_TOUCHTH_0 + 2*i, touch);
      wiringPiI2CWriteReg8(devMpr, MPR121_RELEASETH_0 + 2*i, release);
   }
   wiringPiI2CWriteReg8(devMpr, MPR121_MHDR, 0x01);
   wiringPiI2CWriteReg8(devMpr, MPR121_NHDR, 0x01);
   wiringPiI2CWriteReg8(devMpr, MPR121_NCLR, 0x0E);
   wiringPiI2CWriteReg8(devMpr, MPR121_FDLR, 0x00);
   wiringPiI2CWriteReg8(devMpr, MPR121_MHDF, 0x01);
   wiringPiI2CWriteReg8(devMpr, MPR121_NHDF, 0x05);
   wiringPiI2CWriteReg8(devMpr, MPR121_NCLF, 0x01);
   wiringPiI2CWriteReg8(devMpr, MPR121_FDLF, 0x00);
   wiringPiI2CWriteReg8(devMpr, MPR121_NHDT, 0x00);
   wiringPiI2CWriteReg8(devMpr, MPR121_NCLT, 0x00);
   wiringPiI2CWriteReg8(devMpr, MPR121_FDLT, 0x00);

   wiringPiI2CWriteReg8(devMpr, MPR121_DEBOUNCE, 0x00);
   wiringPiI2CWriteReg8(devMpr, MPR121_CONFIG1, 0x10);
   wiringPiI2CWriteReg8(devMpr, MPR121_CONFIG2, 0x20);

   wiringPiI2CWriteReg8(devMpr, MPR121_ECR, 0x8F);

   // osc init
   // open a socket to listen for datagrams (i.e. UDP packets) on port 9000
   const int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
   printf("socket: %d\n", fd);
   fcntl(fd, F_SETFL, O_NONBLOCK); // set the socket to non-blocking
   struct sockaddr_in sin;
   memset((char *)&sin, 0, sizeof(struct sockaddr_in));
   sin.sin_family = AF_INET;
   sin.sin_port = htons(9000);
   int atonres = inet_aton("127.0.0.1", &sin.sin_addr);
   printf("aton result: %d\n", atonres);
   int bindres = bind(fd, (struct sockaddr *) &sin, sizeof(struct sockaddr_in));
   printf("bind result: %d\n", bindres);

   char buffer[2048]; // declare a 2Kb buffer to read packet data into

   printf("Starting write tests:\n");
   int len = 0;
   len = tosc_writeMessage(buffer, sizeof(buffer), "/address", "s",
      "hello world");
   tosc_printOscBuffer(buffer, len);
   //printf("sent len: %d\n", send(fd, buffer, len, 0));
   int sentlen = sendto(fd, buffer, len, 0, (struct sockaddr *)&sin, sizeof(struct sockaddr_in));
   printf("sent len: %d\n", sentlen);
   printf("done.\n");

   while (1)
   {
      int wl = (int)wiringPiI2CReadReg8(devBno, BNO055_QUATERNION_DATA_W_LSB_ADDR);
      int wm = (int)wiringPiI2CReadReg8(devBno, BNO055_QUATERNION_DATA_W_MSB_ADDR);
      int xl = (int)wiringPiI2CReadReg8(devBno, BNO055_QUATERNION_DATA_X_LSB_ADDR);
      int xm = (int)wiringPiI2CReadReg8(devBno, BNO055_QUATERNION_DATA_X_MSB_ADDR);
      int yl = (int)wiringPiI2CReadReg8(devBno, BNO055_QUATERNION_DATA_Y_LSB_ADDR);
      int ym = (int)wiringPiI2CReadReg8(devBno, BNO055_QUATERNION_DATA_Y_MSB_ADDR);
      int zl = (int)wiringPiI2CReadReg8(devBno, BNO055_QUATERNION_DATA_Z_LSB_ADDR);
      int zm = (int)wiringPiI2CReadReg8(devBno, BNO055_QUATERNION_DATA_Z_MSB_ADDR);

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

      //printf("roll = %7.2f, pitch = %7.2f, yaw = %7.2f\n", roll, pitch, yaw);

      int touched = (int)wiringPiI2CReadReg8(devMpr, MPR121_TOUCHSTATUS_L);
      printf("touchedVal: %d\n", touched);


      int len = 0;
      char v0[50];
      char v1[50];
      char v2[50];
      snprintf(v0, 50, "%f", roll);
      snprintf(v1, 50, "%f", pitch);
      snprintf(v2, 50, "%f", yaw);
      len = tosc_writeMessage(buffer, sizeof(buffer), "/n/pd/bno", "iii",
         (int)(x*100), (int)(y*100), (int)(z*100));
      //len = tosc_writeMessage(buffer, sizeof(buffer), "/n/pd/bno", "sss",
      //   (double)roll, (double)pitch, (double)yaw);
      //   v0, v1, v2);
      //tosc_printOscBuffer(buffer, len);
      sendto(fd, buffer, len, 0, (struct sockaddr *)&sin, sizeof(struct sockaddr_in));
   }
   return 0;
}
