
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "KalmanFilter.h"

#include <SoftwareSerial.h>

SoftwareSerial gySerial(2, 3); // RX, TX

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 accelgyro(0x69); // <-- use for AD0 high
// MPU6050 accelgyro(0x68, &Wire1); // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
// #define OUTPUT_BINARY_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;
byte c0[8] = {
    B01110,
    B01010,
    B11111,
    B10101,
    B11111,
    B10101,
    B11111,
    B10101};
byte c1[8] = {
    B00100,
    B11111,
    B10110,
    B11111,
    B10110,
    B11111,
    B10110,
    B11001};
byte c3[8] = {
    B10100,
    B01111,
    B00110,
    B11111,
    B00110,
    B01111,
    B11100,
    B01001};

KalmanFilter kalmanX(0.001, 0.003, 0.03), kalmanY(0.001, 0.003, 0.03), kalmanZ(0.001, 0.003, 0.03);
float angleX = 0, angleY = 0, angleZ = 0;

float prevTime;
int dd = 0;

float Roll, Pitch, Yaw;
unsigned char buff[8], counter = 0;
bool imu_updated = false;

void setup()
{
  Wire.begin();
  Serial.begin(9600);

  gySerial.begin(9600);
  // 将GY-25设置
  gySerial.write(0xA5);
  gySerial.write(0x52);
  delay(10);

  mpu.initialize();
  if (!mpu.testConnection())
  {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }
  lcd.init(); // initialize the lcd
              //  lcd.init();
  lcd.backlight();

  prevTime = millis();
}

void loop()
{
  dd++;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float dt = (millis() - prevTime) / 1000.0;
  prevTime = millis();

  // 将原始数据转换为物理单位
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // 假设 100Hz 采样率，时间间隔为 0.01 秒
  // float dt = 0.001;

  // 使用加速度计数据计算 X 方向的角度
  float accelAngleX = atan(accelY / sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
  float accelAngleY = atan(accelX / sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
  float accelAngleZ = angleZ + gyroZ * dt;
  // atan(sqrt(accelY * accelY + accelX * accelX) / accelZ) * 180 / PI;

  // 使用卡尔曼滤波器计算 X 方向的角度
  angleX = kalmanX.getAngle(accelAngleX, gyroX, dt);
  angleY = kalmanY.getAngle(accelAngleY, gyroY, dt);
  angleZ = kalmanZ.getAngle(accelAngleZ, gyroZ, dt);

  // 请求GY-25角度数据
  // gySerial.write(0xA5);
  // gySerial.write(0x51);
  // delay(20);

  // 读取GY-25数据
  readGYAngle();

  Serial.print(angleX);
  Serial.print(",");
  Serial.print(angleY);
  Serial.print(",");
  Serial.print(angleZ);
  Serial.print(',');
  Serial.print(Pitch);
  Serial.print(",");
  Serial.print(Roll);
  Serial.print(",");
  Serial.println(Yaw);

  // if(dd>10){

  // lcd.clear();
  char line1[17];
  lcd.setCursor(0, 0);
  sprintf(line1, "%4d,%4d,%4d", (int)angleX, (int)angleY, (int)angleZ);
  lcd.print(line1);

  lcd.setCursor(0, 1);
  sprintf(line1, "%4d,%4d,%4d", (int)Pitch, (int)Roll, (int)Yaw);
  lcd.print(line1);

  dd = 0;
  //}

  // delay(10);
}

bool readGYAngle()
{
  while (gySerial.available())
  {
    buff[counter] = (unsigned char)gySerial.read();
    if (counter == 0 && buff[0] != 0xAA)
      while (gySerial.read() != 0xAA)
        ;
    counter++;
    if (counter == 8) // package is complete
    {
      counter = 0;
      if (buff[0] == 0xAA && buff[7] == 0x55) // data package is correct
      {
        Yaw = (int16_t)(buff[1] << 8 | buff[2]) / 100.00;
        Pitch = (int16_t)(buff[3] << 8 | buff[4]) / 100.00;
        Roll = (int16_t)(buff[5] << 8 | buff[6]) / 100.00;
        // return true;
      }
    }
  } // End of while
  // return false;
}
