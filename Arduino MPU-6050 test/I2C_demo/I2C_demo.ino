// Steven Keyes
// 3 January 2014
//
// This is a test of talking to the MPU6000
// over I2C using an arduino nano

#include "Wire.h"
// with AD0 set to GND, the device address is 0x68
// which is b1101000; if AD0 were 1, this would
// instead be b1101001. This is a 7 bit address
// which the master will follow with a 8th R/W bit
// see section 9.2 of the product specification
#define mpu6050address 0x68

void setup()
{
  Wire.begin(); // start the I2C bus
  Serial.begin(9600); // start the serial bus
}

void wakeup()
{
  // move the register pointer to the right register
  Wire.beginTransmission(mpu6050address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
}

int readZacceleration()
{
  // move the register pointer to the right register
  Wire.beginTransmission(mpu6050address);
  Wire.write(0x3F); // this is ACCEL_ZOUT_H
  Wire.endTransmission();
  
  // get the data from the mpu6050
  Wire.requestFrom(mpu6050address, 1);
  byte a = Wire.read();
  
  // move the register pointer to the right register
  Wire.beginTransmission(mpu6050address);
  Wire.write(0x40); // this is ACCEL_ZOUT_L
  Wire.endTransmission();
  
  // get the data from the mpu6050
  Wire.requestFrom(mpu6050address, 1);
  byte b = Wire.read();
  
  int value = a*256 + b;
  
  return value;
}

void loop()
{
  wakeup();
  Serial.print("Z accelerometer raw value = ");
  Serial.println(readZacceleration());
  delay(250);
}
