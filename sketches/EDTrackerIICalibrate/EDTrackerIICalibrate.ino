//
//  Head Tracker Sketch
//

const char* PROGMEM infoString = "ED Tracker Calibration V2.0";

//
// Changelog:
// 2014-05-05 Initial Version

//

/* ============================================
EDTracker device code is placed under the MIT License
Copyright (c) 2014 Rob James

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

#define POLLMPU

#define EMPL_TARGET_ATMEGA328

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)     Serial.print (x);
#define DEBUG_PRINTLN(x)  Serial.println (x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#include <Wire.h>
#include <I2Cdev.h>
#include <EEPROM.h>


extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}

#define SDA_PIN 2
#define SCL_PIN 3
#define LED_PIN 17


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif



bool outputMode = true;

long gBias[3], aBias[3];

/* EEPROM Offsets for config and calibration stuff*/
#define EE_VERSION 0
// these are now longs (4 bytes)
#define EE_XGYRO 1
#define EE_YGYRO 5
#define EE_ZGYRO 9
#define EE_XACCEL 13
#define EE_YACCEL 17
#define EE_ZACCEL 21
//1 byte
#define EE_ORIENTATION 25
// 2 bytes
#define EE_XDRIFTCOMP 26

//Need some helper funct to read/write integers
void writeIntEE(int address, int value ) {
  EEPROM.write(address + 1, value >> 8); //upper byte
  EEPROM.write(address, value & 0xff); // write lower byte
}

int readIntEE(int address) {
  return (EEPROM.read(address + 1) << 8 | EEPROM.read(address));
}

void writeLongEE(int address,  long value) {
  for (int i =0; i<4;i++)
  {
  EEPROM.write(address++, value & 0xff); // write lower byte
  value = value >>8;
  }
}

long readLongEE(int address) {
    return ((long)EEPROM.read(address + 3) << 24 |
            (long)EEPROM.read(address + 2) << 16 |
            (long)EEPROM.read(address + 1) << 8 | 
            (long)EEPROM.read(address));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(LED_PIN, OUTPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  //
  //  // Disable internal I2C pull-ups
  //  cbi(PORTC, 4);
  //  cbi(PORTC, 5);

  // Gyro sensitivity:      2000 degrees/sec
  // Accel sensitivity:     2g
  // Gyro Low-pass filter:  42Hz
  // DMP Update rate:       100Hz
  loadBiases();
  initialize_mpu() ;
   mpu_set_dmp_state(1);
  //mpu_get_biases
//  enable_mpu();
}

/***************************************
* Invensense Hardware Abstracation Layer
***************************************/
unsigned char dmp_on;

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ    (100)


/****************************************
* Gyro/Accel/DMP Configuration
****************************************/

//unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
//unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
//unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000.  Default:  2000

long quat[4];
unsigned char more ;
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
boolean blinkState;
unsigned long tick = 0.0;

void loop()
{
  parseInput();

  if (millis() > tick)
  {
    tick = millis() + 200;
    digitalWrite(LED_PIN, blinkState);
  }

  int success = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

  if (outputMode)
  {
    Serial.print("0.0\t0.0\t0.0\t");

    //    Serial.print(accel[0] ); //
    //    Serial.print("\t");
    //    Serial.print(accel[1]); // Pitch
    //    Serial.print("\t");
    //    Serial.print(accel[2] ); // Roll
    //    Serial.print("\t");
    tripple(accel);

    //    Serial.print(gyro[0]); // Yaw
    //    Serial.print("\t");
    //    Serial.print(gyro[1] ); // Pitch
    //    Serial.print("\t");
    //    Serial.println(gyro[2]); // Roll
    tripple(gyro);
    Serial.println("");
  }
  return;
}

void parseInput()
{
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    byte command = Serial.read();

    if (command == 'S')
    {
      outputMode = false;
      Serial.println("S"); //silent
    }
    else if (command == 'V')
    {
      Serial.println("V"); //verbose
      //Serial.print("I\t");
      //Serial.println(infoString);
      outputMode = true;
    }
    else if (command == 'I')
    {
      Serial.print("I\t");
      Serial.println(infoString);
      loadBiases();
      mess("M\tGyro Bias ", gBias);

      //      Serial.print("M\tGyro Bias ");
      //      Serial.print(gBias[0]); Serial.print(" / ");
      //      Serial.print(gBias[1]); Serial.print(" / ");
      //      Serial.println(gBias[2]);

      mess("M\tAccel Bias ", aBias);
      //      Serial.print("M\tAccel Bias ");
      //      Serial.print(aBias[0]); Serial.print(" / ");
      //      Serial.print(aBias[1]); Serial.print(" / ");
      //      Serial.println(aBias[2]);
    }
    else if (command == 'B')
    {
      update_bias();
    }

    while (Serial.available() > 0)
      command = Serial.read();
  }
}


void  initialize_mpu() {

  mpu_init();

  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);

  dmp_load_motion_driver_firmware();

  //DEBUG_PRINT("Firmware Loaded ");
  dmp_set_orientation(B10001000);

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
                                DMP_FEATURE_SEND_RAW_GYRO ;//| DMP_FEATURE_GYRO_CAL; //no cal for bias!

  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);

  return ;
}


//void enable_mpu() {
//#ifndef POLLMPU
//  EICRB |= (1 << ISC60) | (1 << ISC61); // sets the interrupt type for EICRB (INT6)
//  EIMSK |= (1 << INT6); // activates the interrupt. 6 for 6, etc
//#endif
//
// // mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
//  dmp_on = 1;
//}


void update_bias()
{
  long gyroSum[3], accelSum[3];
  int samples = 10;
  unsigned short i;

  for (i = 0; i < 3; i++)
  {
    gyroSum[i] = 0;
    accelSum[i] = 0;
  }

  Serial.println("M\t Sampling...");
  //Serial.println(samples);

  for (int s = 0; s < samples; s++)
  {
    //physical values in Q16.16 format
    mpu_get_biases(gBias, aBias);
    
    for (i = 0; i < 3; i++)
    {
      gyroSum[i] += gBias[i];
      accelSum[i] += aBias[i];
    }
  }
  
  //Lets just store them in Q16.16 because we don't 
  //know what bias method 
  
  for (i = 0; i < 3; i++)
  {
    gBias[i] = (long)(((float)gyroSum[i])/(float)samples);
    aBias[i] = (long)(((float)accelSum[i])/(float)samples);
  }

  mess("M\tGyro Bias ", gBias);
  mess("M\tAccel Bias ", aBias);

//  unsigned short accel_sens;
  //mpu_get_accel_sens(&accel_sens);

//  int add = EE_XGYRO;
//  int add2 = EE_XACCEL;
  
//  for ( i=0;i<3;i++)
//  {
//    writeLongEE (EE_XGYRO +i*4, gBias[i]);
//    writeLongEE (EE_XACCEL+i*4, aBias[i]);
//  }
    
  writeLongEE (EE_XGYRO, gBias[0]);
  writeLongEE (EE_YGYRO, gBias[1]);
  writeLongEE (EE_ZGYRO, gBias[2]);
  writeLongEE (EE_XACCEL, aBias[0]);
  writeLongEE (EE_YACCEL, aBias[1]);
  writeLongEE (EE_ZACCEL, aBias[2]);
  
  return;
}

void tripple(short *v)
{
  //    Serial.print(v[0] ); //
  //    Serial.print("\t");
  //    Serial.print(v[1]); // Pitch
  //    Serial.print("\t");
  //    Serial.print(v[2] ); // Roll
  //    Serial.print("\t");
  for (int i = 0; i < 3; i++)
  {
    Serial.print(v[i] ); //
    Serial.print("\t");
  }
}

void mess(char *m, long*v)
{
  Serial.print(m);
  Serial.print(v[0]); Serial.print(" / ");
  Serial.print(v[1]); Serial.print(" / ");
  Serial.println(v[2]);
}


void loadBiases() {
  
  //int add = ;
//  for (int i=0;i < 3;i++)
//  {
//    gBias[i] = readLongEE (EE_XGYRO  + i*4);
//    aBias[i] = readLongEE (EE_XACCEL + i*4);
//  }
  
    gBias[0] = readLongEE (EE_XGYRO);
  gBias[1] = readLongEE (EE_YGYRO);
  gBias[2] = readLongEE (EE_ZGYRO);

  aBias[0] = readLongEE (EE_XACCEL);
  aBias[1] = readLongEE (EE_YACCEL);
  aBias[2] = readLongEE (EE_ZACCEL);
  
  return ;
}
