////////////////////////////////////////////////////////////////
// WIRING OF THE HARDWARE
//
// Arduino - MPU1 - MPU2
// A4 - SDA1 - SDA2
// A5 - SCL1 - SCL2
// 3.3 - VCC1 - VCC2- ADO2
// GND - GND1 - GND2
//
///////////////////////////////////////////////////////////////

#include<Wire.h>
const int MPU_addr=0x68;
const int MPU_addr1=0x69;

double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double AcX1,AcY1,AcZ1,Tmp1,GyX1,GyY1,GyZ1;
uint32_t timer;

double compAngleX, compAngleY;
double compAngleX1, compAngleY1;

#define degconvert 57.2957786

void setup() {
// Set up MPU 6050:
Wire.begin();
#if ARDUINO >= 157
Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Wire.begin();
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);


Serial.begin(9600);
Serial.println("CLEARDATA");
Serial.println("LABEL, time, compAngleX, compAngleY, compAngleX1, compAngleY1");
Serial.println("Initializing I2C devices...");
delay(100);
//start a timer
timer = micros();
}

void loop() {
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
Wire.requestFrom(MPU_addr1,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
Tmp=Wire.read()<<8|Wire.read();
GyX=Wire.read()<<8|Wire.read();
GyY=Wire.read()<<8|Wire.read();
GyZ=Wire.read()<<8|Wire.read();
Wire.endTransmission(true);

double roll = atan2(AcY, AcZ)*degconvert;
double pitch = atan2(-AcX, AcZ)*degconvert;

double gyroXrate = GyX/131.0;
double gyroYrate = GyY/131.0;
compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll;
compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch;

Wire.beginTransmission(MPU_addr1);
Wire.write(0x3B);
AcX1=Wire.read()<<8|Wire.read();
AcY1=Wire.read()<<8|Wire.read();
AcZ1=Wire.read()<<8|Wire.read();
Tmp1=Wire.read()<<8|Wire.read();
GyX1=Wire.read()<<8|Wire.read();
GyY1=Wire.read()<<8|Wire.read();
GyZ1=Wire.read()<<8|Wire.read();
Wire.endTransmission(true);

double dt = (double)(micros() - timer) / 1000000;
double dt1 = (double)(micros() - timer) / 1000000;
timer = micros();

double roll1 = atan2(AcY1, AcZ1)*degconvert;
double pitch1 = atan2(-AcX1, AcZ1)*degconvert;
double gyroXrate1 = GyX1/131.0;
double gyroYrate1 = GyY1/131.0;


compAngleX1 = 0.99 * (compAngleX1 + gyroXrate1 * dt1) + 0.01 * roll1;
compAngleY1 = 0.99 * (compAngleY1 + gyroYrate1 * dt1) + 0.01 * pitch1;

Serial.print("DATA, TIME,"); Serial.print (compAngleX, 4);
Serial.print(","); Serial.print (compAngleY, 4);
Serial.print(","); Serial.print (compAngleX1, 4);
Serial.print(","); Serial.println (compAngleY1, 4);
}