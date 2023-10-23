
#include <Servo.h>
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "funcs.h"
#include "definitions.h"
// ---- MPU6050-------------------------

/*
The pin "AD0" selects between I2C address 0x68 and 0x69.
 That makes it possible to have two of these sensors in a project. Most breakout
 boards have a pullup or pulldown resistor to make AD0 default low or high.
  Connect AD0 to GND or 3.3V for the other I2C address. */

#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu0;
Adafruit_MPU6050 mpu1;
Adafruit_Sensor *mpu_temp0, *mpu_accel0, *mpu_gyro0;
Adafruit_Sensor *mpu_temp1, *mpu_accel1, *mpu_gyro1;


// -- variables------------------------
double x = 0;
double y = 0;
double z = 0;
const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data
int coordes[3] = { 0, 0, 0 };
bool motorFree = true; 

//==================SEtUP =========================
void setup() {
 Serial.begin(9600);

 stepperInit();
 mpuTest();

  Serial.println("<pocho is ready>");
    delay(2000);
}
//==============LOOP==============================

void loop() {
  receiver(); // devuelve a coordes[x,y,z] // activa motorFree = true

  if(motorFree == true){
      moveToAngle(moveToPos(coordes[0], coordes[1], coordes[2]));
      motorFree = false;
  }

    unsigned wait_time_micros = controller.nextAction(); // motor control loop 

    if (wait_time_micros <= 0) {
        Serial.println("motorEnd");
        anglesRead();
        motorFree = true;
    }



   
}
