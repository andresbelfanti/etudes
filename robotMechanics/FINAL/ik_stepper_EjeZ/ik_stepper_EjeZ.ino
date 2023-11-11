
#include <Servo.h>
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include <Adafruit_MPU6050.h>
#include "Adafruit_VL53L0X.h"


// ---- MPU6050-------------------------

/*
The pin "AD0" selects between I2C address 0x68 and 0x69.
 That makes it possible to have two of these sensors in a project. Most breakout
 boards have a pullup or pulldown resistor to make AD0 default low or high.
  Connect AD0 to GND or 3.3V for the other I2C address. */

Adafruit_MPU6050 mpu0;
Adafruit_MPU6050 mpu1;
Adafruit_Sensor *mpu_temp0, *mpu_accel0, *mpu_gyro0;
Adafruit_Sensor *mpu_temp1, *mpu_accel1, *mpu_gyro1;

#include "definitions.h"
#include "funcs.h"

//==================SEtUP =========================
void setup() {
  Serial.begin(9600);

  stepperInit();
  lidarBegin();
  //mpuTest();

  Serial.println("<pocho is ready>");
  delay(2000);
}
//==============LOOP==============================

void loop() {

  receiver();
    //anglesRead();
    //ikSolver(coordes[0], coordes[1], coordes[2]);
    moveToAngle(coordes[0], coordes[1], coordes[2]);
    coordes[0] = 0;
    coordes[1] = 0;
    coordes[2] = 0;

   
  //lidarRead();
 
  // if(stopAll == true){
  //   Serial.println("STOP!!");
  //   //break; // si se da la orden de parar llega hasta acá?
  // }

    unsigned wait_time_micros = controller.nextAction();  // motor control loop
 
  if (wait_time_micros <= 0) {
    Serial.println("motorEnd");
    //anglesRead();
    //moveToAngle(angulos[0], angulos[1], angulos[2]);
    }

}
