/*
   Multi-motor control (experimental)

   Move two or three motors at the same time.
   This module is still work in progress and may not work well or at all.

   Copyright (C)2017 Laurentiu Badea

   This file may be redistributed under the terms of the MIT license.
   A copy of this license has been included with this distribution in the file LICENSE.
*/
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

//Ensable/Disable
#define SLEEP 8

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for X axis motor
#define MOTOR_X_RPM 120
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 120
// Target RPM for Y axis motor
#define MOTOR_Z_RPM 120


// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 200
#define MOTOR_DECEL 200


// X motor
#define DIR_X 5
#define STEP_X 2

// Y motor
#define DIR_Y 6
#define STEP_Y 3
// Z motor
#define DIR_Z 7
#define STEP_Z 4

// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X, SLEEP);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y, SLEEP);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z, SLEEP);



SyncDriver controller(stepperX, stepperY, stepperZ);
bool motorFree = true;

int a[3] = {360*2,360*4,360*8};

void setup() {
  /*
     Set target motors RPM.
  */
  stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);

  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
  stepperX.setEnableActiveState(LOW);
  stepperY.setEnableActiveState(LOW);
  stepperZ.setEnableActiveState(LOW);

  stepperX.enable();
  stepperY.enable();
  stepperZ.enable();

  stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperZ.setSpeedProfile(stepperY.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

  Serial.begin(9600);
  Serial.println("Setup Finish");

    controller.startRotate(a[0], a[1] , a[2] ); //este es el modo non Blocking

}
void loop() {
  while (Serial.available()) {
  //  String a = Serial.readString().toInt(); // read the incoming data as string

    controller.startRotate(a[0], a[1] , a[2] ); //este es el modo non Blocking
  }


  // motor control loop - send pulse and return how long to wait until next pulse
  unsigned wait_time_micros = controller.nextAction();

  if (wait_time_micros <= 0) {
    Serial.println("motorEnd");
    motorFree = true;
  }



}
