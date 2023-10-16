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

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for X axis motor
#define MOTOR_X_RPM 60
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 60
#define SLEEP 8

// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 100
#define MOTOR_DECEL 50


// X motor
#define DIR_X 5
#define STEP_X 2

// Y motor
#define DIR_Y 6
#define STEP_Y 3

// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X, SLEEP);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y, SLEEP);

// Pick one of the two controllers below
// each motor moves independently, trajectory is a hockey stick
// MultiDriver controller(stepperX, stepperY);
// OR
// synchronized move, trajectory is a straight line
SyncDriver controller(stepperX, stepperY);

void setup() {
  /*
     Set target motors RPM.
  */
  stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
  stepperX.setEnableActiveState(LOW);
  stepperY.setEnableActiveState(LOW);
  stepperX.enable();
  stepperY.enable();
  stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  Serial.begin(9600);
  Serial.println("Setup Finish");


}
void loop() {
  Serial.println(millis()*0.001);
  controller.rotate(360 * 4, 100 ); //este es el modo non Blocking
  }
