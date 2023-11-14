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

#define SLEEP 8

#define MOTOR_STEPS 200
// Target RPM for X axis motor
#define MOTOR_X_RPM 80
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 80
// Target RPM for Y axis motor
#define MOTOR_Z_RPM 1800

#define MOTOR_ACCEL 20
#define MOTOR_DECEL 20
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
#define MICROSTEPS 16

// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X, SLEEP);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y, SLEEP);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z, SLEEP);
SyncDriver controller(stepperX, stepperY, stepperZ);

// === variables globales ==============================
bool motorFree = true;
int a[3] = {0,0,0};

// ===0setup======================
void setup() {
  Serial.begin(9600);
   Serial.println("starting");
  delay(1000);
  lidarBegin();
  delay(1000);

  stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
  stepperX.setEnableActiveState(LOW);
  stepperY.setEnableActiveState(LOW);
  stepperZ.setEnableActiveState(LOW);
  
  Serial.println("motorsOn");
  delay(1000);

  stepperX.enable(); // este tambien va?
  stepperY.enable();
  stepperZ.enable();

  stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperZ.setSpeedProfile(stepperY.LINEAR_SPEED, 200, 200);
  
  Serial.println("setupFinished");
  delay(1000);
}
void loop() {
  int a[3] = {0,0,0};

  if(millis()%10==0){// lectura cada n milisegundos para no bloquear
   try{
    gyroRead();
    lidarRead();
    interrupts();
    catch(ExceptionIo as e){
      Serial.println(e);
    }
   }
  }

  while (Serial.available()) {
    readSerial();
    controller.startRotate(a[0], a[1] , a[2] ); //este es el modo non Blocking
    motorFree = false;
  }

  unsigned wait_time_micros = controller.nextAction();  // motor control loop - send pulse and return how long to wait until next pulse

  if (wait_time_micros <= 0 && motorFree==false) {
    Serial.println("motorEnd");
    motorFree = true;
  }
}
// =========== funciones=================================

// =================readSerial=========================
void readSerial(){
  if (Serial.available()) {
    String inByte = "";
    inByte = Serial.readStringUntil('\n'); // read data until newline
   
  // ordenes discretas por serial - son strings
    if (inByte == "off") { // orden de prendido y apagado desde el serial
   stepperX.setEnableActiveState(HIGH);
   stepperY.setEnableActiveState(HIGH);
   stepperZ.setEnableActiveState(HIGH);
    }
    if (inByte == "on") {
    stepperX.setEnableActiveState(HIGH);
    stepperY.setEnableActiveState(HIGH);
    stepperZ.setEnableActiveState(HIGH);
    }
  // si solo son numeros cuentan como coordenadas
    else {
      a[0] = getValue(inByte, ',', 0).toInt(); // Angulos a la var global
      a[1] = getValue(inByte, ',', 1).toInt();
      a[2] = getValue(inByte, ',', 2).toInt();
    }
  } 
  Serial.Println("received"); 

}
//==========getValue=======================================
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//=======Lidar Begin==========================================
void lidarBegin() {
  if(!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1);
   };
    
  lox.startRangeContinuous();
    Serial.println("LIDAR ON");

  }
//===========lidar Read=======================================

void lidarRead() {
  if (lox.isRangeComplete()) {
    Serial.print("lidar:  ");
    Serial.println(lox.readRange());
  }
}
//==============gyro Read=====================================

void gyroRead() {  ///>>>>>>> lectura de giroscopios

  //  /* Get a new normalized sensor event */
  sensors_event_t gyro0;
  sensors_event_t gyro1;
  mpu_gyro0->getEvent(&gyro0);
  mpu_gyro1->getEvent(&gyro1);
  // deberia leer 2 giroscopios
  float x0 = gyro0.gyro.x * RAD_TO_DEG;  // cambiar a grados -- observar que data entrega
  float x1 = gyro1.gyro.x * RAD_TO_DEG - x0;

  Serial.println("gyro: ");
  Serial.print(x0);
  Serial.print(", ");  
  Serial.print(x1);
  Serial.println();
}

///=====================interrupt pins read===============

void interrupts(){
// acá tambien una orden local, por si se desconecta del usb y sigue andando 
// aca digital read de los pines de final d ecarrera

}