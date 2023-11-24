#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Servo.h>


//=================================================
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo efector;
//================MPU=============================
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 accelgyroA;
MPU6050 accelgyroB(0x69);  // <-- use for AD0 high

int16_t axA, ayA, azA;
int16_t gxA, gyA, gzA;

int16_t axB, ayB, azB;
int16_t gxB, gyB, gzB;


int  data[6] = {0,0,0,0,0,0}; // variable global en la quye se almacenan los sensores y ends
//=================================================
#define SLEEP 8
#define MOTOR_STEPS 200
// Target RPM for X axis motor
#define MOTOR_X_RPM 80
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 80
// Target RPM for Y axis motor
#define MOTOR_Z_RPM 80
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
// 1=full step, 2=half step etc.
#define MICROSTEPS 16
// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X, SLEEP);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y, SLEEP);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z, SLEEP);
SyncDriver controller(stepperX, stepperY, stepperZ);

// === variables globales ==============================
bool motorFree = true;
int a[4] = { 0, 0, 0,0 };
byte inByte;

// ===0setup======================
void setup() {

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

pinMode(9, INPUT);
pinMode(10, INPUT);
pinMode(11, INPUT);


  Serial.begin(115200);
 
 Serial.println("===============ON===================");
 
  Serial.println("starting");
  efector.attach(12);  // attaches the servo on pin 12 - SpnEn on Shield
 
  delay(1000);
  gyroBegin();
  //lidarBegin();
 
  delay(500);
  //----------------------------------------
  stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
  stepperX.setEnableActiveState(LOW);
  stepperY.setEnableActiveState(LOW);
  stepperZ.setEnableActiveState(LOW);

  Serial.println("motorsOn");
  delay(1000);

  stepperX.enable();  // este tambien va?
  stepperY.enable();
  stepperZ.enable();

  stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperZ.setSpeedProfile(stepperZ.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

  Serial.println("setupFinished");
  delay(1000);
}
void loop() {
  unsigned long currentMillis =1;// millis();
  if (currentMillis % 10 == 0) { 
 
    Serial.print(data[0]);Serial.print(",");
    Serial.print(data[1]);Serial.print(",");
    Serial.print(data[2]);Serial.print(",");
    Serial.print(data[3]);Serial.print(",");
    Serial.print(data[4]);Serial.print(",");
    Serial.println(data[5]);
     // lectura cada n milisegundos para no bloquear
  }

  while (Serial.available()) {
    readSerial();
    Serial.println(a[1]);
    controller.startRotate(a[0], a[1], a[2]);  //este es el modo non Blocking
    motorFree = false;
  }

  unsigned wait_time_micros = controller.nextAction();  // motor control loop - send pulse and return how long to wait until next pulse

  if (wait_time_micros <= 0 && motorFree == false) {
    Serial.println("motorEnd");
    motorFree = true;
  }
   
   // readLidar();
   // gyroRead();
    efector.write(a[3]);

}

// =========== funciones=================================

// =================readSerial=========================
void readSerial() {
  if (Serial.available()) {
    String inByte = "";
    inByte = Serial.readStringUntil('\n');  // read data until newline

    // ordenes discretas por serial - son strings

    if (inByte == "off") {  // orden de prendido y apagado desde el serial
      stepperX.setEnableActiveState(HIGH);
      stepperY.setEnableActiveState(HIGH);
      stepperZ.setEnableActiveState(HIGH);
      stepperX.disable();
      stepperY.disable();
      stepperZ.disable();
    }
    if (inByte == "on") {
      stepperX.setEnableActiveState(LOW);
      stepperY.setEnableActiveState(LOW);
      stepperZ.setEnableActiveState(LOW);
      stepperX.enable();
      stepperY.enable();
      stepperZ.enable();
    }
    if (inByte == "speed:") {
      int acel[2] = { 0, 0 };
      acel[0] = getValue(inByte, ',', 1).toInt();  // Angulos a la var global
      acel[1] = getValue(inByte, ',', 2).toInt();
      acel[2] = getValue(inByte, ',', 3).toInt();

      stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, acel[0], acel[1]);
      stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, acel[0], acel[1]);
      stepperZ.setSpeedProfile(stepperZ.LINEAR_SPEED, acel[0], acel[1]);
    }
    if (inByte == "stop") {
      controller.startRotate(0, 0, 0);  //este es el modo non Blocking
      motorFree = false;

    }
    // si solo son numeros cuentan como coordenadas
    else {
      a[0] = getValue(inByte, ',', 0).toInt();  // Angulos a la var global
      a[1] = getValue(inByte, ',', 1).toInt();
      a[2] = getValue(inByte, ',', 2).toInt();
      a[3] = getValue(inByte, ',', 3).toInt();
    }
  }
  Serial.print("received: ");
  Serial.println(inByte);
}
//==========getValue=======================================
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
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
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1);
  };

  lox.startRangeContinuous();
  Serial.println("LIDAR ON");
}
//===========lidar Read=======================================

void readLidar() {
  Serial.println("····");
VL53L0X_RangingMeasurementData_t measure;
 
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  data[2]=measure.RangeMilliMeter;
 
}
//==============gyro Read=====================================
void gyroBegin() {
  Serial.println("Initializing MPUs");
  accelgyroA.initialize();
  accelgyroB.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyroA.testConnection() ? "MPU6050A connection successful" : "MPU6050A connection failed");
  Serial.println(accelgyroB.testConnection() ? "MPU6050B connection successful" : "MPU6050B connection failed");
}

//---------------------------------------------------------
void gyroRead() {  ///>>>>>>> lectura de giroscopios
  accelgyroA.getRotation(&gxA, &gyA, &gzA);
  accelgyroB.getRotation(&gxB, &gyB, &gzB);


  data[0]=gxA;
  data[1]=gxB;
}

///=====================interrupt pins read===============

void interruptors() {
  data[3]=digitalRead(9);
  data[4]=digitalRead(10);
  data[5]=digitalRead(11);
  }