#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "MPU6050.h"


//=================================================
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

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
int a[5] = { 0, 0, 0, 0, 0};
int datas[6] = { 0, 0, 0, 0, 0, 0 };
byte inByte;
bool stop = false;
// ===0setup======================
void setup() {
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  Serial.println("starting");
  delay(1000);
  lidarBegin();
  delay(1000);

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

  if (millis() % 250 == 0) {  // lectura cada n milisegundos para no bloquear
    interruptors();
    Serial.print(datas[0]);
    Serial.print(",");
    Serial.print(datas[1]);
    Serial.print(",");
    Serial.print(datas[2]);
    Serial.print(",");
    Serial.print(datas[3]);
    Serial.print(",");
    Serial.print(datas[4]);
    Serial.print(",");
    Serial.print(datas[5]);
    Serial.print(",");
    Serial.println(datas[6]);
  }

  while (Serial.available()) {
    readSerial();
    controller.startRotate(a[0]*4, a[1]*3.75, a[2]*3.75);
    stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, a[3], a[4]);
    stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, a[3], a[4]);
    stepperZ.setSpeedProfile(stepperZ.LINEAR_SPEED, a[3], a[4]);
      //este es el modo non Blocking
    motorFree = false;
    stop = false;
    //int a[3] = {0,0,0};
  }
  unsigned wait_time_micros;

  if (stop == false) {
    wait_time_micros = controller.nextAction();  // motor control loop - send pulse and return how long to wait until next pulse
  }
  if (wait_time_micros <= 0 && motorFree == false) {
    Serial.println("motorEnd");
    motorFree = true;
  }
}
// =========== funciones=================================

// =================readSerial=========================
void readSerial() {
  if (Serial.available()) {
    String inByte = "";
    inByte = Serial.readStringUntil('\n');  // read data until newline
    Serial.print("data received: ");


    if (inByte == "off") {  // orden de prendido y apagado desde el serial
      digitalWrite(8, HIGH);
      Serial.println("off");
    }
    if (inByte == "on") {
      digitalWrite(8, LOW);
      Serial.println("on");
    }
    if (inByte == "stop") {  // stop
      stop = true;
      Serial.println("stop");
    }
   
    // si solo son numeros cuentan como coordenadas
    else {
      a[0] = getValue(inByte, ',', 0).toInt();  // Angulos a la var global
      a[1] = getValue(inByte, ',', 1).toInt();
      a[2] = getValue(inByte, ',', 2).toInt();
      a[3] = getValue(inByte, ',', 3).toInt();
      a[4] = getValue(inByte, ',', 4).toInt();
      Serial.print("coords");
    }
  }
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

void lidarRead() {
  if (lox.isRangeComplete()) {
    datas[3] = lox.readRange();
  }
}
//==============gyro Read=====================================
void gyroBegin() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyroA.initialize();
  accelgyroB.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyroA.testConnection() ? "MPU6050A connection successful" : "MPU6050A connection failed");
  Serial.println(accelgyroB.testConnection() ? "MPU6050B connection successful" : "MPU6050B connection failed");
}
void gyroRead() {  ///>>>>>>> lectura de giroscopios
  accelgyroA.getMotion6(&axA, &ayA, &azA, &gxA, &gyA, &gzA);
  accelgyroB.getMotion6(&axB, &ayB, &azB, &gxB, &gyB, &gzB);
  // buscar codigo nuevo. usar otra libreria
}
///=====================interrupt pins read===============

void interruptors() {
  datas[4] = digitalRead(9);
  datas[5] = digitalRead(10);
  datas[6] = digitalRead(11);
  if ((datas[4] == 1) || (datas[5] == 1) || (datas[6] == 1)) {
    stop = true;
    Serial.println("STOP");
  }
}