#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
//#include "MPU6050.h"
#include <VL53L0X.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <Servo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//==================================================
VL53L0X sensor;
MPU6050 mpu;
MPU6050 mpuB(0x69);  // <-- use for AD0 highh

Servo myservo;

//=================================================
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//================MPU=============================
bool gyro0 = true;
bool gyro1 = true;

#define INTERRUPT_PIN 2   // use pin 2 on Arduino Uno & most boards
#define INTERRUPT_PINB 3  // use pin 2 on Arduino Uno & most boards

bool blinkState = false;
bool blinkStateB = false;

// MPU control/status vars A=======================================
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//====//B///===================================================

bool dmpReadyB = false;   // set true if DMP init was successful
uint8_t mpuIntStatusB;    // holds actual interrupt status byte from MPU
uint8_t devStatusB;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSizeB;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCountB;      // count of all bytes currently in FIFO
uint8_t fifoBufferB[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion qB;         // [w, x, y, z]         quaternion container
VectorFloat gravityB;  // [x, y, z]            gravity vector
float yprB[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//=======STEPPERS================================
#define SLEEP 8

#define MOTOR_STEPS 200
// Target RPM for axis motor
#define MOTOR_X_RPM 80
#define MOTOR_Y_RPM 80
#define MOTOR_Z_RPM 80

#define MOTOR_ACCEL 20
#define MOTOR_DECEL 20
// X Y Z PINS motor
#define DIR_X 5
#define STEP_X 2
#define DIR_Y 6
#define STEP_Y 3
#define DIR_Z 7
#define STEP_Z 4

// If microstepping is set externally, make sure this matches the selected mode 1=full step, 2=half step etc.
#define MICROSTEPS 16

// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X, SLEEP);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y, SLEEP);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z, SLEEP);
SyncDriver controller(stepperX, stepperY, stepperZ);

// === variables globales ==============================
bool motorFree = true;
int a[5] = { 0, 0, 0, 0, 0 };
int datas[7] = { 0, 0, 0, 0, 0, 0, 0 };
byte inByte;
bool stop = false;
int pos = 0;

// ===============MPUS ROUTINES===========================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
volatile bool mpuInterruptB = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReadyB() {
  mpuInterruptB = true;
}


// =============================SETUP======================
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  //-----interrupts pins ----------------
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
 //---------------------------------------
  Serial.begin(115200);
  while (!Serial) {delay(1);}
  Serial.println("starting");
  //----------------------------------------
  stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
  stepperX.setEnableActiveState(LOW);
  stepperY.setEnableActiveState(LOW);
  stepperZ.setEnableActiveState(LOW);

  Serial.println("motorsOn");
  delay(500);
  stepperX.enable();  
  stepperY.enable();
  stepperZ.enable();

  stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperZ.setSpeedProfile(stepperZ.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

  myservo.attach(12);
  servoTest();

  Serial.println("motores check");
  delay(1000);
    // ------- checkeo de posicion --------------------------

  Serial.println("prueba de posicion");
  int counterpos = 0;
  bool fail = false;

  while ((digitalRead(9) != 1 )|| (counterpos < 500)) {
    controller.startRotate(0, -3, 0);

    for(int i = 0; i < 100; i++) {
      controller.nextAction();
      delay(1);
    }
  }

  if (counterpos > 500) {
    Serial.println("error del hombro o error del interruptor 1");
    fail = true;
  }
  
  counterpos = 0;
  while ((digitalRead(11) != 1)|| (counterpos<500)) {
    controller.startRotate(0, 0, 10);
    for(int i = 0; i < 100; i++) {
      controller.nextAction();
      delay(1);
    }
  }
  if (counterpos > 500) {
    Serial.println("error del codo o error del interruptor 3");
    fail = true;
  }

  if (fail == true) { // FALTA FUNCION PARA REPETIR EL CHEQUEO DE POSICION O COMENZAR DE NUEVO
    Serial.println("falló del chequeo, continuar??");
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  }

    Serial.println("sensores Init");

    lidarBegin();
    delay(1000);
    gyroBegin();

    if ((gyro0 == false) || (gyro1 == false)) {
      Serial.println("fallo de giroscopios, continuar??");
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again
    }
    Serial.println("READY");
  }

//================LOOP==============================================================

//==================================================================================
void loop() {

  if (millis() % 100 == 0) {  // lectura cada n milisegundos para no bloquear

    interruptors();
    servogiro(a[3]);

    for ( int i = 0; i < 7; i++) {
      Serial.print(datas[i]);
      Serial.print(",");
    }
    Serial.println();
  }

  while (Serial.available()) {
    readSerial();
    delay(2);
    stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, a[4], a[5]);
    stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, a[4], a[5]);
    stepperZ.setSpeedProfile(stepperZ.LINEAR_SPEED, a[4], a[5]);
    delay(5);
    controller.startRotate(int(a[0] * 4), int(a[1] * 3.75), int(a[2] * 3.75));
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
    Serial.println(inByte.length());
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
      if (inByte.length() >= 10) {
        a[0] = getValue(inByte, ',', 0).toInt();  // Angulos a la var global
        a[1] = getValue(inByte, ',', 1).toInt();
        a[2] = getValue(inByte, ',', 2).toInt();
        a[3] = getValue(inByte, ',', 3).toInt();
        a[4] = getValue(inByte, ',', 4).toInt();
        a[5] = getValue(inByte, ',', 5).toInt();

        Serial.print("coords");
      }
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
  sensor.startContinuous();
  Serial.println("LIDAR ON");
}
//===========lidar Read=======================================

void lidarRead() {
  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  else {
    Serial.print(sensor.readRangeContinuousMillimeters());
  }
  Serial.println();
}
//==============gyro =====================================

void gyroBegin() {
  Serial.println("Initializing I2C devices...");

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();
  //------------------------------------------
  mpuB.initialize();
  pinMode(INTERRUPT_PINB, INPUT);
  // verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050B connection successful") : F("MPU6050B connection failed"));
  devStatusB = mpuB.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51); // ACOMODAR LOS OFFSETS DE ACUERDO A UNA POSICION INICIAL
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    gyro0 = false;
  }
  //###### B -------------------------------------------------------------
  // supply your own gyro offsets here, scaled for min sensitivity
  mpuB.setXGyroOffset(51);
  mpuB.setYGyroOffset(8);
  mpuB.setZGyroOffset(21);
  mpuB.setXAccelOffset(1150);
  mpuB.setYAccelOffset(-50);
  mpuB.setZAccelOffset(1060);

  if (devStatusB == 0) {
    mpuB.CalibrateAccel(6);
    mpuB.CalibrateGyro(6);
    mpuB.PrintActiveOffsets();
    mpuB.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PINB), dmpDataReadyB, RISING);
    mpuIntStatusB = mpuB.getIntStatus();
    Serial.println(F("DMP B ready! Waiting for first interrupt..."));
    dmpReadyB = true;
    packetSizeB = mpuB.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP B Initialization failed (code "));
    Serial.print(devStatusB);
    gyro1 = false;
  }
}

void gyroRead() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    datas[0] = ypr[0] * 180 / M_PI;

  }
  if (!dmpReadyB) return;
  if (mpuB.dmpGetCurrentFIFOPacket(fifoBufferB)) {  // Get the Latest packet
    mpuB.dmpGetQuaternion(&qB, fifoBufferB);
    mpuB.dmpGetGravity(&gravityB, &qB);
    mpuB.dmpGetYawPitchRoll(yprB, &qB, &gravityB);
    datas[1] = yprB[0] * 180 / M_PI;


  }
}

///=====================interrupt pins read===============
void interruptors() {
  datas[4] = digitalRead(9);
  datas[5] = digitalRead(10);
  datas[6] = digitalRead(11);

  if ((datas[4] == 1) || (datas[5] == 1) || (datas[6] == 1)) {
    if (stop == false) {
      Serial.println("STOP");
      stop = true;
    }
  }
}

//---------------------servo-------------------------------
void servoTest() {
  for (pos = 0; pos <= 180; pos += 1) {  // goes from 0 degrees to 180 degrees
    myservo.write(pos);
    delay(5);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(5);
  }
}
//------------------------------------------------
void servogiro(int posObj) {

  if (pos > posObj) {
    pos = pos - 1;
  }

  if (pos < posObj) {
    pos = pos + 1;
  }
  myservo.write(pos);
}
