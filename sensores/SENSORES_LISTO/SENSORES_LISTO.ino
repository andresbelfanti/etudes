#include "I2Cdev.h"
#include <VL53L0X.h>
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

VL53L0X sensor;

MPU6050 mpu;
MPU6050 mpuB(0x69);  // <-- use for AD0 high

/* =========================================================================
revisar pines libres para los interrupts (dos diferentes)
   ========================================================================= */

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


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
volatile bool mpuInterruptB = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReadyB() {
  mpuInterruptB = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
//-----------------------------------------------
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.startContinuous();

  // initialize device MPUs----------------------------------
  Serial.println(F("Initializing I2C devices..."));
  //===============================================================
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  //---------------------------------------------------------------------
  mpuB.initialize();
  pinMode(INTERRUPT_PINB, INPUT);
  // verify connection
  Serial.println(F("Testing device connections B..."));
  Serial.println(mpu.testConnection() ? F("MPU6050B connection successful") : F("MPU6050B connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatusB = mpuB.dmpInitialize();

  //=========A###================================================================
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
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
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
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
    Serial.println(F("Enabling DMP B..."));
    mpuB.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt B "));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PINB), dmpDataReadyB, RISING);
    mpuIntStatusB = mpuB.getIntStatus();
    Serial.println(F("DMP B ready! Waiting for first interrupt..."));
    dmpReadyB = true;
    packetSizeB = mpuB.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP B Initialization failed (code "));
    Serial.print(devStatusB);
  }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\t");

   // Serial.println();
  }
    // if programming failed, don't try to do anything
  if (!dmpReadyB) return;
  // read a packet from FIFO
  if (mpuB.dmpGetCurrentFIFOPacket(fifoBufferB)) {  // Get the Latest packet
    mpuB.dmpGetQuaternion(&qB, fifoBufferB);
    mpuB.dmpGetGravity(&gravityB, &qB);
    mpuB.dmpGetYawPitchRoll(yprB, &qB, &gravityB);
    Serial.print("yprB\t");
    Serial.print(yprB[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(yprB[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(yprB[2] * 180 / M_PI);
    Serial.print("\t");

  }
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  else{
     Serial.print(sensor.readRangeContinuousMillimeters());
  }
    Serial.println();

  delay(5);
}
