#include "Adafruit_VL53L0X.h"
#include "I2Cdev.h"
#include "MPU6050.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyroA;
MPU6050 accelgyroB(0x69); // <-- use for AD0 high

int16_t axA, ayA, azA;
int16_t gxA, gyA, gzA;

int16_t axB, ayB, azB;
int16_t gxB, gyB, gzB;


// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

void setup() {
      Serial.begin(115200);

     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

 // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(10);
  }
if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
    Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));
    lox.startRangeContinuous();


    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyroA.initialize();
    accelgyroB.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyroA.testConnection() ? "MPU6050A connection successful" : "MPU6050A connection failed");
    Serial.println(accelgyroB.testConnection() ? "MPU6050B connection successful" : "MPU6050B connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyroA.getMotion6(&axA, &ayA, &azA, &gxA, &gyA, &gzA);
    accelgyroB.getMotion6(&axB, &ayB, &azB, &gxB, &gyB, &gzB);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("gyroA:");
        Serial.print(axA); Serial.print("\t");
        Serial.print(ayA); Serial.print("\t");
        Serial.print(azA); Serial.print("\t");
        Serial.print(gxA); Serial.print("\t");
        Serial.print(gyA); Serial.print("\t");
        Serial.println(gzA);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(axA >> 8)); Serial.write((uint8_t)(axA & 0xFF));
        Serial.write((uint8_t)(ayA >> 8)); Serial.write((uint8_t)(ayA & 0xFF));
        Serial.write((uint8_t)(azA >> 8)); Serial.write((uint8_t)(azA & 0xFF));
        Serial.write((uint8_t)(gxA >> 8)); Serial.write((uint8_t)(gxA & 0xFF));
        Serial.write((uint8_t)(gyA >> 8)); Serial.write((uint8_t)(gyA & 0xFF));
        Serial.write((uint8_t)(gzA >> 8)); Serial.write((uint8_t)(gzA & 0xFF));
    #endif
     #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("gyroB:");
        Serial.print(axB); Serial.print("\t");
        Serial.print(ayB); Serial.print("\t");
        Serial.print(azB); Serial.print("\t");
        Serial.print(gxB); Serial.print("\t");
        Serial.print(gyB); Serial.print("\t");
        Serial.println(gzB);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(axB >> 8)); Serial.write((uint8_t)(axB & 0xFF));
        Serial.write((uint8_t)(ayB >> 8)); Serial.write((uint8_t)(ayB & 0xFF));
        Serial.write((uint8_t)(azB >> 8)); Serial.write((uint8_t)(azB & 0xFF));
        Serial.write((uint8_t)(gxB >> 8)); Serial.write((uint8_t)(gxB & 0xFF));
        Serial.write((uint8_t)(gyB >> 8)); Serial.write((uint8_t)(gyB & 0xFF));
        Serial.write((uint8_t)(gzB >> 8)); Serial.write((uint8_t)(gzB & 0xFF));
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
