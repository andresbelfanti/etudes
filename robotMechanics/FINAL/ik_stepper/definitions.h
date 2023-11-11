
//----------------------------------steppers --------------------
//Ensable/Disable
#define SLEEP 8

// -- variables------------------------
double x = 0;
double y = 0;
double z = 0;
const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data
int coordes[3] = { 1, 1, 1 };
double angleSensor[3] = {0,0,0};
static double angulos[3] = {0,0,0};

bool motorFree = true; 
bool stopAll = false;
//==================================
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//=============================
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

// 1=full step, 2=half step etc.
#define MICROSTEPS 16

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X, SLEEP); // chequear si el motor de la base andaria mejor con otro driver
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y, SLEEP);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z, SLEEP);
SyncDriver controller(stepperX, stepperY, stepperZ);
