// Arduino/Teensy example for Arduino Braccio
#include <Servo.h>

Servo myservoB;  // create servo object to control a servo
Servo myservoU;  // create servo object to control a servo
Servo myservoF;  // create servo object to control a servo

float a0, a1, a2, a3;

int x = 51; // posicion inicial
int y= 0;
int z = 50;

// Include the library InverseK.h
#include <InverseK.h>

void setup() {

  Serial.begin(9600);

    myservoF.attach(9);  // attaches the servo on pin 9 to the servo object
    myservoU.attach(10);  // attaches the servo on pin 9 to the servo object
    myservoB.attach(11);  // attaches the servo on pin 9 to the servo object

delay(5000);

    

  // Setup the lengths and rotation limits for each link
  Link base, upperarm, forearm, hand;

  base.init(0, b2a(0.0), b2a(180.0));
  upperarm.init(100, b2a(0), b2a(180.0));
  forearm.init(100, b2a(0.0), b2a(180.0));
  hand.init(100, b2a(0.0), b2a(180.0));



  // Attach the links to the inverse kinematic model
  InverseK.attach(base, upperarm, forearm, hand);
  
  
  
  delay(1000);

  Serial.print('Todo va bien');

  // InverseK.solve() return true if it could find a solution and false if not.

  // Calculates the angles without considering a specific approach angle
  //InverseK.solve(x, y, z, a0, a1, a2, a3)
  if(InverseK.solve(x, y, z, a0, a1, a2, a3)) {
    Serial.print(a2b(a0)); Serial.print(',');
    Serial.print(a2b(a1)); Serial.print(',');
    Serial.print(a2b(a2)); Serial.print(',');
    Serial.println(a2b(a3));
    
    Serial.print("en un segundo me muevo");
    delay(2000);
    myservoF.write(a2b(a3));
    myservoU.write(a2b(a2));
    myservoB.write(a2b(a0));
  } else {
    Serial.println("No solution found!");
  }






}

void loop() {

}

// Quick conversion from the Braccio angle system to radians
float b2a(float b){
  return b / 180.0 * PI - HALF_PI;
}

// Quick conversion from radians to the Braccio angle system
float a2b(float a) {
  return (a + HALF_PI) * 180 / PI;
}
