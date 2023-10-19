#include <FABRIK2D.h>
#include <Servo.h>

// A 2DOF arm, where we have 2 links and 2+1 joints,
// where the end effector counts as one joint in this case.
int lengths[] = { 150, 80 };    // Length of shoulder and elbow in mm.
Fabrik2D fabrik2D(2, lengths);  // 3 Joints in total




// Servos should be positioned so that when all servo angles are
// equal to 90 degrees, the manipulator should point straight up.

Servo base;
Servo shoulder;
Servo elbow;


const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data
int coordes[3] = { 100, 50, 0 };


void setup() {
  base.attach(9);
  shoulder.attach(10);
  elbow.attach(11);

  // Tolerance determines how much error is allowed for solving
  // the inverse kinematics for the end effector to reach the
  // desired point.
  fabrik2D.setTolerance(0.5);

  Serial.begin(9600);
  Serial.println("<pocho is ready>");

  // base.write(0);
  // shoulder.write(0);
  // elbow.write(0);
}

void loop() {
receiver();

 
}

//==========================================
void receiver() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      separador(receivedChars);
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
    }
  }
}
//=======================================================
void separador(char input[]) {
  int index = 0;
  char separator[] = ",";
  char *token;

  token = strtok(input, separator);

  while (token != NULL) {
    //Serial.println(atoi(token));
    //Serial.println(index);
    coordes[index] = atoi(token);
    token = strtok(NULL, separator);
    index++;
  }
  mover();
}

///===================================================

void mover(){
 
  // Solve IK, move up to x=200, y=50
  fabrik2D.solve(coordes[0], coordes[1], coordes[2], lengths);
    //fabrik2D.solve2(x, 100, z, -M_PI/2.5, 0, lengths);


  // Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
  int baseAngle = fabrik2D.getBaseAngle() * RAD_TO_DEG;      // In degrees
  int shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG;  // In degrees
  int elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG;     // In degrees

  // Compute servo angles based on the output (see explanation in README.md under "Servo Orientation")
  
  int basemap = min(180, max(0, baseAngle));
  int shouldermap min(180, max(0, -shoulderAngle));
  int elbowmap = min(180, max(0, elbowAngle + 90));

  // base.write(baseAngle);
  //  shoulder.write(shoulderAngle);
  //  elbow.write(shoulderAngle);

Serial.print( coordes[0]);
Serial.print("  ");

Serial.print( coordes[1]);
Serial.print("  ");

Serial.print( coordes[2]);
Serial.print("    ##");

Serial.print("  base: " + String(basemap));
Serial.print("  ");
Serial.print("  shoulder: "+ String(shouldermap));
Serial.print("  ");
Serial.println("  elbow: "+ String(elbowAngle));

base.write(basemap);
shoulder.write(shouldermap);
elbow.write(elbowmap);


}
