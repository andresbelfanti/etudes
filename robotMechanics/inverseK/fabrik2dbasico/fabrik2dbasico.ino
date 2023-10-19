#include <FABRIK2D.h>
#include <Servo.h>

// A 2DOF arm, where we have 2 links and 2+1 joints, 
// where the end effector counts as one joint in this case.
int lengths[] = {225, 150}; // Length of shoulder and elbow in mm.
Fabrik2D fabrik2D(3, lengths); // 3 Joints in total

// Servos should be positioned so that when all servo angles are
// equal to 90 degrees, the manipulator should point straight up.
Servo shoulder;
Servo elbow;

void setup() {
  shoulder.attach(10);
  elbow.attach(11);
  
  // Tolerance determines how much error is allowed for solving
  // the inverse kinematics for the end effector to reach the
  // desired point.
  fabrik2D.setTolerance(0.1);
}

void loop() {

    if (y < 10) {
    toggle_y = 0;
    y = 10;
  } else if (y > 100) {
    toggle_y = 1;
    y = 100;
  }

  // Solve IK, move up to x=200, y=50
  fabrik2D.solve(200,50,lengths);
  
  // Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
  int shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG; // In degrees
  int elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG; // In degrees
  
  // Compute servo angles based on the output (see explanation in README.md under "Servo Orientation")
  shoulder.write(min(180, max(0, -shoulderAngle)));
  elbow.write(min(180, max(0, elbowAngle + 90)));
  
  // The following delay is just a part of this example, remove it
  delay(1000);
  
  // Solve IK, move down to x=150, y=10
  fabrik2D.solve(150,50,lengths);
  
  // Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
  shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG; // In degrees
  elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG; // In degrees
  
  // Compute servo angles based on the output (see explanation in README.md under "Servo Orientation")
  shoulder.write(min(180, max(0, shoulderAngle)));
  elbow.write(min(180, max(0, elbowAngle + 90)));

  Serial.println()
  
  // The following delay is just a part of this example, remove it
  delay(1000);

  if (toggle_y == 0) {
    y++;
  } else {
    y--;
  }

}