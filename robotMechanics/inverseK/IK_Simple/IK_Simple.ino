
#include <Servo.h>

#define BASESERVOPIN 9
#define ARM1SERVOPIN 10
#define ARM2SERVOPIN 11


Servo arm1servo;
Servo arm2servo;
Servo baseservo;

double x = 0;
double y = 0;
double z = 0;

int gripAngle = 67;  // ESSSTO????

const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data
int coordes[3] = { 100, 50, 0 };




long oldPosition = 0;
//====================
int angleToMicroseconds(double angle) {
  double val = 460.0 + (((2400.0 - 460.0) / 180.0) * angle);
  return (int)val;
}
//=================================================
void moveToAngle(double b, double a1, double a2) {
  arm1servo.writeMicroseconds(angleToMicroseconds(a1));
  arm2servo.writeMicroseconds(angleToMicroseconds(a2 ));
  baseservo.writeMicroseconds(angleToMicroseconds(b));
}
//======================================================

void moveToPos(double x, double y, double z) {
  double b = atan2(y, x) * (180 / 3.1415);  // base angle

  double l = sqrt(x * x + y * y);  // x and y extension

  double h = sqrt(l * l + z * z);

  double phi = atan(z / l) * (180 / 3.1415);

  double theta = acos((h / 2) / 75) * (180 / 3.1415);

  double a1 = phi + theta;  // angle for first part of the arm
  double a2 = phi - theta;  // angle for second part of the arm

  moveToAngle(b, a1, a2);
}
//===========================================================
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("<pocho is ready>");

  baseservo.attach(BASESERVOPIN, 460, 2400);
  arm1servo.attach(ARM1SERVOPIN, 460, 2400);
  arm2servo.attach(ARM2SERVOPIN, 460, 2400);

  moveToAngle(x, y, z);
  delay(2000);
}

void loop() {
  receiver();

  moveToPos(x, y, z);
  //delay(2000);
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
    Serial.println(index);
    coordes[index] = atoi(token);
    token = strtok(NULL, separator);
    index++;
  }
}
