#include <Servo.h>

Servo base, eje1, eje2;  // create servo object to control a servo

String inByte = "";
int pos0;
int pos1;
int pos2;


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(4, OUTPUT);
}

void loop() {

  // if (digitalRead(4) == HIGH) {
  //  Serial.println("1");
  // }


  if (Serial.available()) // if data available in serial port
  {
    String inByte = "";

    inByte = Serial.readStringUntil('\n'); // read data until newline
    if (inByte == "sleep") {
      base.detach();
      eje1.detach();
      eje2.detach();
    }
    if (inByte == "wakeup") {
      base.attach(3);
      delay(10);
      eje1.attach(5);
      delay(10);
      eje2.attach(6);
      delay(10);
    }

    if (inByte == "ledon") {
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(4, HIGH);
    }
    if (inByte == "ledoff") {
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(4, LOW);
    }

    else {
      pos0 = getValue(inByte, ',', 0).toInt();
      pos1 = getValue(inByte, ',', 1).toInt();
      pos2 = getValue(inByte, ',', 2).toInt();
    }
    
    base.write(pos0);
    
    eje1.write(pos1);

    eje2.write(pos2);


  }


}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
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
