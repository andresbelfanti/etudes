const int en = 8;
const int stepX = 2; //X.STEP
const int dirX = 5; // X.DIR
const int stepY = 3; //Y.STEP
const int dirY = 6; // Y.DIR
const int stepZ = 4; //Z.STEP
const int dirZ = 7; // Z.DIR

int eje [] = {stepX, dirX, stepY, dirY, stepZ, dirZ};


const int stepsPerRev = 200;
int pulseWidthMicros = 100;   // microseconds
int millisBtwnSteps = 1000;

float  coordenadas[3];
float  objetivos[3];
float angles[3];
float diferencias [3];

void setup() {
  Serial.begin(9600);
  pinMode(en, OUTPUT);
  digitalWrite(en, HIGH);// ESTA BIEN????
  pinMode(stepX, OUTPUT);
  pinMode(dirX, OUTPUT);
  pinMode(stepY, OUTPUT);
  pinMode(dirY, OUTPUT);
  pinMode(stepZ, OUTPUT);
  pinMode(dirZ, OUTPUT);

  Serial.println(F("robot Initialized"));
  for (int i = 0; i <= 2; i++) {
    objetivos[i] = 100;
    angles[i] = 0;
  }
}

void loop() {
  /* una funcion corre constantemente comparando las lecturas de los giroscopios con el angulo objetivo. eso activa una función que mueve los motores
      una segunda función transforma las coordenadas recibidas desde la computadora en angulos y los pasa a los angulos objetivo
  */

  for ( int i = 0; i <= 6; i + 2) {
    diferencias[i] = objetivos[i] - angles[i];
    mover(diferencias[i], eje[i]);
  }
}




void mover(float diffs, int i) {

  if ( diffs <= 0) {
    digitalWrite(eje[i + 1], LOW);
    Serial.println("UP");
  } else {
    digitalWrite(eje[i + 1], HIGH);
    Serial.println("DOUN");
  }
  digitalWrite(eje[i], HIGH);
  delayMicroseconds(pulseWidthMicros);
  digitalWrite(eje[i], LOW);
  delayMicroseconds(millisBtwnSteps);
}
