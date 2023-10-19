const int en = 8;
const int stepX = 2; //X.STEP
const int dirX = 5; // X.DIR
const int stepY = 3; //Y.STEP
const int dirY = 6; // Y.DIR
const int stepZ = 4; //Z.STEP
const int dirZ = 7; // Z.DIR

int eje [] = {stepX, dirX, stepY, dirY, stepZ, dirZ};

float incoming [3];


const int stepsPerRev = 200;
int pulseWidthMicros = 100;   // microseconds
int millisBtwnSteps = 1000; // delay que permitiria manejar las velocidades. alguna función tipo sin

float  coordenadas[3];
float  objetivos[3];
float angles[3];
float diferencias [3];
int velocidad [3];

void setup() {
  Serial.begin(9600);
  pinMode(en, OUTPUT);
  digitalWrite(en, LOW);
  pinMode(stepX, OUTPUT);
  pinMode(dirX, OUTPUT);
  pinMode(stepY, OUTPUT);
  pinMode(dirY, OUTPUT);
  pinMode(stepZ, OUTPUT);
  pinMode(dirZ, OUTPUT);

  Serial.println(F("robot Initialized"));
}

void loop() {
  /* una funcion corre constantemente comparando las lecturas de los giroscopios con el angulo objetivo. eso activa una función que mueve los motores
      una segunda función transforma las coordenadas recibidas desde la computadora en angulos y los pasa a los angulos objetivo
  */

  for ( int i = 0; i <= 3; i++) {
    diferencias[i] = objetivos[i] - angles[i];
    velocidad[i] = float(cos((objetivos / 100) * diferencias) * 1000 )// (coseno del angulo objetivo - anngulo actual normalizado) * 1000 >> sirve para aceleración
  }
  for ( int i = 0; i <= 3; i++) {
    mover(diferencias[i], eje[i]);
  }
}




void mover(float diffs, int i) {
  if ( diffs <= 0) {
    digitalWrite(eje[i + 1], LOW);
  } else {
    digitalWrite(eje[i + 1], HIGH);

  }
  digitalWrite(eje[0], HIGH);
  delayMicroseconds(pulseWidthMicros);
  digitalWrite(eje[0], LOW);
  delayMicroseconds(millisBtwnSteps - velocidad[i]); la velocidad es un factor que se le resta al tiempo entre pasos
}


void serialEvent() {
  for (byte i = 0 ; i < 30 ; i++)
  {
    while (Serial.available () == 0)
    {}
    incoming [i] = Serial.parseFloat ();
  }
}
