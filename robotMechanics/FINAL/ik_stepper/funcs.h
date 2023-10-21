//=================FUNCIONES============================
void stepperInit(){
  stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
  stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
  stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);
  // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
  stepperX.setEnableActiveState(LOW);
  stepperY.setEnableActiveState(LOW);
  stepperZ.setEnableActiveState(LOW);

  stepperX.enable();
  stepperY.enable();
  stepperZ.enable();

  stepperX.setSpeedProfile(stepperX.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperY.setSpeedProfile(stepperY.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
  stepperZ.setSpeedProfile(stepperY.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
}

//==========================================
void receiver() { // recibe serial
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
      motorFree = false;
    }
  }
}
//=======================================================
void separador(char input[]) { // separa el string en int's 
  int index = 0;
  char separator[] = ",";
  char *token;

  token = strtok(input, separator);

  while (token != NULL) {
    //Serial.println(atoi(token));
    //Serial.println(index);
    coordes[index] = atoi(token); // los pasa a coordes
    token = strtok(NULL, separator);
    index++;
  }
}
//=================================================
void moveToAngle(double b, double a1, double a2) {  // mueve los steppers


   double girosA[3] = { 0, 0, 0 }; // almacena a0, a1, a2 
   girosA = anglesRead(); 

    double diffb =  (b - girosA[0]) * 3.75; // diferencia "ratio de la polea"
    double diffa1 = (a1 - girosA[1])*3.75;
    double diffa2 = (a2 - girosA[2] )*3.75 ;

     controller.startRotate(compare(diffb,diffa1,diffa2) ); //rotacion de steppers;
}

//======================================================

double *moveToPos(double x, double y, double z) { //>>>>>>>>>>>> IK SOLVER 

static double posiciones[3];

  double b = atan2(y, x) * (180 / 3.1415);  // base angle
  double l = sqrt(x * x + y * y);  // x and y extension
  double h = sqrt(l * l + z * z);
  double phi = atan(z / l) * (180 / 3.1415);
  double theta = acos((h / 2) / 75) * (180 / 3.1415);
  double a1 = phi + theta;  // angle for first part of the arm
  double a2 = phi - theta;  // angle for second part of the arm, en relación al primero, + hacia abajo

Serial.println("Angulos:")
  Serial.print(b);
  Serial.print(" -");
  Serial.print(a1);
  Serial.print(" -");
  Serial.print(a2);
  Serial.println(" -");
  
  return posiciones; // chequear esto
}
//===========================================================

//==============================================
double *anglesRead(){ ///>>>>>>> lectura de giroscopios

static double angles[3];

 //  /* Get a new normalized sensor event */
  sensors_event_t gyro0;
sensors_event_t gyro1;

  mpu_gyro0->getEvent(&gyro0);
  mpu_gyro1->getEvent(&gyro1);

  // deberia leer 2 giroscopios
angles[0] = gyro0.gyro.x* RAD_TO_DEG; // cambiar a grados -- observar que data entrega
angles[1] = gyro1.gyro.x* RAD_TO_DEG;


 
 Serial.println("posición sensada: ")
  Serial.print("a1: "); // cambiar a grados
  Serial.print(angles[0]);
  Serial.print("a2: "); // cambiar a grados
  Serial.print(angles[1].x);
  Serial.println();

  // ademas lectura de base - Encoder Optico?? // angles[2]

  return angles;

}

void mpuTest(){ Serial.println("Adafruit MPU6050 test!");

  if (!mpu0.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 0 Found!");
  mpu_temp = mpu0.getTemperatureSensor();
  mpu_temp0->printSensorDetails();

  mpu_accel = mpu0.getAccelerometerSensor();
  mpu_accel0->printSensorDetails();

  mpu_gyro = mpu0.getGyroSensor();
  mpu_gyro0->printSensorDetails();


  if (!mpu1.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

    Serial.println("MPU6050 1 Found!");
  mpu_temp = mpu1.getTemperatureSensor();
  mpu_temp1->printSensorDetails();

  mpu_accel = mpu1.getAccelerometerSensor();
  mpu_accel1->printSensorDetails();

  mpu_gyro = mpu1.getGyroSensor();
  mpu_gyro1->printSensorDetails();

  }