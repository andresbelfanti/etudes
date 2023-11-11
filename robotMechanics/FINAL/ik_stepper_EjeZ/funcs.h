//=================FUNCIONES============================
void stepperInit() {
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
  Serial.println("stepper ON");
}
//=============================================================
void lidarBegin() {
  if(!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1);
   };
    
  lox.startRangeContinuous();
    Serial.println("LIDAR ON");

  }
//========================================================

void lidarRead() {

  if (lox.isRangeComplete()) {
    Serial.print("Distance in mm: ");
    Serial.println(lox.readRange());
  }
}


//=========================================================
void separador(char input[]) {  // separa el string en int's
  int index = 0;
  char separator[] = ",";
  char *token;


  if(input == "STOP"){ // CHEUQEAR SI FUNCIONA
    stopAll = true;
  }
  if(input == "REST"){ // CHEUQEAR SI FUNCIONA
   // rest();
  }

  token = strtok(input, separator);

  while (token != NULL) {
    //Serial.println(atoi(token));
    //Serial.println(index);
    coordes[index] = atoi(token);  // los pasa a coordes
    token = strtok(NULL, separator);
    index++;
  }
}

//==========================================
 bool receiver() {  // recibe serial
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  bool recibido = false;

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
      recibido = true;
    }
  }
  return recibido;
}
//=======================================================

void anglesRead() {  ///>>>>>>> lectura de giroscopios

  //  /* Get a new normalized sensor event */
  sensors_event_t gyro0;
  sensors_event_t gyro1;
  mpu_gyro0->getEvent(&gyro0);
  mpu_gyro1->getEvent(&gyro1);

  // deberia leer 2 giroscopios
  angleSensor[0] = gyro0.gyro.x * RAD_TO_DEG;  // cambiar a grados -- observar que data entrega
  angleSensor[1] = gyro1.gyro.x * RAD_TO_DEG - angleSensor[0];

  Serial.println("posición sensada: ");
  Serial.print("a1: ");  // cambiar a grados
  Serial.print(angleSensor[0]);
  Serial.print("a2: ");  // cambiar a grados
  Serial.print(angleSensor[1]);
  Serial.println();
}
//=================================================
void moveToAngle(double b, double a1, double a2) {  // mueve los steppers

 // TIENE QUE TENER ALGUN TIPO DE TOLERANCIA RESPECTO A LA DIFERENCIA

  double diffb = (b - angleSensor[0]) * 3.75;  // diferencia "ratio de la polea"
  double diffa1 = (a1 - angleSensor[1]) * 3.75;
  double diffa2 = (a2 - angleSensor[2]) * 3.75;

  controller.startRotate(diffb, diffa1, diffa2);  //rotacion de steppers;
}

//======================================================

void ikSolver(double x, double y, double z) {  //>>>>>>>>>>>> IK SOLVER


  double b = atan2(y, x) * (180 / 3.1415);  // base angle
  double l = sqrt(x * x + y * y);           // x and y extension
  double h = sqrt(l * l + z * z);
  double phi = atan(z / l) * (180 / 3.1415);
  double theta = acos((h / 2) / 75) * (180 / 3.1415);
  double a1 = phi + theta;  // angle for first part of the arm
  double a2 = phi - theta;  // angle for second part of the arm, en relación al primero, + hacia abajo
  
  angulos[0] = b;
  angulos[1] = a1;
  angulos[2] = a2;
}
//=======================================================
void rest(){ // posición de descanso automatica independiente
    moveToAngle(90,170,10); // posicion de descanso

    while(1){ // funcion para mover lentamente 
    
      unsigned wait_time_micros = controller.nextAction();  // motor control loop
      delay(1);  
    
      if (wait_time_micros <= 0) {
       Serial.println("RESTING");
        stepperX.disable();
        stepperY.disable();
        stepperZ.disable();
        break;
       }
    }
}
//===========================================================


void mpuTest() {
  Serial.println("Adafruit MPU6050 test!");

  if (!mpu0.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 0 Found!");
  mpu_temp0 = mpu0.getTemperatureSensor();
  mpu_temp0->printSensorDetails();

  mpu_accel0 = mpu0.getAccelerometerSensor();
  mpu_accel0->printSensorDetails();

  mpu_gyro0 = mpu0.getGyroSensor();
  mpu_gyro0->printSensorDetails();

  if (!mpu1.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 1 Found!");
  mpu_temp1 = mpu1.getTemperatureSensor();
  mpu_temp1->printSensorDetails();

  mpu_accel1 = mpu1.getAccelerometerSensor();
  mpu_accel1->printSensorDetails();

  mpu_gyro1 = mpu1.getGyroSensor();
  mpu_gyro1->printSensorDetails();
}