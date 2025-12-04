// ------------------------------------------------- FOR 7 SENSORS -------------------------------------------- //


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define AIN1 2
#define AIN2 4
#define PWMA 5
#define PWMB 6
#define BIN1 8
#define BIN2 7
#define STBY 9

unsigned int lineThickness = 15;
const int numSensors = 7;
int sensorWeight[numSensors] = {5, 2, 1, 0, -1, -2, -5};
int minValues[numSensors], maxValues[numSensors], threshold[numSensors];
int sensorValue[numSensors], sensorArray[numSensors];
int dynamicThreshold[numSensors];
int lineMode = 0; // 0 = Black line on White, 1 = White line on Black
int onLine = 1;

// ------ PID VARIABLES ------
float Kp = 0.0311; //0.49 //0.04
float Ki = 0;
float Kd = 0.001; //0.059 //3.5
int P, I, D, previousError, PIDvalue;
double error;

int lfSpeed = 80;
int currentSpeed = 30;
int lsp, rsp;
int activeSensors;

void setup() {

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);


  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(9, OUTPUT);        //LED
  digitalWrite(9, HIGH);

  //pinMode(11, INPUT_PULLUP); // button 1 calibrate
  //pinMode(12, INPUT_PULLUP); // button 2 actual run

  lineThickness = constrain(lineThickness, 10, 35);

  calibrate();
  delay(3000);
}

void loop() {
  int avgSensor = 0, avgThreshold = 0;
  for (int i = 0; i < numSensors; i++) {
    avgSensor += analogRead(i);
    avgThreshold += threshold[i];
  }
  avgSensor /= numSensors;
  avgThreshold /= numSensors;



  if (avgSensor < avgThreshold) {
    lineMode = 0;
  }

  else {
    lineMode = 1;
  }

  readLine();

  if (currentSpeed < lfSpeed) currentSpeed++;

  if (onLine == 1) {
    linefollow();
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
    if (error > 0) {
      motor1run(-100);
      motor2run(100);
    } else {
      motor1run(100);
      motor2run(-100);
    }
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < numSensors; i++) {
    error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }

  if (activeSensors > 0) {
    error = error / activeSensors;
  }

  P = error;
  I += error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);

  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  for (int i = 0; i < numSensors; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 15000; i++) {
    motor1run(80);
    motor2run(-80);

    for (int j = 0; j < numSensors; j++) {
      int value = analogRead(j);
      if (value < minValues[j]) minValues[j] = value;
      if (value > maxValues[j]) maxValues[j] = value;
    }
  }

  for (int i = 0; i < numSensors; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;

  for (int i = 0; i < numSensors; i++) {
    int raw = analogRead(i);
    if (lineMode == 0) {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 1000, 0);
    }

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    dynamicThreshold[i] = (minValues[i] + maxValues[i]) / 2;
    sensorArray[i] = sensorValue[i] > dynamicThreshold[i];

    if (sensorArray[i]) onLine = 1;

    Serial.print(sensorArray[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// ------ MOTOR CONTROL ------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}