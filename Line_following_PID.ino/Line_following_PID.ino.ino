
#include <L298NX2.h>
// #include <L298NX2.cpp>

//Enter Line Details
bool isBlackLine = 1;          //keep 1 in case of black line. In case of white line change this to 0
unsigned int lineThickness = 25;  //Enter line thickness in mm. Works best for thickness between 10 & 35
unsigned int numSensors = 8;      // Enter number of sensors as 5 or 7
bool brakeEnabled = 0;

#define EN_A 10
#define EN_B 11
#define IN1_A 7
#define IN2_A 6
#define IN1_B 5
#define IN2_B 4

L298NX2 myMotors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
int P, D, I, previousError, PIDvalue, error;
int speedB, speedA;
int lfSpeed = 90/2;
int currentSpeed = 30;

float Kp = 0.0;
float Kd = 0.3;
float Ki = 0.0;

int onLine = 1;
int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];
bool brakeFlag = 0;

void setup() {
  Serial.begin(9600);
  pinMode(31, INPUT_PULLUP); //11
  pinMode(33, INPUT_PULLUP); //12
  pinMode(35, OUTPUT); //13
  lineThickness = constrain(lineThickness, 10, 35);
}

void loop() {
  while (digitalRead(31)) {}
  delay(1000);
  calibrate();
  while (digitalRead(33)) {}
  delay(1000);

  while (1) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  //PID LINE FOLLOW
      linefollow();
      digitalWrite(35, HIGH);
      brakeFlag = 0;
    } else {
      digitalWrite(35, LOW);
      if (error > 0) {
        if (brakeEnabled == 1 && brakeFlag == 0) {
          myMotors.stop();
          delay(30);
        }
        myMotors.setSpeedA(80);
        myMotors.setSpeedB(130);
        myMotors.backwardA();
        myMotors.forwardB();
        brakeFlag = 1;
      } else {
        if (brakeEnabled == 1 && brakeFlag == 0) {
          myMotors.stop();
          delay(30);
        }
        myMotors.setSpeedA(130);
        myMotors.setSpeedB(80);
        myMotors.forwardA();
        myMotors.backwardB();
        brakeFlag = 1;
      }
    }
  }
}

void linefollow() {
  if (numSensors == 8) {
    error = (3 * sensorValue[0] + 2 * sensorValue[1] + sensorValue[2] - sensorValue[5] - 2 * sensorValue[6] - 3 * sensorValue[7]);
  }
  if (lineThickness > 22) {
    error = error * -1;
  }
  if (isBlackLine) {
    error = error * -1;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  speedB = currentSpeed - PIDvalue;
  speedA = currentSpeed + PIDvalue;

  if (speedB > 255) {
    speedB = 255;
  }
  if (speedB < 0) {
    speedB = 0;
  }
  if (speedA > 255) {
    speedA = 255;
  }
  if (speedA < 0) {
    speedA = 0;
  }
  myMotors.setSpeedA(speedA);
  myMotors.setSpeedB(speedB);
  myMotors.forward();
}

void calibrate() {
  for (int i = 0; i < 8; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 10000/4; i++) {
    myMotors.setSpeed(255);
    myMotors.forwardA();
    myMotors.backwardB();
    for (int i = 0; i < 8; i++) {
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for (int i = 0; i < 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();
  myMotors.stop();
}

void readLine() {
  onLine = 0;
  for (int i = 0; i < 8; i++) {
    sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    if (isBlackLine==1 && sensorValue[i] > 700) onLine = 1;
    if (isBlackLine==0 && sensorValue[i] < 700) onLine = 1;
  }
}