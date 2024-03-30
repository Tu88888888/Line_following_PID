// For PID code
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

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
int lfSpeed = 90/2+20;
int currentSpeed = 20/2;

// float Kp = 0.08-0.08;
// float Kd = 1.5-0.25-0.75-0.25;
// float Ki = 0.0;
float Kp = 0.00;
float Kd = 0.05;
float Ki = 0.00;

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
    if(digitalRead(33)==0) {
      while(1) {
        myMotors.stop();
      }
    }
    delay(10);
  }
}

void linefollow() {
  if (numSensors == 8) {
    // error = (3 * sensorValue[0] + 2 * sensorValue[1] + sensorValue[2] - sensorValue[5] - 2 * sensorValue[6] - 3 * sensorValue[7]);
    error = ( 1.5 * sensorValue[1] + 1.0 * sensorValue[2] - 1.0 * sensorValue[5] - 1.5 * sensorValue[6]);
    error=error*3/4;
  }
  if (lineThickness > 22) {
    error = error * -1;
  }
  if (isBlackLine) {
    error = error * -1;
  }

  P = error; // it is barely an error
  I = I + error; // loi dc tich luy qua thoi gian
  D = error - previousError; //measure the changing speed of error

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// For If else normal code.

// #ifndef cbi
// #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// #endif
// #ifndef sbi
// #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// #endif

// #include <L298NX2.h>
// // #include <L298NX2.cpp>

// #define EN_A 10
// #define EN_B 11
// #define IN1_A 7
// #define IN2_A 6
// #define IN1_B 5
// #define IN2_B 4

// L298NX2 myMotors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

// /*
// P: Proportional Gain. Ex: 18, 8, 1,etc. -> Kp = 1/P //Increase Kp makes the system is more stable. Kp max = 1
// I: Integral Gain. -> Ki=1/Ti //Decrease Ti (Or increase Ki) makes "larger" integral action
// D: Derivative (seconds/minutes). Purpose: predicting change. The value of this param means how far in the future u want to predict the rate of change. 
//    This param can helps create faster response in a loop 
// */
// int globalSpeed = 200-100-15;
// int onLine = 1;
// int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];

// void setup() {
//   Serial.begin(9600);
//   pinMode(31, INPUT_PULLUP); 
//   pinMode(33, INPUT_PULLUP); 
//   pinMode(35, OUTPUT); 
// }


// void loop() {
//   while (digitalRead(31)) {}
//   delay(1000);
//   calibrate();
//   while (digitalRead(33)) {}
//   delay(1000);

//   while (1) {
//     readLine();
//     Serial.print("Sensor value: ");
//     for(int i = 7; i >= 0; i--) {
//       // Serial.print("Sensor value: ");
//       Serial.print(sensorValue[i]);
//       Serial.print("\t");
//     }
//     if (onLine == 1) {  // LINE FOLLOW
//       linefollow();
//       digitalWrite(35, HIGH);
//     } else {
//       digitalWrite(35, LOW);
      
//       turnLeft(50+20);
//     }
//    if(digitalRead(33)==0) {
//     myMotors.stop();
//     while(1) {
//       //stop
//     }
//    }
//   }
// }

// void linefollow() {
//   // if(sensorValue[0] > 700) turnRight(globalSpeed);
//   // if(sensorValue[1] > 700) turnRight(globalSpeed-20);
//   // if(sensorValue[2] > 700) turnRight(globalSpeed-50);
//   // if(sensorValue[3] > 700) moveForward(globalSpeed);
//   // if(sensorValue[4] > 700) moveForward(globalSpeed);
//   // if(sensorValue[5] > 700) turnLeft(globalSpeed-50);
//   // if(sensorValue[6] > 700) turnLeft(globalSpeed-20);
//   // if(sensorValue[7] > 700) turnLeft(globalSpeed);
//   // if(sensorValue[0] > 900 ) turnRight(globalSpeed) {
//   //   if(sensorValue[0] > sensorValue[7]) {
//   //     turnRight(globalSpeed);
//   //   }
//   // }
//   // if(sensorValue[1] > 900 ) turnRight(globalSpeed-20);
//   // if(sensorValue[2] > 900 ) turnRight(globalSpeed-50);
//   // if(sensorValue[3] > 900 ) moveForward(globalSpeed);
//   // if(sensorValue[4] > 900 ) moveForward(globalSpeed);
//   // if(sensorValue[5] > 900 ) turnLeft(globalSpeed-50);
//   // if(sensorValue[6] > 900 ) turnLeft(globalSpeed-20);
//   // if(sensorValue[7] > 900 ) turnLeft(globalSpeed);
//   // if(sensorValue[0] > 700 ) turnRight(globalSpeed);
//   // if(sensorValue[1] > 700 ) turnRight(globalSpeed-20);
//   // if(sensorValue[2] > 700 ) moveForward(globalSpeed);
//   // if(sensorValue[3] > 700 ) moveForward(globalSpeed);
//   // if(sensorValue[4] > 700 ) moveForward(globalSpeed);
//   // if(sensorValue[5] > 700 ) moveForward(globalSpeed);
//   // if(sensorValue[6] > 700 ) turnLeft(globalSpeed-20);
//   // if(sensorValue[7] == 1000 ) turnLeft(globalSpeed);

//   int maxSensorValue = 0;
//   for (int i = 2; i < 6; i++) {
//     if (sensorValue[i] > maxSensorValue) {
//       maxSensorValue = sensorValue[i];
//     }
//   }
//   for (int i = 2; i < 6; i++) {
//     if (sensorValue[i] == maxSensorValue) {
//       switch (i) {
//       case 0:
//       turnRight(globalSpeed);
//       break;
//       case 1:
//       turnRight(globalSpeed-20);
//       break;
//       case 2:
//       turnRight(globalSpeed-20);
//       break;
//       case 3:
//       moveForward(globalSpeed);
//       break;
//       case 4:
//       moveForward(globalSpeed);
//       break;
//       case 5:
//       turnLeft(globalSpeed-20);
//       break;
//       case 6:
//       turnLeft(globalSpeed);
//       break;
//       case 7:
//       // turnLeft(globalSpeed);
//       break;
//       default:
//       break;
//       }
//     }
//   }

// }



// void calibrate() {
//   for (int i = 0; i < 8; i++) {
//     minValues[i] = analogRead(i);
//     maxValues[i] = analogRead(i);
//   }

//   for (int i = 0; i < 10000/2; i++) {
//     myMotors.setSpeed(globalSpeed);
//     myMotors.forwardA();
//     myMotors.backwardB();
//     for (int i = 0; i < 8; i++) {
//       if (analogRead(i) < minValues[i]) {
//         minValues[i] = analogRead(i);
//       }
//       if (analogRead(i) > maxValues[i]) {
//         maxValues[i] = analogRead(i);
//       }
//     }
//   }

//   for (int i = 0; i < 8; i++) {
//     threshold[i] = (minValues[i] + maxValues[i]) / 2;
//     Serial.print(threshold[i]);
//     Serial.print(" ");
//   }
//   Serial.println();
//   myMotors.stop();
// }

// void readLine() {
//   onLine = 0;
//   for (int i = 2; i < 6; i++) {
//     sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
//     sensorValue[i] = constrain(sensorValue[i], 0, 1000);
//     if (sensorValue[i] > 700 ) {
//       onLine = 1;
//     };
//   }
// }

// void turnLeft(int speed) {
//   myMotors.setSpeed(speed);
//   myMotors.forwardA();
//   myMotors.backwardB();
//   Serial.println("Turning left");
// }

// void turnRight(int speed) {
//   myMotors.setSpeed(speed);
//   myMotors.backwardA();
//   myMotors.forwardB();
//   Serial.println("Turning right");
// }

// void moveForward(int speed) {
//   myMotors.setSpeed(speed);
//   myMotors.forward();
//   Serial.println("Moving forward");
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// //updated 29 June 2023

// #ifndef cbi
// #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
// #endif
// #ifndef sbi
// #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// #endif

// #include <L298NX2.h>
// // #include <L298NX2.cpp>

// //Enter Line Details
// bool isBlackLine = 1;          //keep 1 in case of black line. In case of white line change this to 0
// unsigned int lineThickness = 25;  //Enter line thickness in mm. Works best for thickness between 10 & 35
// unsigned int numSensors = 8;      // Enter number of sensors as 5 or 7
// bool brakeEnabled = 0;

// #define EN_A 10
// #define EN_B 11
// #define IN1_A 7
// #define IN2_A 6
// #define IN1_B 5
// #define IN2_B 4

// L298NX2 myMotors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

// /*
// P: Proportional Gain. Ex: 18, 8, 1,etc. -> Kp = 1/P //Increase Kp makes the system is more stable. Kp max = 1
// I: Integral Gain. -> Ki=1/Ti //Decrease Ti (Or increase Ki) makes "larger" integral action
// D: Derivative (seconds/minutes). Purpose: predicting change. The value of this param means how far in the future u want to predict the rate of change. 
//    This param can helps create faster response in a loop 
// */
// int P, D, I, previousError, PIDvalue, error;
// int speedB, speedA;
// int lfSpeed = 120;
// int currentSpeed = 30;

// float Kp = 0.06;
// float Kd = 1.5;
// float Ki = 0.0;
// // int currentSpeed = 30;
// // float Kp = 0.018;
// // float Kd = 0.0;
// // float Ki = 0.0;

// int onLine = 1;
// int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];
// bool brakeFlag = 0;

// void setup() {
//   Serial.begin(9600);
//   pinMode(31, INPUT_PULLUP); //11
//   pinMode(33, INPUT_PULLUP); //12
//   pinMode(35, OUTPUT); //13
//   lineThickness = constrain(lineThickness, 10, 35);
// }


// void loop() {
//   while (digitalRead(31)) {}
//   delay(1000);
//   calibrate();
//   while (digitalRead(33)) {}
//   delay(1000);

//   while (1) {
//     readLine();
//     if (currentSpeed < lfSpeed) currentSpeed++;
//     if (onLine == 1) {  //PID LINE FOLLOW
//       linefollow();
//       digitalWrite(35, HIGH);
//       brakeFlag = 0;
//     } else {
//       digitalWrite(35, LOW);
//       if (error > 0) {
//         if (brakeEnabled == 1 && brakeFlag == 0) {
//           myMotors.stop();
//           delay(30);
//         }
//         myMotors.setSpeedA(100);
//         myMotors.setSpeedB(150);
//         myMotors.backwardA();
//         myMotors.forwardB();
//         brakeFlag = 1;
//       } else {
//         if (brakeEnabled == 1 && brakeFlag == 0) {
//           myMotors.stop();
//           delay(30);
//         }
//         myMotors.setSpeedA(150);
//         myMotors.setSpeedB(100);
//         myMotors.forwardA();
//         myMotors.backwardB();
//         brakeFlag = 1;
//       }
//     }

//    if(digitalRead(33)==0) {
//     myMotors.stop();
//     while(1) {
//       //stop
//     }
//    }
//   }
// }

// void linefollow() {
//   if (numSensors == 8) {
//     error = (3 * sensorValue[0] + 2 * sensorValue[1] + sensorValue[2] - sensorValue[5] - 2 * sensorValue[6] - 3 * sensorValue[7]);
//     // error = (3 * sensorValue[0] + 2 * sensorValue[1] + 1 * sensorValue[2] + 0.5 * sensorValue[3] - 0.5 * sensorValue[4] - 1 * sensorValue[5] - 2 * sensorValue[6] - 3 * sensorValue[7]);
//   }
//   if (lineThickness > 22) {
//     error = error * -1;
//   }
//   if (isBlackLine) {
//     error = error * -1;
//   }

//   P = error;
//   I = I + error;
//   D = error - previousError;

//   PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
//   previousError = error;

//   speedB = currentSpeed - PIDvalue;
//   speedA = currentSpeed + PIDvalue;
//   speedA /= 4;
//   speedB /= 4;

//   if (speedB > 255) {
//     speedB = 255;
//   }
//   if (speedB < 0) {
//     speedB = 0;
//   }
//   if (speedA > 255) {
//     speedA = 255;
//   }
//   if (speedA < 0) {
//     speedA = 0;
//   }
//   myMotors.setSpeedA(speedA);
//   myMotors.setSpeedB(speedB);
//   myMotors.forward();
// }



// void calibrate() {
//   for (int i = 0; i < 8; i++) {
//     minValues[i] = analogRead(i);
//     maxValues[i] = analogRead(i);
//   }

//   for (int i = 0; i < 10000/2; i++) {
//     myMotors.setSpeed(255);
//     myMotors.forwardA();
//     myMotors.backwardB();
//     for (int i = 0; i < 8; i++) {
//       if (analogRead(i) < minValues[i]) {
//         minValues[i] = analogRead(i);
//       }
//       if (analogRead(i) > maxValues[i]) {
//         maxValues[i] = analogRead(i);
//       }
//     }
//   }

//   for (int i = 0; i < 8; i++) {
//     threshold[i] = (minValues[i] + maxValues[i]) / 2;
//     Serial.print(threshold[i]);
//     Serial.print(" ");
//   }
//   Serial.println();
//   myMotors.stop();
// }

// void readLine() {
//   onLine = 0;
//   for (int i = 0; i < 8; i++) {
//     sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
//     sensorValue[i] = constrain(sensorValue[i], 0, 1000);
//     if (isBlackLine==1 && sensorValue[i] > 700) onLine = 1;
//     if (isBlackLine==0 && sensorValue[i] < 700) onLine = 1;
//   }
// }
