#include <Servo.h>
#include <ezButton.h>
#include <stdlib.h>
#define LEFT 0
#define RIGHT 1
#define PI 3.14159
#define WHEEL_ENCODER_COUNT 70
#define WHEEL_DIAMETER 56
#define WHEEL_WIDTH 150
#define FORWARD_COEFF_LEFT 1.2
#define BACK_COEFF_LEFT 1.0
#define FORWARD_ADJUSTED_R 100/FORWARD_COEFF_LEFT
#define BACK_ADJUSTED_R 100/BACK_COEFF_LEFT
#define LEFTENC 0
#define RIGHTENC 1
#define STRAIGHTKP 1.5



int switchInput = 13;

int leftForward = 9;  // make sure this is IN1 or IN3
int leftBack = 10;
int rightForward = 3;  // make sure this is IN1 or IN3
int rightBack = 11;

int lowerArmInput = 6;
int upperArmInput = 5;
int leftIR = 4;
int rightIR = 2;

int leftEncoderCount = 0;
int rightEncoderCount = 0;


int leftPastPinValue, rightPastPinValue;
int leftPinValue, rightPinValue;
double leftSpeed, rightSpeed;
int distanceToDrive;
int state = HIGH;
bool switchPressed = false;


int convertPower(double percentPower);

class dcMotor {
public:
  int forwardPin;
  int backPin;
  void setPins(int f, int b) {
    forwardPin = f;
    backPin = b;
    pinMode(f, OUTPUT);
    pinMode(b, OUTPUT);
  }
  void drive(double power) {
    int convertedPower = convertPower(power);
    if (power > 0) {
      digitalWrite(backPin, LOW);
      analogWrite(forwardPin, convertedPower);
    } else if (power < 0) {
      digitalWrite(forwardPin, LOW);
      analogWrite(backPin, convertedPower);
    } else {
      digitalWrite(forwardPin, LOW);
      digitalWrite(backPin, LOW);
    }
  }
};

dcMotor leftMotor;
dcMotor rightMotor;
Servo lowerArm;
Servo upperArm;

// For the switch: have GND connected to C, have input connected to NO
ezButton limitSwitch(switchInput);  // create ezButton object that attach to pin 4;


void setup() {
  limitSwitch.setDebounceTime(50);  // set debounce time to 50 milliseconds

  lowerArm.attach(lowerArmInput);
  upperArm.attach(upperArmInput);
  leftMotor.setPins(leftForward, leftBack);
  rightMotor.setPins(rightForward, rightBack);

  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  leftPastPinValue = digitalRead(leftIR);
  rightPastPinValue = digitalRead(rightIR);
  Serial.begin(9600);
}


void loop() {



  // /*
  wait(); // waits for bump switch press

  delay(1000);

  driveStraightSwitch();
  stopDrive();
  delay(1000);


  // backRightWheel(100, 5);
  // delay(1000);

  offsetBackward(-120);

  delay(1000);

  turnLeft(100, 85);
  delay(1000);

  driveStraightSwitch();
  delay(1000);


  // backRightWheel(100, 5);
  // delay(1000);

  offsetBackward(-260);
  delay(1000);
  
  turnLeft(100, 111);
  delay(1000);

  driveDistance(FORWARD_ADJUSTED_R, 180);
  delay(1000);

  lowerArm.write(0);
  delay(300);
  lowerArm.write(90);
  delay(1000);

  driveDistance(FORWARD_ADJUSTED_R, 140);
  delay(1000);

  for(int i = 90; i <= 180; i+=10) {
    upperArm.write(i);
    delay(100);
  }
  delay(2000);

  // */
  
  
  // //-------------- Drive forward until bump switch ---------------------------
  // while (limitSwitch.getState() == LOW) limitSwitch.loop();
  // while (limitSwitch.getState() != LOW) {
  //   driveStraight(30);
  //   limitSwitch.loop();
  // }


}

// functions ----------------------------------------



void wait(void) {
  while (1) {
    while (limitSwitch.getState() == LOW) limitSwitch.loop();
    limitSwitch.loop();
    int switchState = limitSwitch.getState();
    if (switchState == LOW) return;
  }
}

void motorPurge(void) {
  digitalWrite(leftBack, HIGH);
  digitalWrite(rightBack, HIGH);
  delay(50);
  digitalWrite(leftBack, LOW);
  digitalWrite(rightBack, LOW);
  delay(25);
  digitalWrite(leftForward, HIGH);
  digitalWrite(rightForward, HIGH);
  delay(50);
  digitalWrite(leftForward, LOW);
  digitalWrite(rightForward, LOW);
  delay(25);
  digitalWrite(leftBack, HIGH);
  digitalWrite(rightBack, HIGH);
  delay(100);
  digitalWrite(leftBack, LOW);
  digitalWrite(rightBack, LOW);
  delay(200);
}

void driveStraightSwitch(void) {
  resetEncoders();
  while (limitSwitch.getState() == LOW) limitSwitch.loop();
  digitalWrite(leftForward, HIGH);
  while (limitSwitch.getState() != LOW) {
    driveStraight(FORWARD_ADJUSTED_R);
    limitSwitch.loop();
  }
  stopDrive();
}

void turnLeft(double power, double angle) {
  resetEncoders();
  int convertedPower = convertPower(power);
  double turnDistanceWheel = 2*PI * (angle/360.0) * WHEEL_WIDTH ;
  int encTurn = distanceToEncoder(turnDistanceWheel);
  analogWrite(rightForward, convertedPower);
  while (rightEncoderCount < encTurn) {
    if (readEncoder(RIGHTENC)) rightEncoderCount += 1;
  }
  analogWrite(rightForward, 0);
}

void backRightWheel(double power, double angle) {
  resetEncoders();
  int convertedPower = convertPower(power);
  double turnDistanceWheel = 2*PI * (angle/360.0) * WHEEL_WIDTH ;
  int encTurn = distanceToEncoder(turnDistanceWheel);
  analogWrite(rightBack, convertedPower);
  while (rightEncoderCount < encTurn) {
    if (readEncoder(RIGHTENC)) rightEncoderCount += 1;
  }
  analogWrite(rightBack, 0);

}


void driveDistance(double power, double distance) {
  resetEncoders();
  int startTime;
  int factor;
  int leftPin;
  double leftPower, rightPower;
  double straightError;

  int encoderAvg, encDist;
  if (distance > 0) { 
    factor = 1; 
    leftPin = leftForward;
  } else {
    factor = -1; 
    leftPin = leftBack;
  }

  for (int i = 0; abs(i) < power; i+=(factor*10)) {
    driveStraight(i);
    if (abs(encoderAverage()) > abs(distanceToEncoder(distance)) ) break; 
  }


  digitalWrite(leftPin, HIGH);
  while (1) {
    driveStraight(power*factor);
    if (abs(encoderAverage()) > abs(distanceToEncoder(distance)) ) break; 
  }
  stopDrive();
}

void driveStraight(double power) {
  int startTime = millis();
  int factor;
  int delay = 30;
  if (power > 0) factor = 1;
  else factor = -1;

  while (millisFromOffset(startTime) < delay) {
    if (readEncoder(LEFTENC)) leftEncoderCount += factor;
    if (readEncoder(RIGHTENC)) rightEncoderCount += factor;
  }

  double leftPower, rightPower;
  double straightError;

  straightError = leftEncoderCount - rightEncoderCount;

  // leftPower = power - straightError * STRAIGHTKP;
  rightPower = power + straightError * STRAIGHTKP;
  // leftMotor.drive(leftPower);
  rightMotor.drive(rightPower);
}

void resetEncoders(void) {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}

double encoderAverage(void) {
  return (leftEncoderCount + rightEncoderCount) / 2.0;
}

int millisFromOffset(int offset) {
  return (millis() - offset);
}

void stopDrive(void) {
  rightMotor.drive(0);
  leftMotor.drive(0);
}


bool readEncoder(int type) {
  bool returnvar;
  if (type == LEFTENC) {
    leftPinValue = digitalRead(leftIR);
    if (leftPinValue != leftPastPinValue)
      returnvar = true;
    else
      returnvar = false;
    leftPastPinValue = leftPinValue;
  }
  if (type == RIGHTENC) {
    rightPinValue = digitalRead(rightIR);
    if (rightPinValue != rightPastPinValue)
      returnvar = true;
    else
      returnvar = false;
    rightPastPinValue = rightPinValue;
  }

  return returnvar;
}

int distanceToEncoder(double distance) {
  double wheel_circ = WHEEL_DIAMETER * PI;
  int encoder_value = customRound(distance / wheel_circ * WHEEL_ENCODER_COUNT);
  return encoder_value;
}

double encoderToDistance(int encoder) {
  double wheel_circ = WHEEL_DIAMETER * PI;
  double dist = encoder * ((double)wheel_circ / WHEEL_ENCODER_COUNT);
  return dist;
}

int convertPower(double percentPower) {
  percentPower = abs(percentPower);
  int num = customRound(percentPower * (255 / 100.0));
  if (num > 255) num = 255;
  if (num < 0) num = 0;
  return num;
}

int customRound(double num) {
  int rounded;
  double rem = num - floor(num);
  if (rem >= 0.5) rounded = floor(num) + 1;
  else rounded = floor(num);
  return rounded;
}

double saturate(double lower, double upper, double input) {
  if (input < lower) return lower;
  if (input > upper) return upper;
  return input;
}


void offsetForward(double distance) {
  resetEncoders();
  digitalWrite(leftForward, HIGH);
  rightMotor.drive(FORWARD_ADJUSTED_R);

  while (abs(encoderAverage()) < abs(distanceToEncoder(distance))) {
    if (readEncoder(LEFTENC)) leftEncoderCount += 1;
    if (readEncoder(RIGHTENC)) rightEncoderCount += 1;
  }
  analogWrite(rightForward, 0);
  delay(50);
  digitalWrite(leftForward, LOW);

}

void offsetBackward(double distance) {
  resetEncoders();
  // rightMotor.drive(-1*BACK_ADJUSTED_R);
  digitalWrite(rightBack, HIGH);
  digitalWrite(leftBack, HIGH);

  while (abs(encoderAverage()) < abs(distanceToEncoder(distance))) {
    if (readEncoder(LEFTENC)) leftEncoderCount += 1;
    if (readEncoder(RIGHTENC)) rightEncoderCount += 1;
  }
  digitalWrite(rightBack, LOW);
  digitalWrite(leftBack, LOW);

}

void offsetForwardSwitch(void) {
  resetEncoders();
  while (limitSwitch.getState() == LOW) limitSwitch.loop();
  digitalWrite(leftForward, HIGH);
  rightMotor.drive(FORWARD_ADJUSTED_R);
  while (limitSwitch.getState() != LOW) limitSwitch.loop();
  analogWrite(rightForward, 0);
  delay(100);
  digitalWrite(leftForward, LOW);
}

void findPWMDirection(void) {
  // ----- both wheels forward full speed
  digitalWrite(leftMotor.backPin, LOW);
  digitalWrite(rightMotor.backPin, LOW);
  digitalWrite(leftMotor.forwardPin, HIGH);
  digitalWrite(rightMotor.forwardPin, HIGH);
  delay(2000); 
  digitalWrite(leftMotor.forwardPin, LOW);
  digitalWrite(rightMotor.forwardPin, LOW);
  delay(500);
  // ----- both wheels back full speed
  digitalWrite(leftMotor.backPin, HIGH);
  digitalWrite(rightMotor.backPin, HIGH);
  delay(2000);
  digitalWrite(leftMotor.backPin, LOW);
  digitalWrite(rightMotor.backPin, LOW);
  delay(500);

  // ----- both wheels forward PWM
  // analogWrite(leftMotor.forwardPin, 200);
  analogWrite(rightMotor.forwardPin, 200);
  delay(2000);
  // digitalWrite(leftMotor.forwardPin, LOW);
  digitalWrite(rightMotor.forwardPin, LOW);
  delay(500);
  // both wheels back PWM
  // analogWrite(leftMotor.backPin, 200);
  analogWrite(rightMotor.backPin, 200);
  delay(2000);
  // digitalWrite(leftMotor.backPin, LOW);
  digitalWrite(rightMotor.backPin, LOW);
}







// -----------------------------------
// Code for testintg new inputs / wheel encoders.






// int inputPin = 7;
// int encoderCount = 0;
// int pastPinValue;
// int pinValue;
// int switchValue = HIGH; // value that encoder is updated on

// void setup() {
//   Serial.begin(9600);
//   pinMode(inputPin, INPUT);
//   pastPinValue = digitalRead(inputPin);
// }

// void loop() {

  // if(detectSwitch()) {
  //   encoderCount++;
  // }
//   Serial.print("Encoder count: ");
//   Serial.println(encoderCount);


// }

// bool detectSwitch(void) {
//   bool returnvar;
//   pinValue = digitalRead(inputPin);
//   if (pinValue != pastPinValue) {
//     returnvar = true;
//   } else {
//     returnvar = false;
//   }
//   pastPinValue = pinValue;
//   return returnvar;
// }
