#include <PID_v1.h>

// Motor driver pins
const int en1 = 13;
const int in1 = 12;
const int in2 = 14;
const int en2 = 27;
const int in3 = 25;
const int in4 = 26;

// Encoder pins
const int leftEncoderPinA = 34;
const int leftEncoderPinB = 35;
const int rightEncoderPinA = 32;
const int rightEncoderPinB = 33;

// Encoder variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long previousLeftEncoderCount = 0;
volatile long previousRightEncoderCount = 0;

// PID variables
double Setpoint, Input, Output;
double aggKp = 1.0, aggKi = 0.5, aggKd = 0.05; // Start with smaller values
double consKp = 1, consKi = 0.05, consKd = 0.01;

// PID instance
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

// Base speed for motors
const int baseSpeed = 150;

// ISR for left encoder
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(leftEncoderPinB) == HIGH) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

// ISR for right encoder
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(rightEncoderPinB) == HIGH) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Initialize motor driver pins
  pinMode(en1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize encoder pins
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoderISR, RISING);

  // Initialize PID
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // Calculate the number of ticks since the last loop
  long leftTicks = leftEncoderCount - previousLeftEncoderCount;
  long rightTicks = rightEncoderCount - previousRightEncoderCount;

  // Save current encoder counts for the next loop
  previousLeftEncoderCount = leftEncoderCount;
  previousRightEncoderCount = rightEncoderCount;

  // Calculate error
  Input = leftTicks - rightTicks;

  // Calculate gap to decide tuning parameters
  double gap = abs(Setpoint - Input); // distance away from setpoint
  if (gap < 10) {
    // We're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  } else {
    // We're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  // Compute PID output
  myPID.Compute();

  // Adjust motor speeds
  int leftMotorSpeed = baseSpeed + Output;
  int rightMotorSpeed = baseSpeed - Output;

  // Ensure motor speeds are within valid range
  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // Set motor directions and speeds
  if (leftMotorSpeed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    leftMotorSpeed = -leftMotorSpeed;
  }
  analogWrite(en1, leftMotorSpeed);

  if (rightMotorSpeed > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    rightMotorSpeed = -rightMotorSpeed;
  }
  analogWrite(en2, rightMotorSpeed);

  // Print variables for Serial Plotter
  Serial.print("Input: ");
  Serial.print(Input);
  Serial.print("Output: ");
  Serial.print(", ");
  Serial.println(Output);
  Serial.print(", Left Motor Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print(", Right Motor Speed: ");
  Serial.println(rightMotorSpeed);

  // Small delay for the loop
  //delay(100);
}
