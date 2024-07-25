#include <Adafruit_VL53L0X.h>
#include <Wire.h>

// Motor control pins for ESP32
const int en1 = 13;
const int in1 = 14;
const int in2 = 12;
const int en2 = 25;
const int in3 = 27;
const int in4 = 26;

// Encoder pins for ESP32
const int leftEncoderPinA = 33;
const int leftEncoderPinB = 32;
const int rightEncoderPinA = 34;
const int rightEncoderPinB = 35;

// Variables for encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;


// Variables for PID Control
long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

long previousTime2 = 0;
float ePrevious2 = 0;
float eIntegral2 = 0;

long previousTime3 = 0;
float ePrevious3 = 0;
float eIntegral3 = 0;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
#define WALL_THRESHOLD_MM 80 // Define the distance threshold for detecting a wall (in mm)

void setup() {
  Serial.begin(115200);

  // Set pin modes
  pinMode(en1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);

  // Interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), handleLeftEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), handleRightEncoderA, RISING);

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println(F("VL53L0X initialized"));

  delay(4000);
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}

void loop() {
  if (detectWall()) {
    turnRight();
  } else {
    moveForward();
  }
}

bool detectWall() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // Take a distance measurement

  if (measure.RangeStatus != 4) {  // If measurement is valid
    Serial.print("Distance (mm): ");
    //Serial.println(measure.RangeMilliMeter);

    if (measure.RangeMilliMeter < WALL_THRESHOLD_MM) {
      Serial.println("Wall detected!");
      return true;
    } else {
      Serial.println("No wall detected.");
    }
  } else {
    Serial.println("Measurement out of range");
  }
  return false;
}

void moveForward() {
  float speed1 = 80;
  // Set desired setpoint for right motor  
  int target = rightEncoderCount;
  // Move left motor forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en2, speed1); // Change the PWM value as needed

  // PID gains and computation
  float kp = 5.0;
  float kd = 0.0;
  float ki = 1.0;
  float u = pidController(target, kp, kd, ki);
  float speed2 = fabs(u);

  // Control right motor based on PID
  moveMotor2(in1, in2, en1, u);

  // Print statements for debugging
  Serial.print(rightEncoderCount);
  Serial.print(", ");
  Serial.println(leftEncoderCount);
}

void turnRight() {
  stopMotors();
  delay(1000);

  long initialLeftEncoderCount = leftEncoderCount; // Store initial encoder count
  long initialLeftEncoderCount2 = rightEncoderCount; // Store initial encoder count

  int targetTurnCount = 135; // Number of encoder counts for 90-degree turn
  float kp = 1.2; // Adjust these PID values as needed
  float kd = 1.0;
  float ki = 0.0;

  while ((leftEncoderCount - initialLeftEncoderCount) < targetTurnCount && (rightEncoderCount - initialLeftEncoderCount2) < targetTurnCount) {
    int target = initialLeftEncoderCount + targetTurnCount;
    float u = pidController2(target, kp, kd, ki);
    float speed = fabs(u);

    moveMotor(in1, in2, en1, u);
    moveMotor(in3, in4, en2, -u); // Reverse direction for the left motor

    // Print statements for debugging
    //Serial.print("Target: ");
    //Serial.print(target);
    //Serial.print(" Left Encoder: ");
    //Serial.print(leftEncoderCount);
    //Serial.print(" Right Encoder: ");
    //Serial.println(rightEncoderCount);
  }

  // Stop motors after the turn
  stopMotors();
  delay(2000);
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}



void stopMotors() {
  analogWrite(en1, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(en2, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Functions called during interrupts
void handleLeftEncoderA() {
  if (digitalRead(leftEncoderPinA) == digitalRead(leftEncoderPinB)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void handleRightEncoderA() {
  if (digitalRead(rightEncoderPinA) == digitalRead(rightEncoderPinB)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

float pidController(int target, float kp, float kd, float ki) {
  // Measure time elapsed since the last iteration
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

  // Compute the error, derivative, and integral
  int e = target - leftEncoderCount;
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  // Compute the PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

  // Update variables for the next iteration
  previousTime = currentTime;
  ePrevious = e;

  return u;
}



float pidController2(int target, float kp, float kd, float ki) {
  // Measure time elapsed since the last iteration
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime2)) / 1.0e6;

  // Compute the error, derivative, and integral
  int e = target - leftEncoderCount;
  float eDerivative = (e - ePrevious2) / deltaT;
  eIntegral2 = eIntegral2 + e * deltaT;

  // Compute the PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral2);

  // Update variables for the next iteration
  previousTime2 = currentTime;
  ePrevious2 = e;

  return u;
}



float pidController3(int target, float kp, float kd, float ki) {
  // Measure time elapsed since the last iteration
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime3)) / 1.0e6;

  // Compute the error, derivative, and integral
  int e = target - rightEncoderCount;
  float eDerivative = (e - ePrevious3) / deltaT;
  eIntegral3 = eIntegral3 + e * deltaT;

  // Compute the PID control signal
  float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral3);

  // Update variables for the next iteration
  previousTime3 = currentTime;
  ePrevious3 = e;

  return u;
}

void moveMotor(int dirPin1, int dirPin2, int pwmPin, float u) {
  // Maximum motor speed
  float speed = fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  // Control the motor
  if (u > 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  } else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  }
  analogWrite(pwmPin, speed);
}



void moveMotor2(int dirPin1, int dirPin2, int pwmPin, float u) {
  // Maximum motor speed
  float speed = fabs(u);
  if (speed > 255) {
    speed = 255;
  }
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  
  analogWrite(pwmPin, speed);
}