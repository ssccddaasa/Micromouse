#include <Arduino.h>

// Motor control pins for ESP32
const int en1 = 13;
const int in1 = 12;
const int in2 = 14;
const int en2 = 27;
const int in4 = 26;
const int in3 = 25;

// Encoder pins for ESP32
const int leftEncoderPinA = 35;
const int leftEncoderPinB = 34;
const int rightEncoderPinA = 32;
const int rightEncoderPinB = 33;

// Global variables
volatile int leftPos = 0;
volatile int rightPos = 0;
volatile long prevTime = 0;

float leftV1Filt = 0;
float leftV1Prev = 0;
float rightV1Filt = 0;
float rightV1Prev = 0;

float leftEIntegral = 0;
float rightEIntegral = 0;

// PID constants
float Kp = 1.2;  // Fine-tune proportional gain
float Ki = 0.5;  // Fine-tune integral gain
float Kd = 0.0;  // Add a derivative gain

float Kpr = 0.8;
float Kir = 0.4;
float Kdr = 0.1;

// Function prototypes
void readLeftEncoder();
void readRightEncoder();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);

void setup() {
  Serial.begin(115200);

  // Setup motor control pins
  pinMode(en1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Setup encoder pins
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);

  // Attach interrupt for encoders
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), readRightEncoder, RISING);

  prevTime = micros();
}
float leftPrevError = 0;
float rightPrevError = 0;

void loop() {
  // Read and compute velocities for both motors
  long currentTime = micros();
  float deltaTime = (currentTime - prevTime) / 1.0e6;
  prevTime = currentTime;

  int leftPosCopy, rightPosCopy;
  noInterrupts(); // Disable interrupts to safely read shared variables
  leftPosCopy = leftPos;
  rightPosCopy = rightPos;
  leftPos = 0;  // Reset for next interval
  rightPos = 0; // Reset for next interval
  interrupts(); // Re-enable interrupts

  // Compute velocities
  float leftV1 = (float) leftPosCopy / deltaTime;
  float rightV1 = (float) rightPosCopy / deltaTime;

  // Print raw encoder counts for debugging
  Serial.print("Left Pos: ");
  Serial.print(leftPosCopy);
  Serial.print(", Right Pos: ");
  Serial.println(rightPosCopy);

  // Low-pass filter for velocities (25 Hz cutoff)
  leftV1Filt = 0.854 * leftV1Filt + 0.0728 * leftV1 + 0.0728 * leftV1Prev;
  leftV1Prev = leftV1;
  rightV1Filt = 0.854 * rightV1Filt + 0.0728 * rightV1 + 0.0728 * rightV1Prev;
  rightV1Prev = rightV1;

  // Target speed (e.g., 100 RPM)
  float targetSpeed = 100.0;

  // PID control for left motor
  float leftError = targetSpeed - leftV1Filt;
  leftEIntegral += leftError * deltaTime;
  float leftUDerivative = (leftError - leftPrevError) / deltaTime;
  float leftU = Kp * leftError + Ki * leftEIntegral + Kd * leftUDerivative;
  leftPrevError = leftError;

  // PID control for right motor
  float rightError = targetSpeed - rightV1Filt;
  rightEIntegral += rightError * deltaTime;
  float rightUDerivative = (rightError - rightPrevError) / deltaTime;
  float rightU = Kpr * rightError + Kir * rightEIntegral + Kdr * rightUDerivative;
  rightPrevError = rightError;

  // Set motor speeds and directions
  int leftDir = (leftU < 0) ? -1 : 1;
  int leftPwm = min(abs((int) leftU), 255);
  setMotor(leftDir, leftPwm, en1, in1, in2);

  int rightDir = (rightU < 0) ? -1 : 1;
  int rightPwm = min(abs((int) rightU), 255);
  setMotor(rightDir, rightPwm, en2, in3, in4);

  // Debug output
  Serial.print("Left: ");
  Serial.print(leftV1Filt);
  Serial.print(" RPM, Right: ");
  Serial.print(rightV1Filt);
  Serial.println(" RPM");

  delay(20); // Small delay to prevent overwhelming the serial output
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Set motor speed
  if (dir == 1) {
    // Move forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    // Move backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}

void readLeftEncoder() {
  // Read left encoder B to determine direction
  int b = digitalRead(leftEncoderPinB);
  int increment = (b > 0) ? 1 : -1;
  leftPos += increment;
}

void readRightEncoder() {
  // Read right encoder B to determine direction
  int b = digitalRead(rightEncoderPinB);
  int increment = (b > 0) ? 1 : -1;
  rightPos += increment;
}
