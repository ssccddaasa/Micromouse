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
const int rightEncoderPinA = 35;
const int rightEncoderPinB = 34;

// Variables for encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Variables for PID Control
long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

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
}

void loop() {
  float speed1 = 60;
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
  moveMotor(in1, in2, en1, u);

  // Print statements for debugging
  Serial.print(rightEncoderCount);
  Serial.print(", ");
  Serial.println(leftEncoderCount);
}

// Functions called during interrupts
void handleLeftEncoderA() {
  leftEncoderCount++;
}
void handleRightEncoderA() {
  rightEncoderCount++;
}

void moveMotor(int dirPin1, int dirPin2, int pwmPin, float u) {
  // Maximum motor speed
  float speed = fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  // Stop the motor during overshoot
  if (rightEncoderCount < leftEncoderCount) {
    speed = 0;
  }

  // Control the motor
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, LOW);
  analogWrite(pwmPin, speed);
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
