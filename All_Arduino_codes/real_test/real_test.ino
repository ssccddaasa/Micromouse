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
  

}

void loop() {
  float speed1 = 200;
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(en2, speed1); // Change the PWM value as needed
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(en1, speed1); // Change the PWM value as needed


}

