// Define constants for pin numbers
const int ir1Pin = 13;
const int ir2Pin = 12;

// Variables for interrupt handling
volatile bool ir1Alert = false;
volatile bool ir2Alert = false;

// Interrupt service routine for IR sensor 1
void IRAM_ATTR irISR1() {
  ir1Alert = true;
}

// Interrupt service routine for IR sensor 2
void IRAM_ATTR irISR2() {
  ir2Alert = true;
}

void setup() {
  Serial.begin(115200);

  // Set up interrupt pins
  pinMode(ir1Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ir1Pin), irISR1, RISING);

  pinMode(ir2Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ir2Pin), irISR2, RISING);
}

void loop() {
  // Check if IR sensor 1 triggered an interrupt
  if (ir1Alert) {
    Serial.println("IR sensor 1 detected!");
    ir1Alert = false; // Reset the alert flag
  }

  // Check if IR sensor 2 triggered an interrupt
  if (ir2Alert) {
    Serial.println("IR sensor 2 detected!");
    ir2Alert = false; // Reset the alert flag
  }

  // Simulate a time-consuming process
  delay(100);
}
