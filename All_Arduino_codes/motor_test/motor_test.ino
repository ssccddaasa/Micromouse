


#define ENCA2 12 // Encoder A for Motor 2
#define ENCB2 13 // Encoder B for Motor 2

volatile int posi1 = 0; // Encoder position for Motor 1
volatile int posi2 = 0; // Encoder position for Motor 2

// Motor Driver Pins
int enA1 = 27;
int in1_1 = 15;
int in2_1 = 2;
int enA2 = 18;
int in1_2 = 4;
int in2_2 = 16;

void setup() {
  Serial.begin(115200);

  // Set encoder pins as inputs
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);


  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);

  // Set motor driver pins as outputs
  pinMode(enA1, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);
  pinMode(enA2, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(in2_2, OUTPUT);
}

void loop() {
  // Test Motor 1
  digitalWrite(in1_1, HIGH);
  digitalWrite(in2_1, LOW);
  analogWrite(enA1, 255);

  // Test Motor 2
  digitalWrite(in1_2, HIGH);
  digitalWrite(in2_2, LOW);
  analogWrite(enA2, 255);

  // Read encoder positions in an atomic block
  int pos1 = 0;
  int pos2 = 0;

  pos1 = posi1;
  pos2 = posi2;


  Serial.print("Encoder Position Motor 2: ");
  Serial.println(pos2);

  // Delay to see changes
  delay(1000);
}


void readEncoder2(){
  int b = digitalRead(ENCB2);
  if(b > 0){
    posi2++;
  }
  else{
    posi2--;
  }
}
