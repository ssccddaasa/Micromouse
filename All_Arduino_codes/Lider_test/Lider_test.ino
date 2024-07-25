#include <Adafruit_VL53L0X.h>




#include <Wire.h>


Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define WALL_THRESHOLD_MM 80 // Define the distance threshold for detecting a wall (in mm)

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  Serial.println("Micromouse Wall Detection");

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println(F("VL53L0X initialized"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  
  lox.rangingTest(&measure, false); // Take a distance measurement

  if (measure.RangeStatus != 4) {  // If measurement is valid
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
    
    if (measure.RangeMilliMeter < WALL_THRESHOLD_MM) {
      Serial.println("Wall detected!");
      // Add code here to stop the robot or change its direction
    } else {
      Serial.println("No wall detected.");
      // Add code here to keep the robot moving forward
    }
  } else {
    Serial.println("Measurement out of range");
  }

  delay(3000); // Short delay before the next measurement
}
