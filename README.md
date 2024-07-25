# Micromouse Robot


## Abstract
The aim of this project is to design and build an autonomous robot to navigate an unknown maze. The goal is to reach the center from a specified corner as quickly as possible. This presentation highlights our approach, including sensor integration, motor control, and maze-solving algorithms. We'll discuss the challenges we faced and the solutions we developed, showcasing our robot's design and performance.

## Parts Used
- ESP32 microcontroller
- Motor Driver L298N
- 2 DC Motors with Encoders
- VL53L0X Lidar Sensor
- 2 IR Sensors
- Breadboard
- Jumper Wires
- 2 Kingtianli Batteries

## Testing Components and Steps
Videos demonstrating the tests and steps are available in the `assets` folder, and the corresponding code can be found in the `All_Arduino_codes` folder.

### DC Motor with Driver
https://github.com/user-attachments/assets/984f3a19-abe4-4103-b1a2-f43f0a341900
- **Code**: [motor_test](https://github.com/ssccddaasa/Micromouse/blob/main/All_Arduino_codes/motor_test)

### VL53L0X Lidar Sensor
https://github.com/user-attachments/assets/06fdd07d-8f0c-4055-903a-3346d7c9caf0
- **Code**: [Lidar_test](https://github.com/ssccddaasa/Micromouse/blob/main/All_Arduino_codes/Lider_test)

### IR Sensor
https://github.com/user-attachments/assets/17b84551-c2af-4a6c-b5ca-a864b0ea409b
- **Code**: [Ir_test](https://github.com/ssccddaasa/Micromouse/blob/main/All_Arduino_codes/Ir_test)

### Assembling Robot Parts
- **Image**: ![Assembled Robot](https://github.com/ssccddaasa/Micromouse/blob/main/assets/s2.jpg)

### Straight Movement with PID
https://github.com/user-attachments/assets/6658be3f-c49d-4d99-ad53-6eadb328a44f
- **Code**: [pid3](https://github.com/ssccddaasa/Micromouse/blob/main/All_Arduino_codes/pid3)

### 90-Degree Turn with PID
https://github.com/user-attachments/assets/bc0eccfa-7015-4189-a2bd-b056db2f02ae
- **Code**: [test_turn](https://github.com/ssccddaasa/Micromouse/blob/main/All_Arduino_codes/test_turn)

### Integrate and Implement Left-Roll Algorithm
Here is a brief explanation of how the final code works:

1. **Initialization**: Sets up the pins for motor control, encoders, and the VL53L0X sensor. Initializes the sensor and encoder counts.

2. **Main Loop**: Continuously checks for walls using the VL53L0X sensor. If a wall is detected, the robot performs a right turn. Otherwise, it moves forward.

3. **Wall Detection**: Uses the VL53L0X sensor to measure distances and determine if a wall is within a specified threshold.

4. **Movement Control**: Utilizes PID control to manage the speed and direction of the motors for both straight movement and turns.

5. **Turning**: Implements a 90-degree turn using PID control to ensure accurate and smooth rotation.

https://github.com/user-attachments/assets/04dee7b2-3f34-40ed-9896-07d5d3b773f8
- **Code**: [all](https://github.com/ssccddaasa/Micromouse/blob/main/All_Arduino_codes/all)

## Simulation
For the simulation part, we implemented the Left-Roll and FloodFill algorithms

https://github.com/user-attachments/assets/11214dd5-d883-43aa-8845-3466f076bfc0
- **Code**: [Simulation](https://github.com/ssccddaasa/Micromouse/blob/main/Simulation)


## Additional Resources
For more information, you can check out our detailed [Canva Presentation](https://www.canva.com/design/DAGKtb8idME/GT6D8ccrOrEDix6-eAkoUQ/view?utm_content=DAGKtb8idME&utm_campaign=designshare&utm_medium=link&utm_source=editor).


