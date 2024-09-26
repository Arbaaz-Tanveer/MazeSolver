# WRC Maze Solver Robot

This project was developed from scratch for the World Robotics Championship (WRC), where robots navigate through complex mazes autonomously. The robot uses advanced algorithms and precise hardware to efficiently solve and traverse mazes.

![WRC Competition Image](./images/Robot%20in%20maze.jpeg)

## Table of Contents
1. [Competition Overview](#competition-overview)
2. [Hardware](#hardware)
3. [Software](#software)
4. [Setup](#setup)
5. [Future Improvements](#future-improvements)

## Competition Overview

The World Robotics Championship challenges to create autonomous robots capable of navigating complex mazes. Robots must efficiently explore, map, and solve the maze while adhering to strict competition rules.

## Hardware

The robot's hardware was carefully selected to balance performance, reliability, and ease of implementation:

### Main Components:
- **Microcontroller**: Arduino Mega
- **Motors**: N20 1000rpm motors with encoders
- **Sensors**: Sharp IR sensors for wall detection
- **Chassis**: Zero pcb board

![Robot Hardware](./images/Maze%20Solver.jpeg)

### Key Hardware Decisions:

1. **Arduino Mega**: opted for the Arduino Mega instead of the Nano due to its increased number of interrupt pins, which were crucial for our encoder implementation,STM 32 was preferred but because of less time remaining for the competition and increased complexity of STM32 arduino mega was choosen.

2. **N20 Motors with Encoders**: These compact motors with built-in encoders provide precise control over movement. The encoders allow for accurate position tracking, essential for navigating the maze reliably.

3. **IR Sensors**: Used three IR sensors (left, front, right) for wall detection, enabling the robot to map its surroundings and make navigation decisions.

## Software

Our software implementation combines several algorithms and techniques to efficiently solve the maze:

1. **Flood Fill Algorithm**: The core of our maze-solving strategy. It helps determine the optimal path to the goal and unlike wall hugging algorithms it doesn't get stuck because of free standing wall.

2. **PID Control**: Implements precise motor control for smooth movement and turns.

3. **Wall Detection and Mapping**: Continuously updates the maze map based on sensor readings.

4. **Orientation Tracking**: Keeps track of the robot's direction within the maze.

5. **Dynamic Path Planning**: Recalculates the optimal path as new information about the maze is discovered.

6. **State machine architecture**: The robot's behavior is managed through a state machine, allowing for clear transitions between different modes of operation (e.g., exploration, backtracking, goal-seeking).

7. **Modular code structure**: The software is organized into distinct modules for sensor management, motor control, maze solving, and high-level decision making, improving maintainability and allowing for easier future enhancements.
Key software features:

- Real-time maze mapping
- Efficient path recalculation
- Smooth motion control
- Error handling and recovery strategies

## Setup

To set up and run the project:

1. Clone this repository
2. Install the Arduino IDE
3. Install required libraries (Wire.h,stdint.h)
4. Connect the hardware components according to the pin configuration in the code
5. Upload the code to your Arduino Mega
6. Place the robot at the start of the maze and power it on and dont forget to specify the maze size and the target position.

## Future Improvements

1. **Migrate to STM32**: While we chose Arduino for its simplicity and our time constraints, migrating to an STM32 microcontroller could offer improved performance and more advanced features.

2. **Optimize Flood Fill**: Further refine our flood fill algorithm for even faster maze solving.

3. **Enhance Sensor Array**: Add more sensors for improved environmental awareness and obstacle detection.

3. **Develop Custom PCB**: Design a custom PCB to reduce wiring complexity and improve reliability.

4. **Use of Vaccuum fan**: Design ,test and implement a vaccuum suction device for improved friction to avaoid slippage which adds up to position inaccuracy.

---

I welcome contributions and suggestions to improve Maze Solver Robot.






























