#include <Wire.h>
#include <stdint.h>


#define NMOTORS 2  // Number of motors
#define N 33  // Size of the maze
#define M (N - 1) / 2
#define WHEEL_RADIUS_CM 2.25  // Example value for wheel radius in cm
#define TICKS_PER_REVOLUTION 800  // Example value for encoder ticks per wheel revolution
int SPEED = 999; // The amount to increase targetPos while moving forward



// Motor pins
const int pwm[] = {9, 5};   // PWM pins for the motors
const int in1[] = {8, 4};   // IN1 pins for the motors
const int in2[] = {7, 6};   // IN2 pins for the motors

// Encoder pins
const int encA[] = {2, 19};   // Interrupt pins for encoder A
const int encB[] = {3, 18};   // Interrupt pins for encoder B

// Global variables for encoders
volatile long encoderPos[] = {0, 0};
volatile int lastEncoded[] = {0, 0};
long prevEncoderPos[] = {0, 0};
long targetPos[] = {0, 0};
int inputs[] = {0, 0};

long finalTargetPos[2] = {0, 0};
bool isTransitioning = false;

// State variables
enum RobotState { STOPPED, MOVING, ROTATING,INROTATION };
RobotState robotState = STOPPED;

// Robot position in centimeters
float x_cm = 0;
float y_cm = 0;

// Robot direction: 1 = up, 2 = right, 3 = down, 4 = left
int currentDirection = 3;


// PID variables
double kp = 5.0;
double ki = 0.4;
double kd = 0.1;
long prevError[] = {0, 0};
long integral[] = {0, 0};

unsigned long prevTime = 0;

// Button pins
const int buttonPin1 = 13;
const int buttonPin2 = 12;
const int buttonPin3 = 11;

// Variables to hold button states
bool button1Pressed = false;
bool button2Pressed = false;
bool button3Pressed = false;

// Debouncing variables
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;
unsigned long debounceDelay = 50;  // debounce time in milliseconds
unsigned long clock = 0;
// Previous button states for debouncing
int lastButtonState1 = HIGH;
int lastButtonState2 = HIGH;
int lastButtonState3 = HIGH;

// IR sensor pins
const int irPin1 = A0;
const int irPin2 = A1;
const int irPin3 = A2;

// Maze and flood fill matrices
uint8_t maze[N][N];           // The maze (0 for open space, 1 for walls)
uint8_t floodFillMatrix[M][M]; // Flood fill matrix to store distances for actual cells
int cellSize = 28;
int target_x = 7,target_y = 15;
int mode = 0;  // 0, reoresents idle state 

int floodFlag = 0;

// Function to set motor direction and speed
void setMotor(int motor, int dir, int pwmVal) {
  analogWrite(pwm[motor], pwmVal);
  if (dir == 1) {
    digitalWrite(in1[motor], HIGH);
    digitalWrite(in2[motor], LOW);
  } else {
    digitalWrite(in1[motor], LOW);
    digitalWrite(in2[motor], HIGH);
  }
}

// Encoder interrupt functions
void updateEncoder0() {
  updateEncoderGeneric(0);
}

void updateEncoder1() {
  updateEncoderGeneric(1);
}

void updateEncoderGeneric(int motor) {
  int MSB = digitalRead(encA[motor]);
  int LSB = digitalRead(encB[motor]);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded[motor] << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPos[motor]--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPos[motor]++;
  
  lastEncoded[motor] = encoded;
}

// Function to print the flood fill matrix to the Serial Monitor
void printFloodFillMatrix() {
    for (uint8_t i = 0; i < M; i++) {
        for (uint8_t j = 0; j < M; j++) {
            if (floodFillMatrix[i][j] == 255) {
                Serial.print("# ");
            } else {
                Serial.print(floodFillMatrix[i][j]);
                Serial.print(" ");
            }
        }
        Serial.println();
    }
    Serial.println();
}

void printMaze() {
  Serial.println("Maze:");
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      Serial.print(maze[i][j]);
    }
    Serial.println();
  }
  Serial.println();
}


bool isWallBetween(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    uint8_t wallX = x1 + (x2 - x1) / 2;
    uint8_t wallY = y1 + (y2 - y1) / 2;
    return maze[wallY][wallX] == 1;  // Note: y and x are swapped here to match maze orientation
}

// Recursive flood fill function
void floodFill(uint8_t x, uint8_t y, uint8_t currentDistance) {
    // Boundary check
    if (x >= M || y >= M) return;
    
    // If the current position has already been filled with a shorter distance, skip it
    if (floodFillMatrix[y][x] <= currentDistance) return;
    
    // Update the flood fill matrix with the current distance
    floodFillMatrix[y][x] = currentDistance;
    
    // Check and update adjacent cells
    if (x + 1 < M && !isWallBetween(2*x+1, 2*y+1, 2*(x+1)+1, 2*y+1))
        floodFill(x + 1, y, currentDistance + 1); // Right
    if (x > 0 && !isWallBetween(2*x+1, 2*y+1, 2*(x-1)+1, 2*y+1))
        floodFill(x - 1, y, currentDistance + 1); // Left
    if (y + 1 < M && !isWallBetween(2*x+1, 2*y+1, 2*x+1, 2*(y+1)+1))
        floodFill(x, y + 1, currentDistance + 1); // Down
    if (y > 0 && !isWallBetween(2*x+1, 2*y+1, 2*x+1, 2*(y-1)+1))
        floodFill(x, y - 1, currentDistance + 1); // Up
}

// Function to initialize and run the flood fill algorithm
void runFloodFill(uint8_t targetX, uint8_t targetY) {
    // Initialize the flood fill matrix with a high value (indicating unvisited cells)
    for (uint8_t i = 0; i < M; i++) {
        for (uint8_t j = 0; j < M; j++) {
            floodFillMatrix[i][j] = 255; // Initialize all cells as unvisited (max value for uint8_t)
        }
    }
    
    // Start flood filling from the target position
    floodFill(targetX, targetY, 0);
    
    // Print the flood fill matrix to the Serial Monitor
    // printFloodFillMatrix();
}



int nextDirection(float x, float y, int currentDirection) {
  int threshold = 2;
  int x_cell = round(x/cellSize);
  int y_cell = round(y/cellSize);

  // Store possible directions and their flood fill values
  int directions[4] = {1, 2, 3, 4}; // Up, Right, Down, Left
  int values[4] = {9999, 9999, 9999, 9999}; // Placeholder for flood fill values

  // Get the values in all four directions from the current cell
  if (y_cell > 0 && maze[y_cell*2][x_cell*2 + 1] == 0) values[0] = floodFillMatrix[y_cell-1][x_cell]; // Up
  if (x_cell < M - 1 && maze[y_cell*2 + 1][x_cell*2 + 2] == 0) values[1] = floodFillMatrix[y_cell][x_cell + 1]; // Right
  if (y_cell < M - 1 && maze[y_cell*2 + 2][x_cell*2 + 1] == 0) values[2] = floodFillMatrix[y_cell + 1][x_cell]; // Down
  if (x_cell > 0 && maze[y_cell*2 + 1][x_cell*2] == 0) values[3] = floodFillMatrix[y_cell][x_cell-1]; // Left

  // Find the minimum flood fill value among possible directions          
  int minValue = min(min(values[0], values[1]), min(values[2], values[3]));
  
  // Collect directions with the minimum value
  int possibleDirections[4];
  int count = 0;
  for (int i = 0; i < 4; i++) {
    if (values[i] == minValue) {
      possibleDirections[count] = directions[i];
      count++;
    }
  }

  // If the current direction is among the best options, keep it
  for (int i = 0; i < count; i++) {
    if (possibleDirections[i] == currentDirection) {
      return currentDirection;
    }
  }
  
  // If not, choose a random direction among the best options
  if(currentDirection == 2 || currentDirection == 4){
    if(abs(x_cm - cellSize*x_cell) < threshold){
      x_cm = x_cell*cellSize;
      y_cm = y_cell*cellSize;
      return possibleDirections[random(0, count)];
    }
  }

  if(currentDirection == 1 || currentDirection == 3){
    if(abs(y_cm - cellSize*y_cell) < threshold){
        x_cm = x_cell*cellSize;
        y_cm = y_cell*cellSize;
      return possibleDirections[random(0, count)];
    }
  }
  // return possibleDirections[random(0, count)];
}





void updateMaze(int x, int y, int value) {
    maze[y][x] = value;
}




// Function to determine and update maze walls
void checkAndUpdateWalls(float irLeft, float irRight, float irFront) {
  Serial.print("updating wall");
  // Calculate the closest cell's center in cm
  int threshold = 3;
  int wallThreshold = 15;
  int currentCellX = round(x_cm / cellSize);
  int currentCellY = round(y_cm / cellSize);
  int direction = currentDirection;
  // Calculate the distance from the robot to the cell center
  float distToCellCenterX = abs(x_cm - (currentCellX * cellSize));
  float distToCellCenterY = abs(y_cm - (currentCellY * cellSize));
  // Only update maze if within threshold distance to the cell center
  if (distToCellCenterX < threshold && distToCellCenterY < threshold) {
    // Check the IR sensor redings and update walls accordingly
    
    if (irFront < wallThreshold) {
        Serial.println("maze updated");
        if(direction == 1) updateMaze(2*currentCellX + 1,2*currentCellY,1); //up
        if(direction == 2) updateMaze(2*currentCellX + 2,2*currentCellY+1,1); //right
        if(direction == 3) updateMaze(2*currentCellX + 1,2*currentCellY+2,1); //down
        if(direction == 4) updateMaze(2*currentCellX ,2*currentCellY+1,1); //left
        
    }

    // Update the left wall
    if (irLeft < wallThreshold) {
        if(direction == 1) updateMaze(2*currentCellX,2*currentCellY + 1,1); //up
        if(direction == 2) updateMaze(2*currentCellX + 1,2*currentCellY,1); //right
        if(direction == 3) updateMaze(2*currentCellX + 2,2*currentCellY+1,1); //down
        if(direction == 4) updateMaze(2*currentCellX + 1,2*currentCellY+2,1); //left
    }

    // Update the right wall
    if (irRight < wallThreshold) {
        if(direction == 1) updateMaze(2*currentCellX + 2,2*currentCellY+1,1); //up
        if(direction == 2) updateMaze(2*currentCellX+1,2*currentCellY+2,1); //right
        if(direction == 3) updateMaze(2*currentCellX,2*currentCellY+1,1); //down
        if(direction == 4) updateMaze(2*currentCellX +1,2*currentCellY,1); //left
    }
    if (floodFlag == 0) {
      runFloodFill(target_x,target_y);
      Serial.println("floodFill ran");
      floodFlag = 1;
    }


  }else{
    floodFlag = 0;
  }
}






void smoothTransition(long newTargetPos[2], float transitionSpeed) {
  finalTargetPos[0] = newTargetPos[0];
  finalTargetPos[1] = newTargetPos[1];
  isTransitioning = true;
}

void updateSmoothTransition(float deltaTime) {
  if (!isTransitioning) return;

  float maxChange = SPEED * deltaTime;
  bool transitionComplete = true;

  for (int i = 0; i < 2; i++) {
    long difference = finalTargetPos[i] - targetPos[i];
    if (abs(difference) > maxChange) {
      targetPos[i] += (difference > 0) ? maxChange : -maxChange;
      transitionComplete = false;
    } else {
      targetPos[i] = finalTargetPos[i];
    }
  }

  if (transitionComplete) {
    isTransitioning = false;
  }
}


void updateMotion(int desiredDirection) {
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  
  float deltaTime = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  updateSmoothTransition(deltaTime);

  if (robotState == MOVING) {
    int speed = SPEED * 0.02;
    if (currentDirection == desiredDirection) {
      targetPos[0] += speed;
      targetPos[1] += speed;
      
      // Update x_cm and y_cm based on wheel ticks
      float distanceMoved = (encoderPos[0] - prevEncoderPos[0]) * (2 * PI * WHEEL_RADIUS_CM / TICKS_PER_REVOLUTION);
      if (currentDirection == 1) y_cm -= distanceMoved;
      else if (currentDirection == 2) x_cm += distanceMoved;
      else if (currentDirection == 3) y_cm += distanceMoved;
      else if (currentDirection == 4) x_cm -= distanceMoved;
      
      prevEncoderPos[0] = encoderPos[0];
      prevEncoderPos[1] = encoderPos[1];
    } else {
      robotState = ROTATING;
    }
  } else if (robotState == ROTATING || robotState == INROTATION) {
    if (currentDirection != desiredDirection) {
      long rotationAmount90 = -590;  // Example value for a 90-degree turn
      long rotationAmount180 = rotationAmount90 * 2;
      
      int turnDifference = abs(desiredDirection - currentDirection);
      if (robotState != INROTATION) {
        long newTargetPos[2] = {targetPos[0], targetPos[1]};
        if (turnDifference == 2) {
          // 180-degree turn
          long rotationAmount = (desiredDirection - currentDirection == 2 || currentDirection - desiredDirection == 2) ? rotationAmount180 : -rotationAmount180;
          newTargetPos[0] += rotationAmount;
          newTargetPos[1] -= rotationAmount;
        } else if (turnDifference == 1 || turnDifference == 3) {
          // 90-degree turn
          long rotationAmount = ((desiredDirection - currentDirection == 1) || (desiredDirection - currentDirection == -3)) ? rotationAmount90 : -rotationAmount90;
          newTargetPos[0] += rotationAmount;
          newTargetPos[1] -= rotationAmount;
        }
        smoothTransition(newTargetPos, SPEED);
        robotState = INROTATION;
      }
      
      if (!isTransitioning) {
        robotState = MOVING;
        currentDirection = desiredDirection;
      }
    }
    prevEncoderPos[0] = encoderPos[0];
    prevEncoderPos[1] = encoderPos[1];
  } else if (robotState == STOPPED) {
    robotState = MOVING;
  }
}
// Function to update the maze (for example, if a wall is added)

void wallCorrectionAndAlignment(float irLeft, float irRight, float irFront) {
  // Threshold values for wall proximity detection
  int wallProximityThreshold = 10;
  int alignmentThreshold = 1.4;  // Adjust this for fine-tuning alignment correction
  int WallThreshold = 20;

  // Calculate the adjustment needed if the robot is too close to a side wall
  if(irRight<WallThreshold){// Too close to the left wall, adjust targetPos to steer slightly right
    if( irRight > wallProximityThreshold){
      targetPos[0] -= alignmentThreshold;
      targetPos[1] += alignmentThreshold;
    }else{
      targetPos[0] += alignmentThreshold;
      targetPos[1] -= alignmentThreshold;
    }
      
}else if(irLeft < WallThreshold){
  if( irLeft > wallProximityThreshold){
    targetPos[0]+= alignmentThreshold;
      targetPos[1] -= alignmentThreshold;
  }else{
    targetPos[0]-= alignmentThreshold;
      targetPos[1] += alignmentThreshold;
  }
}
}
  
void pidcontrol(){
    unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  
  for (int i = 0; i < NMOTORS; i++) {
    long error = targetPos[i] - encoderPos[i];
    integral[i] += error * deltaTime;
    float derivative = (error - prevError[i]) / deltaTime;
    
    float output = kp * error + ki * integral[i] + kd * derivative;
    
    int power = constrain(abs(output), 0, 200);
    int direction = (output >= 0) ? 1 : -1;
    
    setMotor(i, direction, power);
    
    prevError[i] = error;
    
    // Update target position

  }
  
  }

void setup() {
  Serial.begin(9600);
  
  // Set up motor pins
  for (int i = 0; i < NMOTORS; i++) {
    pinMode(pwm[i], OUTPUT);
    pinMode(in1[i], OUTPUT);
    pinMode(in2[i], OUTPUT);
    
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(encA[0]), updateEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[0]), updateEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[1]), updateEncoder1, CHANGE);
  
  // Set button pins as inputs with internal pull-up resistors
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);

  
  
  // Initialize the maze
  uint8_t initialMaze[N][N] = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    };
  
  for (uint8_t i = 0; i < N; i++) {
        for (uint8_t j = 0; j < N; j++) {
            maze[i][j] = initialMaze[i][j];
        }
    }

  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 1000;

  TCCR1B |= (1 << WGM12);

  TCCR1B |= (1 << CS11) | (1<< CS10);

  TIMSK1 |= (1 << OCIE1A);

  sei();

}
int count = 0;
void loop() {
  count++;
  // Serial.println(count);
  // Read button states with debouncing
  
  int currentButtonState1 = digitalRead(buttonPin1);
  if (currentButtonState1 != lastButtonState1) {
    lastDebounceTime1 = millis();
  }
  if ((millis() - lastDebounceTime1) > debounceDelay) {
    button1Pressed = (currentButtonState1 == LOW);
  }
  lastButtonState1 = currentButtonState1;

  int currentButtonState2 = digitalRead(buttonPin2);
  if (currentButtonState2 != lastButtonState2) {
    lastDebounceTime2 = millis();
  }
  if ((millis() - lastDebounceTime2) > debounceDelay) {
    button2Pressed = (currentButtonState2 == LOW);
  }
  lastButtonState2 = currentButtonState2;

  int currentButtonState3 = digitalRead(buttonPin3);
  if (currentButtonState3 != lastButtonState3) {
    lastDebounceTime3 = millis();
  }
  if ((millis() - lastDebounceTime3) > debounceDelay) {
    button3Pressed = (currentButtonState3 == LOW);
  }
  lastButtonState3 = currentButtonState3;

  // Read IR sensors
  int irValue1 = analogRead(irPin1);
  float voltage1 = irValue1 * (5.0 / 1023.0);
  float distance1 = 27.86 * pow(voltage1, -1.15);
  int irValue2 = analogRead(irPin2);
  float voltage2 = irValue2 * (5.0 / 1023.0);
  float distance2 = 27.86 * pow(voltage2, -1.15);

  int irValue3 = analogRead(irPin3);
  float voltage3 = irValue3 * (5.0 / 1023.0);
  float distance3 = 27.86 * pow(voltage3, -1.15);

  // Update motor control with PID
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;
  
  if(currentTime-clock > 7000 && mode == 1){
    mode = 2;
  }
  // Example control based on buttons
  if (button1Pressed) {
    printMaze();
    printFloodFillMatrix();
  }
  if (button2Pressed) {
    mode = 1;
    clock = millis();  
    Serial.println("button2 pressed");
  }
  if (button3Pressed) {
    clock = millis();
    mode = 0;  // Wait for WDT to reset the Arduino
    x_cm = 0;
    y_cm = 0;
    currentDirection = 3;
  }
   
  if (mode == 2){
    
    if(robotState != ROTATING && robotState != INROTATION){

    checkAndUpdateWalls(distance3,distance1,distance2);
    wallCorrectionAndAlignment(distance3,distance1,distance2);

    }
    // runFloodFill(target_row,htarget_column);
    int nextdirection = nextDirection(x_cm,y_cm,currentDirection);
    // Serial.println(nextdirection);
    updateMotion(nextdirection); 
    

    if(round(x_cm/cellSize) == target_x && round(y_cm/cellSize) == target_y){
      mode = 0;
    }

  delay(10);  // Adjust loop speed as needed

}


ISR(TIMER1_COMPA_vect){
  pidcontrol();
}