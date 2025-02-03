#include <ESP32Servo.h>
#include <Ultrasonic.h>

// Motor direction pins
#define motor1pin1 4
#define motor1pin2 3
#define motor2pin1 6
#define motor2pin2 7
#define switchpin 13
#define SERVO_PIN 9

#define trigpin 10
#define echopin 11

// Motor speed pins
#define motor1speed D2
#define motor2speed D5

// Servo positions
#define FRONT 90
#define LEFT 180
#define RIGHT 15

// Constants for navigation
#define OBSTACLE_DISTANCE 5    
#define TURN_DURATION 1900     
#define TURNING_SPEED 150      
#define FORWARD_SPEED 255      
#define LEFT_TRIM 0            
#define RIGHT_TRIM 0           
#define ACCEL_STEP 10         
#define ACCEL_DELAY 10        
#define SCAN_DELAY 500        

// Maze configuration
#define GRID_SIZE 8           
#define CELL_SIZE 22          
#define MOVEMENT_CHECK_TIME 1300 

// Direction definitions
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Global variables
bool pathPrinted = false;
int switch_state;
Servo ultrasonicServo;
Ultrasonic ultrasonic(trigpin, echopin);
bool returnJourney = false;

// Current position and direction
struct Position {
    int x = 0;
    int y = 0;
    int facing = NORTH;
} currentPos;

// Cell structure for maze mapping
struct Cell {
    bool isWall = false;
    bool visited = false;
    int distanceToCenter;  // Modified to use minimum distance to any center cell
    int totalCost;
};

Cell maze[GRID_SIZE][GRID_SIZE];

// Path history tracking
#define MAX_HISTORY 64
struct CellHistory {
    int x[MAX_HISTORY];
    int y[MAX_HISTORY];
    int count = 0;
} pathHistory;

// Structure for scanning results
struct ScanResult {
    long leftDistance;
    long frontDistance;
    long rightDistance;
};

// Function to check if a cell is a center cell
bool isCenterCell(int x, int y) {
    return (x == 3 && y == 3) || (x == 3 && y == 4) || 
           (x == 4 && y == 3) || (x == 4 && y == 4);
}

// Calculate minimum Manhattan distance to any center cell
int calculateDistanceToCenter(int x, int y) {
    int minDist = GRID_SIZE * 2;  // Initialize to max possible distance
    int centerCoords[4][2] = {{3,3}, {3,4}, {4,3}, {4,4}};
    
    for(int i = 0; i < 4; i++) {
        int dist = abs(x - centerCoords[i][0]) + abs(y - centerCoords[i][1]);
        if(dist < minDist) {
            minDist = dist;
        }
    }
    return minDist;
}

// Initialize the maze
void initializeMaze() {
    for (int x = 0; x < GRID_SIZE; x++) {
        for (int y = 0; y < GRID_SIZE; y++) {
            maze[x][y].isWall = false;
            maze[x][y].visited = false;
            maze[x][y].distanceToCenter = calculateDistanceToCenter(x, y);
            maze[x][y].totalCost = maze[x][y].distanceToCenter;
        }
    }
}

// Update costs when a wall is found
void updateDistances(int x, int y) {
    if(x < 0 || x >= GRID_SIZE || y < 0 || y >= GRID_SIZE) return;
    
    maze[x][y].isWall = true;
    
    // Only update costs for immediate neighbors
    int dx[] = {0, 1, 0, -1};  // Adjacent cells
    int dy[] = {1, 0, -1, 0};
    
    for(int i = 0; i < 4; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        
        if(nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && !maze[nx][ny].isWall) {
            // Increase cost for cells adjacent to walls
            maze[nx][ny].totalCost = maze[nx][ny].distanceToCenter + 2;
        }
    }
}

// Find the best next move considering walls and center direction
Position findBestMove() {
    Position bestMove = currentPos;
    int lowestCost = INT_MAX;
    bool foundMove = false;
    
    int dx[] = {0, 1, 0, -1};  // North, East, South, West
    int dy[] = {1, 0, -1, 0};
    
    // First priority: unvisited cells that lead to center
    for (int i = 0; i < 4; i++) {
        int newX = currentPos.x + dx[i];
        int newY = currentPos.y + dy[i];
        
        if (newX >= 0 && newX < GRID_SIZE && newY >= 0 && newY < GRID_SIZE) {
            if (!maze[newX][newY].isWall && !maze[newX][newY].visited) {
                int cost = maze[newX][newY].totalCost;
                if (cost < lowestCost) {
                    lowestCost = cost;
                    bestMove.x = newX;
                    bestMove.y = newY;
                    foundMove = true;
                }
            }
        }
    }
    
    // If no unvisited cells, allow revisiting cells (except walls)
    if (!foundMove) {
        lowestCost = INT_MAX;
        for (int i = 0; i < 4; i++) {
            int newX = currentPos.x + dx[i];
            int newY = currentPos.y + dy[i];
            
            if (newX >= 0 && newX < GRID_SIZE && newY >= 0 && newY < GRID_SIZE) {
                if (!maze[newX][newY].isWall) {
                    int cost = maze[newX][newY].totalCost;
                    if (cost < lowestCost) {
                        lowestCost = cost;
                        bestMove.x = newX;
                        bestMove.y = newY;
                        foundMove = true;
                    }
                }
            }
        }
    }
    
    return bestMove;
}

// Add current position to history
void addToHistory() {
    if (pathHistory.count < MAX_HISTORY) {
        pathHistory.x[pathHistory.count] = currentPos.x;
        pathHistory.y[pathHistory.count] = currentPos.y;
        pathHistory.count++;
        
        Serial.print("Added to history: (");
        Serial.print(currentPos.x);
        Serial.print(",");
        Serial.print(currentPos.y);
        Serial.println(")");
    }
}

// Update robot's position based on facing direction
void updatePosition() {
    int newX = currentPos.x;
    int newY = currentPos.y;
    
    switch(currentPos.facing) {
        case NORTH: newY++; break;
        case EAST:  newX++; break;
        case SOUTH: newY--; break;
        case WEST:  newX--; break;
    }
    
    // Apply grid boundaries
    newX = constrain(newX, 0, GRID_SIZE-1);
    newY = constrain(newY, 0, GRID_SIZE-1);
    
    // Update position
    currentPos.x = newX;
    currentPos.y = newY;
    addToHistory();
    
    Serial.print("Moved to (");
    Serial.print(currentPos.x);
    Serial.print(",");
    Serial.print(currentPos.y);
    Serial.println(")");
}

// Update robot's facing direction after a turn
void updateDirection(bool turnRight) {
    if(turnRight) {
        currentPos.facing = (currentPos.facing + 1) % 4;
    } else {
        currentPos.facing = (currentPos.facing + 3) % 4;
    }
    
    String directions[] = {"NORTH", "EAST", "SOUTH", "WEST"};
    Serial.print("Now facing ");
    Serial.println(directions[currentPos.facing]);
}

// Servo control functions
void lookFront() {
    ultrasonicServo.write(FRONT);
    delay(150);
}

void lookLeft() {
    ultrasonicServo.write(LEFT);
    delay(150);
}

void lookRight() {
    ultrasonicServo.write(RIGHT);
    delay(150);
}

// Scan surroundings and return distances
ScanResult scanSurroundings() {
    ScanResult scan;
    
    lookLeft();
    delay(SCAN_DELAY);
    scan.leftDistance = ultrasonic.read();
    
    lookFront();
    delay(SCAN_DELAY);
    scan.frontDistance = ultrasonic.read();
    
    lookRight();
    delay(SCAN_DELAY);
    scan.rightDistance = ultrasonic.read();
    
    lookFront();
    return scan;
}

// Movement functions
void forward() {
    if (switch_state == HIGH) {
        Serial.println("Starting forward movement");

        digitalWrite(motor1pin1, HIGH);
        digitalWrite(motor1pin2, LOW);
        digitalWrite(motor2pin1, HIGH);
        digitalWrite(motor2pin2, LOW);

        // Acceleration
        for (int speed = 155; speed <= FORWARD_SPEED; speed += 50) {
            analogWrite(motor1speed, speed + LEFT_TRIM);
            analogWrite(motor2speed, speed + RIGHT_TRIM);
            delay(20);
        }

        unsigned long startTime = millis();
        while ((millis() - startTime) < MOVEMENT_CHECK_TIME) {
            long currentDistance = ultrasonic.read();
            
            if (currentDistance <= OBSTACLE_DISTANCE + 2) {
                Serial.println("Obstacle detected - stopping");
                stop();
                return;
            }
            delay(100);
        }

        stop();
        updatePosition();
    }
}

void turnRight90() {
    if(switch_state == HIGH) {
        analogWrite(motor1speed, TURNING_SPEED);
        analogWrite(motor2speed, TURNING_SPEED);

        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, HIGH);
        digitalWrite(motor2pin1, HIGH);
        digitalWrite(motor2pin2, LOW);
        
        delay(TURN_DURATION);
        stop();
        
        updateDirection(true);
    }
}

void turnLeft90() {
    if(switch_state == HIGH) {
        analogWrite(motor1speed, TURNING_SPEED);
        analogWrite(motor2speed, TURNING_SPEED);
        
        digitalWrite(motor1pin1, HIGH);
        digitalWrite(motor1pin2, LOW);
        digitalWrite(motor2pin1, LOW);
        digitalWrite(motor2pin2, HIGH);
        
        delay(TURN_DURATION);
        stop();
        
        updateDirection(false);
    }
}

void stop() {
    analogWrite(motor1speed, 0);
    analogWrite(motor2speed, 0);
    
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
}

// Modified autonomous navigation
void navigateAutonomously() {
    long currentDistance = ultrasonic.read();
    
    // Check if we've reached any center cell
    if (isCenterCell(currentPos.x, currentPos.y)) {
        Serial.println("Reached center! Stopping navigation.");
        stop();
        return;
    }
    
    if (currentDistance <= OBSTACLE_DISTANCE) {
        stop();
        Serial.println("Obstacle detected, updating maze");
        
        // Calculate wall position
        int wallX = currentPos.x;
        int wallY = currentPos.y;
        
        switch(currentPos.facing) {
            case NORTH: wallY++; break;
            case EAST:  wallX++; break;
            case SOUTH: wallY--; break;
            case WEST:  wallX--; break;
        }
        
        // Update wall information and recalculate paths
        if (wallX >= 0 && wallX < GRID_SIZE && wallY >= 0 && wallY < GRID_SIZE) {
            updateDistances(wallX, wallY);
        }
        
        // Find best move considering walls and visited cells
        Position nextMove = findBestMove();
        
        // If no valid move found, try turning right and scanning again
        if (nextMove.x == currentPos.x && nextMove.y == currentPos.y) {
            turnRight90();
            return;
        }
        
        // Determine optimal direction to face
        int targetDirection;
        if (nextMove.x > currentPos.x) targetDirection = EAST;
        else if (nextMove.x < currentPos.x) targetDirection = WEST;
        else if (nextMove.y > currentPos.y) targetDirection = NORTH;
        else targetDirection = SOUTH;
        
        // Turn to face target direction
        while (currentPos.facing != targetDirection) {
            turnRight90();
        }
    }
    else {
        maze[currentPos.x][currentPos.y].visited = true;
        forward();
    }
}

// Setup function
void setup() {
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    ultrasonicServo.setPeriodHertz(50);
    ultrasonicServo.attach(SERVO_PIN, 500, 2400);
    
    pinMode(motor1pin1, OUTPUT);
    pinMode(motor1pin2, OUTPUT);
    pinMode(motor2pin1, OUTPUT);
    pinMode(motor2pin2, OUTPUT);
    pinMode(motor1speed, OUTPUT);
    pinMode(motor2speed, OUTPUT);
    pinMode(switchpin, INPUT_PULLUP);
    
    lookFront();
    Serial.begin(115200);
    Serial.println("Maze solver starting at position (0,0)");
    
    initializeMaze();
    addToHistory();
}

// Main loop
void loop() {
    switch_state = digitalRead(switchpin);
    
    if(switch_state == HIGH) {
        pathPrinted = true;
        navigateAutonomously();
        delay(100);
    }
    else {
        stop();
        // if (pathPrinted) {
            Serial.println("\n=== FINAL PATH SUMMARY ===");
            Serial.print("Total cells visited: ");
            Serial.println(pathHistory.count);
            
            Serial.println("\nComplete Path:");
            for (int i = 0; i < pathHistory.count; i++) {
                Serial.print("(");
                Serial.print(pathHistory.x[i]);
                Serial.print(",");
                Serial.print(pathHistory.y[i]);
                Serial.print(")");
                
                if (i < pathHistory.count - 1) {
                    int dx = pathHistory.x[i+1] - pathHistory.x[i];
                    int dy = pathHistory.y[i+1] - pathHistory.y[i];
                    if (dx == 1) Serial.print(" → ");
                    else if (dx == -1) Serial.print(" ← ");
                    else if (dy == 1) Serial.print(" ↑ ");   // North
                    else if (dy == -1) Serial.print(" ↓ ");  // South
                }
            }
            Serial.println("\n======================\n");
            
            // Print maze state
            Serial.println("\nMaze State:");
            for (int y = GRID_SIZE-1; y >= 0; y--) {
                for (int x = 0; x < GRID_SIZE; x++) {
                    if (maze[x][y].isWall) Serial.print("W ");
                    else if (maze[x][y].visited) Serial.print("V ");
                    else Serial.print(". ");
                }
                Serial.println();
            }
            
            Serial.println("\nCosts to Center:");
            for (int y = GRID_SIZE-1; y >= 0; y--) {
                for (int x = 0; x < GRID_SIZE; x++) {
                    Serial.print(maze[x][y].totalCost);
                    Serial.print("\t");
                }
                Serial.println();
            }
            
            pathPrinted = false;
        // }
        delay(100);
    }
}