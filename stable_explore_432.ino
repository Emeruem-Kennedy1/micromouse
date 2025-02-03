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
#define OBSTACLE_DISTANCE 5    // Distance in cm to trigger obstacle detection
#define TURN_DURATION 1900     // Calibrated duration for 90-degree turn (milliseconds)
#define TURNING_SPEED 150      // Speed during turns
#define FORWARD_SPEED 255      // Speed for forward movement
#define LEFT_TRIM 0            // Adjustment for left motor (-20 to +20)
#define RIGHT_TRIM 0           // Adjustment for right motor (-20 to +20)
#define ACCEL_STEP 10           // Speed increment for acceleration
#define ACCEL_DELAY 10         // Milliseconds between speed increases
#define SCAN_DELAY 500         // Delay during scanning (milliseconds)

// Maze configuration
#define GRID_SIZE 8            // 8x8 grid
#define CELL_SIZE 22           // 22cm per cell (updated to match your maze)
#define MOVEMENT_CHECK_TIME 1300 // Time to move one cell (calibrated for 22cm)
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Global variables
bool pathPrinted = false;
int switch_state;
Servo ultrasonicServo;
Ultrasonic ultrasonic(trigpin, echopin);

// Current position and direction
struct Position {
    int x = 0;
    int y = 0;
    int facing = NORTH;  // Start facing North
} currentPos;

// Cell history tracking
#define MAX_HISTORY 64  // Maximum number of cells in history (8x8 grid)

struct CellHistory {
    int x[MAX_HISTORY];
    int y[MAX_HISTORY];
    int count = 0;
} pathHistory;

// Structure to store scanning results
struct ScanResult {
    long leftDistance;
    long frontDistance;
    long rightDistance;
};

// Function prototypes
void addToHistory();
bool isInHistory(int x, int y);
int getVisitCount(int x, int y);
void updatePosition();
void updateDirection(bool turnRight);
void lookFront();
void lookLeft();
void lookRight();
ScanResult scanSurroundings();
void forward();
void turnRight90();
void turnLeft90();
void stop();
void navigateAutonomously();

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
    addToHistory(); // Add starting position to history
}

// Add current position to history
void addToHistory() {
    if (pathHistory.count < MAX_HISTORY) {
        pathHistory.x[pathHistory.count] = currentPos.x;
        pathHistory.y[pathHistory.count] = currentPos.y;
        pathHistory.count++;
        
        // Print current path for debugging
        Serial.print("Added to history: (");
        Serial.print(currentPos.x);
        Serial.print(",");
        Serial.print(currentPos.y);
        Serial.println(")");
    }
}

// Check if a cell is in history
bool isInHistory(int x, int y) {
    for (int i = 0; i < pathHistory.count; i++) {
        if (pathHistory.x[i] == x && pathHistory.y[i] == y) {
            return true;
        }
    }
    return false;
}

// Get visit count for a cell
int getVisitCount(int x, int y) {
    int count = 0;
    for (int i = 0; i < pathHistory.count; i++) {
        if (pathHistory.x[i] == x && pathHistory.y[i] == y) {
            count++;
        }
    }
    return count;
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
    
    // Debug log
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
        currentPos.facing = (currentPos.facing + 3) % 4;  // +3 is same as -1 with modulo 4
    }
    
    String directions[] = {"NORTH", "EAST", "SOUTH", "WEST"};
    Serial.print("Turned ");
    Serial.print(turnRight ? "RIGHT" : "LEFT");
    Serial.print(". Now facing ");
    Serial.println(directions[currentPos.facing]);
}

// Move servo to look forward
void lookFront() {
    ultrasonicServo.write(FRONT);
    delay(150);
}

// Move servo to look left
void lookLeft() {
    ultrasonicServo.write(LEFT);
    delay(150);
}

// Move servo to look right
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
    Serial.print("Left distance: ");
    Serial.println(scan.leftDistance);
    
    lookFront();
    delay(SCAN_DELAY);
    scan.frontDistance = ultrasonic.read();
    Serial.print("Front distance: ");
    Serial.println(scan.frontDistance);
    
    lookRight();
    delay(SCAN_DELAY);
    scan.rightDistance = ultrasonic.read();
    Serial.print("Right distance: ");
    Serial.println(scan.rightDistance);
    
    lookFront();
    return scan;
}

// Move forward one cell
void forward() {
    if (switch_state == HIGH) {
        Serial.println("Starting forward movement");

        // Set direction
        digitalWrite(motor1pin1, HIGH);
        digitalWrite(motor1pin2, LOW);
        digitalWrite(motor2pin1, HIGH);
        digitalWrite(motor2pin2, LOW);

        // Short acceleration phase
        for (int speed = 155; speed <= FORWARD_SPEED; speed += 50) {
            analogWrite(motor1speed, speed + LEFT_TRIM);
            analogWrite(motor2speed, speed + RIGHT_TRIM);
            delay(20);  // Short delay for smoother acceleration
        }

        // Move for fixed time or until obstacle
        unsigned long startTime = millis();
        while ((millis() - startTime) < MOVEMENT_CHECK_TIME) {
            long currentDistance = ultrasonic.read();
            Serial.print("Current distance: ");
            Serial.println(currentDistance);

            delay(100);

            if (currentDistance <= OBSTACLE_DISTANCE + 2) {
                Serial.println("Obstacle detected - stopping");
                stop();
                return;  // Exit without updating position
            }
        }

        stop();
        updatePosition(); // Update position after successful movement
    }
}

// Turn right 90 degrees
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

// Turn left 90 degrees
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

// Stop all motors
void stop() {
    analogWrite(motor1speed, 0);
    analogWrite(motor2speed, 0);
    
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
}

// Autonomous navigation logic
void navigateAutonomously() {
    long currentDistance = ultrasonic.read();
    
    if(currentDistance <= OBSTACLE_DISTANCE) {
        stop();
        Serial.println("Obstacle detected, scanning surroundings");
        delay(100); // Small delay to ensure complete stop
        
        ScanResult scan = scanSurroundings();
        
        if(scan.leftDistance > scan.rightDistance && scan.leftDistance > OBSTACLE_DISTANCE + 2) {
            Serial.println("Turning left - more space");
            turnLeft90();
        }
        else if(scan.rightDistance > OBSTACLE_DISTANCE + 2) {
            Serial.println("Turning right - more space");
            turnRight90();
        }
        else {
            Serial.println("Dead end - turning around");
            turnRight90();
            turnRight90();
        }
    }
    else {
        forward();
    }
}

// Main loop
void loop() {
    switch_state = digitalRead(switchpin);
    
    if(switch_state == HIGH) {
        pathPrinted = true;  // Reset flag when switch is turned on
        navigateAutonomously();
        delay(100);
    }
    else {
        stop();
        // Print path history when switch is turned off
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
                    // Show direction of movement between points
                    int dx = pathHistory.x[i+1] - pathHistory.x[i];
                    int dy = pathHistory.y[i+1] - pathHistory.y[i];
                    if (dx == 1) Serial.print(" → ");       // East
                    else if (dx == -1) Serial.print(" ← ");  // West
                    else if (dy == 1) Serial.print(" ↑ ");   // North
                    else if (dy == -1) Serial.print(" ↓ ");  // South
                }
            }
            Serial.println("\n======================\n");
            pathPrinted = false; // Reset flag after printing
        // }
        delay(100);
    }
}