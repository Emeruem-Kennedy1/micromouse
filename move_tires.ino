// Motor direction pins
#define motor1pin1 2  // GPIO2 instead of D2
#define motor1pin2 3  // GPIO3 instead of D3
#define motor2pin1 4  // GPIO4 instead of D4
#define motor2pin2 5  // GPIO5 instead of D5

// Motor speed pins
#define motor1speed 22  // GPIO6 for motor 1 speed
#define motor2speed 23 // GPIO7 for motor 2 speed

void setup() {
  // Set pin modes
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor1speed, OUTPUT);
  pinMode(motor2speed, OUTPUT);
  
  // Start serial for debugging
  Serial.begin(115200);
  Serial.println("Motor test starting...");
}

void forward() {
  // Set speeds
  analogWrite(motor1speed, 255);  // Full speed
  analogWrite(motor2speed, 255);  // Full speed
  
  // Set directions
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  
  Serial.println("Moving forward");
}

void backward() {
  // Set speeds
  analogWrite(motor1speed, 255);  // Full speed
  analogWrite(motor2speed, 255);  // Full speed
  
  // Set directions
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  
  Serial.println("Moving backward");
}

void stop() {
  // Stop motors by setting speed to 0
  analogWrite(motor1speed, 0);
  analogWrite(motor2speed, 0);
  
  // Or stop by setting both pins LOW
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  
  Serial.println("Stopped");
}

void loop() {
  forward();
  delay(2000);  // Move forward for 2 seconds
  
  stop();
  delay(1000);  // Stop for 1 second
  
  backward();
  delay(2000);  // Move backward for 2 seconds
  
  stop();
  delay(1000);  // Stop for 1 second
}