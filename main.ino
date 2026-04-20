// SMART OBSTACLE AVOIDANCE CAR WITH 3-SECOND STATIONARY CHECK
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>

// Pin Definitions
#define TRIG_PIN A0
#define ECHO_PIN A1
#define SERVO_PIN 10

// Distance Settings
#define MAX_DISTANCE 200
#define OBSTACLE_DISTANCE 20  // Distance to trigger avoidance (cm)
#define CLEAR_PATH_DISTANCE 30 // Considered clear path (cm)
#define MOVEMENT_THRESHOLD 3   // Minimum change to consider obstacle moving (cm)

// Timing Settings
#define STATIONARY_CHECK_TIME 3000 // 3 seconds to confirm obstacle is fixed
#define BACKUP_TIME 800  // Time to move backward (ms)
#define TURN_TIME 500    // Time for 90-degree turn (ms)

// Speed Settings
#define MAX_SPEED 150
#define BACKUP_SPEED 100

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Servo myservo;

// Motor connections
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

void setup() {
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);
  centerServo();
  delay(1000);
  Serial.println("Smart Obstacle Avoidance Car Ready");
}

void loop() {
  int distance = getFilteredDistance();
  
  if (distance <= OBSTACLE_DISTANCE) {
    handleObstacle();
  } else {
    moveForward();
  }
  delay(50);
}

void handleObstacle() {
  // 1. Stop immediately
  moveStop();
  Serial.println("\n! OBSTACLE DETECTED !");
  
  // 2. Check if obstacle remains stationary for 3 seconds
  if (checkIfObstacleIsFixed()) {
    Serial.println("Obstacle is stationary - initiating avoidance");
    avoidObstacle();
  } else {
    Serial.println("Obstacle moved - continuing forward");
  }
}

bool checkIfObstacleIsFixed() {
  Serial.println("Checking if obstacle is stationary for 3 seconds...");
  unsigned long startTime = millis();
  int initialDistance = getFilteredDistance();
  
  while (millis() - startTime < STATIONARY_CHECK_TIME) {
    int currentDistance = getFilteredDistance();
    
    // If path clears
    if (currentDistance > CLEAR_PATH_DISTANCE) {
      Serial.println("Path cleared!");
      return false;
    }
    
    // If obstacle moves significantly
    if (abs(currentDistance - initialDistance) > MOVEMENT_THRESHOLD) {
      Serial.println("Obstacle is moving");
      return false;
    }
    
    delay(100);
  }
  Serial.println("Obstacle confirmed stationary");
  return true;
}

void avoidObstacle() {
  // 1. Move backward
  Serial.println("Moving backward...");
  moveBackward();
  delay(BACKUP_TIME);
  moveStop();
  delay(200);
  
  // 2. Scan surroundings
  Serial.println("Scanning surroundings...");
  int rightDist = scanRight();
  int leftDist = scanLeft();
  
  Serial.print("Right: "); Serial.print(rightDist); Serial.print("cm | ");
  Serial.print("Left: "); Serial.print(leftDist); Serial.println("cm");
  
  // 3. Decide turn direction
  if (rightDist > leftDist && rightDist > CLEAR_PATH_DISTANCE) {
    Serial.println("Turning RIGHT");
    turnRight();
  } else if (leftDist > CLEAR_PATH_DISTANCE) {
    Serial.println("Turning LEFT");
    turnLeft();
  } else {
    Serial.println("No clear path - TURNING AROUND");
    turnAround();
  }
  
  // 4. Move forward in new direction
  Serial.println("Moving forward in new direction");
  moveForward();
}

// Sensor and movement functions
int getFilteredDistance() {
  // Take multiple readings for accuracy
  int d1 = sonar.ping_cm();
  delay(30);
  int d2 = sonar.ping_cm();
  delay(30);
  int d3 = sonar.ping_cm();
  
  // Return median distance
  if (d1 > d2) swap(d1, d2);
  if (d2 > d3) swap(d2, d3);
  if (d1 > d2) swap(d1, d2);
  return d2 == 0 ? MAX_DISTANCE : d2;
}

int scanRight() {
  myservo.write(30);  // Look right
  delay(500);
  int dist = getFilteredDistance();
  centerServo();
  return dist;
}

int scanLeft() {
  myservo.write(150);  // Look left
  delay(500);
  int dist = getFilteredDistance();
  centerServo();
  return dist;
}

void centerServo() {
  myservo.write(90);
  delay(300);
}

// Movement functions
void moveForward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  setSpeed(MAX_SPEED);
}

void moveBackward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  setSpeed(BACKUP_SPEED);
}

void moveStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  setSpeed(MAX_SPEED);
  delay(TURN_TIME);
  moveStop();
}

void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  setSpeed(MAX_SPEED);
  delay(TURN_TIME);
  moveStop();
}

void turnAround() {
  turnRight();
  delay(100);
  turnRight();
}

void setSpeed(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}

void swap(int &a, int &b) {
  int temp = a;
  a = b;
  b = temp;
}