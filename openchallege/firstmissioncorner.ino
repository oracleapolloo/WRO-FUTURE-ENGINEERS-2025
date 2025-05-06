//It solves the first OPen challenge left corner

#include <Servo.h>

// Pin definitions
#define MOTOR_ENB 3 // PWM for motor speed
#define MOTOR_IN3 4 // Motor direction 1
#define MOTOR_IN4 5 // Motor direction 2
#define SERVO_PIN 9 // Servo for steering
#define TRIG_LEFT A3 // Left ultrasonic trigger
#define ECHO_LEFT A2 // Left ultrasonic echo
#define TRIG_FRONT 11 // Front ultrasonic trigger
#define ECHO_FRONT 10 // Front ultrasonic echo
#define BUTTON_PIN 2 // Pull-up pushbutton

// Constants
const float TARGET_DISTANCE = 20.0; // cm from left wall
const float DISTANCE_THRESHOLD = 2.0; // cm tolerance (±2 cm)
const float STOP_DISTANCE = 7.0; // cm for front obstacle
const float CORNER_DISTANCE = 50.0; // cm to detect corner
const int MOTOR_SPEED = 100; // PWM 0-255
const int STEER_STRAIGHT = 90; // Servo angle for straight
const int STEER_LEFT = 120; // Servo angle for left (wall following)
const int STEER_RIGHT = 60; // Servo angle for right
const int CORNER_STEER_LEFT = 150; // Servo angle for corner turn
const unsigned long CORNER_CONFIRM_TIME = 200; // ms to confirm corner

// Servo object
Servo steeringServo;

// Variables
bool isMoving = false; // Tracks if robot is moving
unsigned long cornerStartTime = 0; // For corner confirmation
bool cornerPending = false; // Tracks if corner is being confirmed

void setup() {
  // Initialize pins
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Internal pull-up

  // Initialize servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(STEER_STRAIGHT);

  // Start serial for debugging
  Serial.begin(9600);
}

void loop() {
  // Check button press (LOW when pressed)
  if (digitalRead(BUTTON_PIN) == LOW) {
    isMoving = !isMoving; // Toggle moving state
    if (!isMoving) {
      stopMotors(); // Stop when toggled off
      cornerPending = false; // Reset corner detection
    }
    Serial.print("Moving: ");
    Serial.println(isMoving ? "ON" : "OFF");
    delay(200); // Simple debounce
  }

  // Get distances
  float distanceLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  float distanceFront = getDistance(TRIG_FRONT, ECHO_FRONT);

  // Steering decision
  int steeringAngle = STEER_STRAIGHT; // Default
  if (isMoving) {
    if (distanceFront < STOP_DISTANCE) {
      stopMotors();
      isMoving = false;
      cornerPending = false; // Reset corner detection
      Serial.println("Stopped (obstacle)");
    } else if (distanceLeft > CORNER_DISTANCE) {
      // Start or continue corner confirmation
      if (!cornerPending) {
        cornerStartTime = millis();
        cornerPending = true;
      }
      // Check if corner condition persists for 200 ms
      if (cornerPending && (millis() - cornerStartTime >= CORNER_CONFIRM_TIME)) {
        Serial.println("Corner turn: Steering 150 deg");
        // Turn left until wall is detected
        while (getDistance(TRIG_LEFT, ECHO_LEFT) > 20.0) {
          steeringServo.write(CORNER_STEER_LEFT); // Sharp left
          setMotorSpeed(MOTOR_SPEED);
          distanceLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
          distanceFront = getDistance(TRIG_FRONT, ECHO_FRONT);
          // Stop if front obstacle detected during turn
          if (distanceFront < STOP_DISTANCE) {
            stopMotors();
            isMoving = false;
            Serial.println("Stopped (obstacle during corner)");
            break;
          }
          // Print during turn
          Serial.print("Turning | Left: ");
          Serial.print(distanceLeft);
          Serial.print(" cm | Front: ");
          Serial.print(distanceFront);
          Serial.println(" cm | Steering: 150 deg");
          delay(50); // Sensor stability
        }
        cornerPending = false; // Reset after turn
      }
    } else {
      // Reset corner detection if wall is close
      cornerPending = false;
      // Normal wall following
      if (distanceLeft < (TARGET_DISTANCE - DISTANCE_THRESHOLD)) {
        steeringAngle = STEER_RIGHT; // Too close, turn right
      } else if (distanceLeft > (TARGET_DISTANCE + DISTANCE_THRESHOLD)) {
        steeringAngle = STEER_LEFT; // Too far, turn left
      }
    }
  }

  // Apply steering
  steeringServo.write(steeringAngle);

  // Move if active and no corner turn
  if (isMoving && distanceFront >= STOP_DISTANCE && !cornerPending) {
    setMotorSpeed(MOTOR_SPEED);
  }

  // Print values
  Serial.print("Moving: ");
  Serial.print(isMoving ? "ON" : "OFF");
  Serial.print(" | Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm | Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm | Steering: ");
  Serial.print(steeringAngle);
  Serial.println(" deg");

  delay(50); // Avoid sensor interference
}

// Get distance from ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  float distance = duration * 0.034 / 2;
  if (distance < 1 || distance > 200) distance = 50.0; // Sanity check
  return distance;
}

// Set motor speed and direction
void setMotorSpeed(int speed) {
  digitalWrite(MOTOR_IN3, HIGH); // Forward
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENB, speed);
}

// Stop motors
void stopMotors() {
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENB, 0);
}