#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <NewPing.h>

// Pin definitions
#define TRIG_PIN_LEFT A3     // Left ultrasonic trigger
#define ECHO_PIN_LEFT A2     // Left ultrasonic echo
#define TRIG_PIN_FRONT 7     // Front ultrasonic trigger
#define ECHO_PIN_FRONT 6     // Front ultrasonic echo
#define TRIG_PIN_RIGHT A1    // Right ultrasonic trigger
#define ECHO_PIN_RIGHT A0    // Right ultrasonic echo
#define SERVO_PIN 10          // Servo signal
#define MOTOR_ENB 3          // L298N speed control
#define MOTOR_IN3 4          // L298N direction 1
#define MOTOR_IN4 5          // L298N direction 2
#define BUTTON_PIN 2         // Push button

// Ultrasonic setup
#define MAX_DISTANCE 500     // Max distance (cm)
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// MPU6050 and Servo
MPU6050 mpu(Wire);
Servo steeringServo;

// Constants
const int MOTOR_SPEED = 140;  // PWM speed (0-255)
const float TURN_DISTANCE = 70; // Edge detection distance (cm)
const int TOTAL_EDGES = 12;   // 4 corners * 3 laps
const int SERVO_CENTER = 90;  // Servo center (degrees)
const int SERVO_MAX_LEFT = 150; // Max left (degrees)
const int SERVO_MAX_RIGHT = 30; // Max right (degrees)
const float KP = 2.0;         // Gyro correction gain

// Variables
bool isClockwise = false;
bool directionSet = false;
int edgeCount = 0;
bool isRunning = false;
float targetAngle = 0;
bool lastButtonState = HIGH;

void resetState() {
  Serial.println("Resetting to fresh state...");
  isClockwise = false;
  directionSet = false;
  edgeCount = 0;
  targetAngle = 0;
  steeringServo.write(SERVO_CENTER);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  Serial.println("Recalibrating MPU6050...");
  mpu.calcOffsets(); // Recalibrate gyro
  Serial.println("Reset complete: Servo centered, motor forward, gyro calibrated.");
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (mpu.begin() != 0) {
    Serial.println("MPU6050 initialization failed!");
    while (1);
  }
  Serial.println("Calibrating MPU6050...");
  mpu.calcOffsets(); // Initial calibration
  Serial.println("MPU6050 ready.");

  // Initialize pins
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);
  Serial.println("Servo set to center (90 degrees).");

  // Set motor forward
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  Serial.println("Motor set to forward direction.");
  Serial.println("Waiting for button press to start...");
}

void loop() {
  // Check button for toggle
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    if (!isRunning) {
      Serial.println("Button pressed: Starting navigation...");
      resetState(); // Reset for fresh start
      isRunning = true;
    } else {
      Serial.println("Button pressed: Stopped!");
      analogWrite(MOTOR_ENB, 0); // Stop motors
      isRunning = false;
    }
    delay(200); // Debounce
  }
  lastButtonState = currentButtonState;

  if (!isRunning) return;

  // Update gyro
  mpu.update();
  float currentAngle = mpu.getAngleZ();

  // Move straight with gyro correction
  float error = targetAngle - currentAngle;
  int servoAngle = SERVO_CENTER + (int)(KP * error);
  servoAngle = constrain(servoAngle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
  steeringServo.write(servoAngle);
  analogWrite(MOTOR_ENB, MOTOR_SPEED);
  Serial.print("Moving straight: Gyro angle=");
  Serial.print(currentAngle);
  Serial.print(", Target angle=");
  Serial.print(targetAngle);
  Serial.print(", Servo angle=");
  Serial.println(servoAngle);

  // Check for corner
  float leftDist = sonarLeft.ping_cm();
  float rightDist = sonarRight.ping_cm();
  Serial.print("Ultrasonic: Left=");
  Serial.print(leftDist);
  Serial.print(" cm, Right=");
  Serial.println(rightDist);

  if (!directionSet) {
    // Detect first corner to set direction
    if (rightDist > TURN_DISTANCE && rightDist < MAX_DISTANCE) {
      isClockwise = true;
      directionSet = true;
      edgeCount++;
      Serial.print("First corner detected (Right): Direction set to Clockwise, Edge count=");
      Serial.println(edgeCount);
    } else if (leftDist > TURN_DISTANCE && leftDist < MAX_DISTANCE) {
      isClockwise = false;
      directionSet = true;
      edgeCount++;
      Serial.print("First corner detected (Left): Direction set to Counterclockwise, Edge count=");
      Serial.println(edgeCount);
    }
  }

  // Continue navigation after direction is set
  if (directionSet) {
    float sideDist = isClockwise ? rightDist : leftDist;
    if (sideDist > TURN_DISTANCE && sideDist < MAX_DISTANCE) {
      edgeCount++;
      Serial.print("Edge detected: Edge count=");
      Serial.println(edgeCount);
      if (edgeCount >= TOTAL_EDGES) {
        analogWrite(MOTOR_ENB, 0); // Stop
        isRunning = false;
        Serial.println("Finished 3 laps!");
        while (1); // Halt
      }

      // Turn 90 degrees
      float startAngle = currentAngle;
      float turnTarget = startAngle + (isClockwise ? -70 : 70);
      Serial.print("Initiating 90-degree turn: Target angle=");
      Serial.println(turnTarget);
      analogWrite(MOTOR_ENB, 0); // Stop motors
      steeringServo.write(isClockwise ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT);
      Serial.print("Servo set to ");
      Serial.print(isClockwise ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT);
      Serial.println(" for turn.");
      while (abs(mpu.getAngleZ() - turnTarget) > 2) {
        mpu.update();
        analogWrite(MOTOR_ENB, 100); // Slow turn
        Serial.print("Turning: Current angle=");
        Serial.println(mpu.getAngleZ());
      }
      analogWrite(MOTOR_ENB, 0);
      steeringServo.write(SERVO_CENTER);
      targetAngle = turnTarget; // Update target
      Serial.println("Turn complete: Servo reset to center.");
    }
  }
}