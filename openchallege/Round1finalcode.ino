#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <NewPing.h>

// Pin definitions
#define TRIG_PIN_LEFT A3     // Left ultrasonic trigger
#define ECHO_PIN_LEFT A2     // Left ultrasonic echo
#define TRIG_PIN_RIGHT A1    // Right ultrasonic trigger
#define ECHO_PIN_RIGHT A0    // Right ultrasonic echo
#define SERVO_PIN 10         // Servo signal
#define MOTOR_ENB 3          // L298N speed control
#define MOTOR_IN3 4          // L298N direction 1
#define MOTOR_IN4 5          // L298N direction 2
#define BUTTON_PIN 2         // Push button

// Ultrasonic setup
#define MAX_DISTANCE 500     // Max distance (cm)
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);

// MPU6050 and Servo
MPU6050 mpu(Wire);
Servo steeringServo;

// Constants
const int MOTOR_SPEED = 220;  // PWM speed
const int SLOW_SPEED = 150;  // Reduced speed for turns
const float SIDE_TURN_DISTANCE = 90; // Side sensor trigger (cm)
const int TOTAL_EDGES = 12;   // 4 corners * 3 laps
const int SERVO_CENTER = 90;  // Servo center
const int SERVO_MAX_LEFT = 150; // Max left
const int SERVO_MAX_RIGHT = 40; // Max right
const float KP = 2.0;         // Gyro correction gain
const float KW = 0.5;         // Inner wall-following gain
const float KO = 1.0;         // Outer wall avoidance gain (increased)
const float TARGET_WALL_DISTANCE = 30; // Target inner wall distance (cm)
const float OUTER_WALL_THRESHOLD = 30; // Outer wall crash threshold (cm) (increased)
const unsigned long WAIT_DURATION = 1500; // 1s wait post-turn

// Variables
bool isClockwise = false;
bool directionSet = false;
int edgeCount = 0;
bool isRunning = false;
float targetAngle = 0;
bool lastButtonState = HIGH;
bool isWaiting = false;
unsigned long waitStartTime = 0;

float getFilteredDistance(NewPing &sonar) {
  const int samples = 5; // Increased for better reliability
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    float dist = sonar.ping_cm();
    if (dist == 0) dist = MAX_DISTANCE;
    sum += dist;
    delay(10);
  }
  return sum / samples;
}

void resetState() {
  isClockwise = false;
  directionSet = false;
  edgeCount = 0;
  targetAngle = 0;
  isWaiting = false;
  steeringServo.write(SERVO_CENTER); // Servo to 90°
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENB, 0); // Motor off
  mpu.calcOffsets(); // Recalibrate MPU6050
  Serial.println("State reset: Servo at 90°, motor off, MPU6050 recalibrated.");
}

void performTurn(bool clockwise) {
  mpu.update();
  float startAngle = mpu.getAngleZ();
  float turnTarget = startAngle + (clockwise ? -85 : 85);
  Serial.print("Initiating 90-degree turn: Target=");
  Serial.println(turnTarget);

  analogWrite(MOTOR_ENB, 0);
  steeringServo.write(clockwise ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT);
  while (abs(mpu.getAngleZ() - turnTarget) > 2) {
    mpu.update();
    analogWrite(MOTOR_ENB, SLOW_SPEED);
  }
  analogWrite(MOTOR_ENB, 0);
  steeringServo.write(SERVO_CENTER);
  targetAngle = turnTarget;
  Serial.println("Turn complete: Servo reset to center.");
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (mpu.begin() != 0) {
    Serial.println("MPU6050 failed!");
    while (1);
  }
  // INITIAL RESET: Set servo to 90°, motor off, configure pins
  mpu.calcOffsets(); // Calibrate MPU6050
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER); // Servo to 90°
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(MOTOR_ENB, 0); // Motor off
  Serial.println("Setup complete: Servo at 90°, motor off, ready for button press.");
}

void loop() {
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    if (!isRunning) {
      Serial.println("Button pressed: Starting navigation...");
      resetState(); // Reset for fresh start
      isRunning = true;
    } else {
      Serial.println("Button pressed: Stopping and resetting...");
      resetState(); // Full reset to fresh state
      isRunning = false;
    }
    delay(200); // Debounce
  }
  lastButtonState = currentButtonState;

  if (!isRunning) return;

  mpu.update();
  float currentAngle = mpu.getAngleZ();
  float leftDist = getFilteredDistance(sonarLeft);
  float rightDist = getFilteredDistance(sonarRight);

  // Gyro, inner wall, and outer wall correction
  float gyroError = targetAngle - currentAngle;
  int servoAngle = SERVO_CENTER;
  float outerError = 0; // Initialize for debugging
  if (directionSet && !isWaiting) {
    float innerDist = isClockwise ? leftDist : rightDist; // Inner wall
    float outerDist = isClockwise ? rightDist : leftDist; // Outer wall
    float wallError = TARGET_WALL_DISTANCE - innerDist;  // Inner wall error (~30 cm)
    outerError = outerDist < OUTER_WALL_THRESHOLD ? OUTER_WALL_THRESHOLD - outerDist : 0; // Outer wall error (<30 cm)
    // DRIFTING LOGIC: Adjusts servo to drift toward/away from inner wall (~30 cm) and inward if outer wall <30 cm
    servoAngle += (int)(KP * gyroError + KW * wallError + KO * outerError);
  } else {
    servoAngle += (int)(KP * gyroError);
  }
  servoAngle = constrain(servoAngle, SERVO_MAX_RIGHT, SERVO_MAX_LEFT);
  steeringServo.write(servoAngle);

  int currentSpeed = MOTOR_SPEED;
  analogWrite(MOTOR_ENB, currentSpeed);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    Serial.print("Angle=");
    Serial.print(currentAngle);
    Serial.print(", Target=");
    Serial.print(targetAngle);
    Serial.print(", Servo=");
    Serial.print(servoAngle);
    Serial.print(", Speed=");
    Serial.print(currentSpeed);
    Serial.print(" | Left=");
    Serial.print(leftDist);
    Serial.print(", Right=");
    Serial.print(rightDist);
    Serial.print(", OuterError=");
    Serial.println(outerError);
    lastPrint = millis();
  }

  if (!directionSet) {
    if (rightDist > SIDE_TURN_DISTANCE && rightDist < MAX_DISTANCE) {
      isClockwise = true;
      directionSet = true;
      edgeCount++;
      Serial.print("First corner: Clockwise, Edge=");
      Serial.println(edgeCount);
      performTurn(isClockwise);
      isWaiting = true;
      waitStartTime = millis();
    } else if (leftDist > SIDE_TURN_DISTANCE && leftDist < MAX_DISTANCE) {
      isClockwise = false;
      directionSet = true;
      edgeCount++;
      Serial.print("First corner: Anticlockwise, Edge=");
      Serial.println(edgeCount);
      performTurn(isClockwise);
      isWaiting = true;
      waitStartTime = millis();
    }
  }

  if (directionSet) {
    if (isWaiting) {
      if (millis() - waitStartTime >= WAIT_DURATION) {
        isWaiting = false;
        Serial.println("Wait complete");
      } else {
        return;
      }
    }

    float outerDist = isClockwise ? rightDist : leftDist; // Outer wall for edge
    if (outerDist > SIDE_TURN_DISTANCE && outerDist < MAX_DISTANCE) {
      edgeCount++;
      Serial.print("Edge detected: Count=");
      Serial.println(edgeCount);
      if (edgeCount >= TOTAL_EDGES) {
        Serial.println("Finished 3 laps! Performing final maneuver...");
        performTurn(isClockwise); // 85-degree turn in same direction
        Serial.println("Moving forward for 750 ms...");
        steeringServo.write(SERVO_CENTER); // Straight
        analogWrite(MOTOR_ENB, MOTOR_SPEED); // Move forward
        delay(750); // 750 ms
        analogWrite(MOTOR_ENB, 0); // Stop motor
        isRunning = false;
        Serial.println("Final maneuver complete. Vehicle stopped.");
        while (1);
      }
      performTurn(isClockwise);
      isWaiting = true;
      waitStartTime = millis();
    }
  }
}