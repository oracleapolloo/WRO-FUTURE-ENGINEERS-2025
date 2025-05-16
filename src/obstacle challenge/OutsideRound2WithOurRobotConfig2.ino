#include <Servo.h>
#include <Pixy2.h>
#include <Wire.h>
#include <MPU6050.h>

// === Pin Configuration ===
#define SERVO_PIN 7
#define SERVO_CENTER 90
#define SERVO_MAX_LEFT 150
#define SERVO_MAX_RIGHT 40

#define TRIG_PIN_LEFT A3
#define ECHO_PIN_LEFT A2
#define TRIG_PIN_RIGHT A1
#define ECHO_PIN_RIGHT A0

#define MOTOR_ENB 3
#define MOTOR_IN3 4
#define MOTOR_IN4 5

#define BUTTON_PIN 2

// === Objects ===
Servo servo_9;
Pixy2 pixy;
MPU6050 gyro;

// === Variables ===
float leftdist = 0, rightdist = 0;
float currentAngle = 0, straightAngle = 0, offsetZ = 0;
int clockwise = 0, anticlockwise = 0;
bool green1 = false, red1 = false;
bool isRunning = false;
int turnCount = 0;
int colorX = 0, blockHeight = 0;
unsigned long previousTime = 0;
unsigned long previousButtonMillis = 0;
const unsigned long debounceInterval = 50;

// === Function Prototypes ===
long readUltrasonicDistance(int triggerPin, int echoPin);
void Forward();
void Stop();
void MeasureDistance();
void calibrateGyro();
void updateGyroAngle();
void rotate90Degrees();
void goStraight();
void findClosestBlock();
void avoidBlock();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  gyro.initialize();
  servo_9.attach(SERVO_PIN);
  pixy.init();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

  if (!gyro.testConnection()) {
    Serial.println("MPU6050 connection failed.");
    while (1);
  }

  servo_9.write(SERVO_CENTER);
  calibrateGyro();
}

void loop() {
  // Button toggle with debouncing
  if (digitalRead(BUTTON_PIN) == LOW) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousButtonMillis >= debounceInterval) {
      previousButtonMillis = currentMillis;
      isRunning = !isRunning;
      if (isRunning) {
        turnCount = 0;
        clockwise = 0;
        anticlockwise = 0;
        Forward();
        Serial.println("Started moving forward");
      } else {
        Stop();
        Serial.println("Stopped");
      }
    }
    while (digitalRead(BUTTON_PIN) == LOW); // Wait for release
  }

  if (isRunning && turnCount < 4) {
    MeasureDistance();

    // Initial path selection
    if (clockwise == 0 && anticlockwise == 0) {
      if (leftdist > 90) {
        anticlockwise = 1;
        Serial.println("Chose anticlockwise");
        rotate90Degrees(); // Left turn
        straightAngle = currentAngle;
      } else if (rightdist > 90) {
        clockwise = 1;
        Serial.println("Chose clockwise");
        rotate90Degrees(); // Right turn
        straightAngle = currentAngle;
      }
    } else {
      // Main navigation
      pixy.ccc.getBlocks();
      findClosestBlock();
      if (green1 || red1) {
        avoidBlock();
      } else {
        goStraight();
      }

      // Check for corner
      if (anticlockwise && leftdist > 90) {
        rotate90Degrees(); // Left turn
        straightAngle = currentAngle;
        turnCount++;
        Serial.print("Turn count: ");
        Serial.println(turnCount);
      } else if (clockwise && rightdist > 90) {
        rotate90Degrees(); // Right turn
        straightAngle = currentAngle;
        turnCount++;
        Serial.print("Turn count: ");
        Serial.println(turnCount);
      }
    }
  }

  if (turnCount >= 4 && isRunning) {
    isRunning = false;
    Stop();
    Serial.println("Completed one round, stopped");
  }
}

long readUltrasonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

void Forward() {
  analogWrite(MOTOR_ENB, 100);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
}

void Stop() {
  analogWrite(MOTOR_ENB, 0);
}

void MeasureDistance() {
  leftdist = 0.01723 * readUltrasonicDistance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
  delay(10);
  rightdist = 0.01723 * readUltrasonicDistance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
  delay(10);
}

void calibrateGyro() {
  int numReadings = 200;
  long totalZ = 0;
  for (int i = 0; i < numReadings; i++) {
    gyro.getRotationZ();
    delay(10);
  }
  for (int i = 0; i < numReadings; i++) {
    int16_t gz = gyro.getRotationZ();
    totalZ += gz;
    delay(10);
  }
  offsetZ = totalZ / (float)numReadings / 131.0;
  Serial.print("Gyroscope Z-axis offset: ");
  Serial.println(offsetZ);
}

void updateGyroAngle() {
  int16_t gz;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  gz = gyro.getRotationZ();
  float rotationZ = (gz / 131.0) - offsetZ;
  currentAngle += rotationZ * deltaTime;
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);
}

void rotate90Degrees() {
  float targetAngle = currentAngle + (anticlockwise ? 90 : -90);
  int turnSpeed = 100; // Reduced PWM for controlled turn
  analogWrite(MOTOR_ENB, turnSpeed);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);

  while (abs(currentAngle - targetAngle) > 2) {
    updateGyroAngle();
    if (anticlockwise) {
      servo_9.write(SERVO_MAX_LEFT);
    } else {
      servo_9.write(SERVO_MAX_RIGHT);
    }
  }

  servo_9.write(SERVO_CENTER);
  analogWrite(MOTOR_ENB, 100); // Restore normal speed
  Serial.println("Completed 90-degree turn");
}

void goStraight() {
  int deadzone = 5;
  if (currentAngle > straightAngle + deadzone) {
    servo_9.write(SERVO_CENTER - 20); // Steer left
  } else if (currentAngle < straightAngle - deadzone) {
    servo_9.write(SERVO_CENTER + 20); // Steer right
  } else {
    servo_9.write(SERVO_CENTER);
  }
  updateGyroAngle();
  Serial.print("Maintaining straight angle: ");
  Serial.println(straightAngle);
}

void findClosestBlock() {
  green1 = false;
  red1 = false;
  colorX = 0;
  blockHeight = 0;
  int j = 0;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_height > blockHeight) {
      blockHeight = pixy.ccc.blocks[i].m_height;
      j = i;
    }
  }

  if (pixy.ccc.numBlocks > 0) {
    if (pixy.ccc.blocks[j].m_signature == 1) {
      green1 = true;
      colorX = pixy.ccc.blocks[j].m_x;
      Serial.println("Green block detected");
    } else if (pixy.ccc.blocks[j].m_signature == 2) {
      red1 = true;
      colorX = pixy.ccc.blocks[j].m_x;
      Serial.println("Red block detected");
    }
  }
}

void avoidBlock() {
  // Steer toward block
  while ((leftdist > 20 && rightdist > 20) && (green1 || red1)) {
    pixy.ccc.getBlocks();
    findClosestBlock();
    if (colorX > 0) {
      if (colorX < 138) {
        servo_9.write(SERVO_MAX_LEFT); // Steer left
      } else if (colorX > 178) {
        servo_9.write(SERVO_MAX_RIGHT); // Steer right
      } else {
        servo_9.write(SERVO_CENTER);
      }
    }
    MeasureDistance();
    updateGyroAngle();
  }

  // Perform color-specific maneuver
  if (green1) {
    servo_9.write(SERVO_MAX_LEFT);
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      updateGyroAngle();
    }
    Serial.println("Green maneuver: Turned left");
  } else if (red1) {
    servo_9.write(SERVO_MAX_RIGHT);
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      updateGyroAngle();
    }
    Serial.println("Red maneuver: Turned right");
  }

  goStraight();
}