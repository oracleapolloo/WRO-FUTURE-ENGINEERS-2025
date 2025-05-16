#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Pixy2.h>

// --- Servo Setup ---
#define SERVO_PIN 7

// --- Motor Setup ---
#define ENB 3  // PWM pin for speed
#define IN3 4  // Motor direction control
#define IN4 5

// --- Ultrasonic Setup ---
#define TRIG_PIN_LEFT  A3
#define ECHO_PIN_LEFT  A2
#define TRIG_PIN_RIGHT A1
#define ECHO_PIN_RIGHT A0
#define MAX_DISTANCE   200  // cm

// --- Objects ---
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);
Servo steeringServo;
MPU6050 mpu;
Pixy2 pixy;

// --- Variables ---
float yawAngle = 0;
unsigned long prevTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // --- Servo Init ---
  steeringServo.attach(SERVO_PIN);
  Serial.println("Servo Test...");
  steeringServo.write(110); delay(1000);
  // steeringServo.write(150); delay(2000);
  // steeringServo.write(90); delay(2000);
  // steeringServo.write(40); delay(2000);
  // steeringServo.write(90); delay(2000);

  // --- Motor Init ---
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Serial.println("Motor Test...");
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  // analogWrite(ENB, 100);
  // delay(1000);
  //  digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  // analogWrite(ENB, 150);
  // delay(1000);
  //  digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);
  // analogWrite(ENB, 255);
  // delay(1000);
  // analogWrite(ENB, 0);

  // --- MPU6050 Init ---
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected.");
  } else {
    Serial.println("MPU6050 connection failed!");
  }
  prevTime = millis();

  // --- Pixy2 Init ---
  Serial.println("Initializing Pixy2...");
  pixy.init();
  Serial.println("Pixy2 ready.");
}

void loop() {
  // --- Ultrasonic ---
  int distanceLeft = sonarLeft.ping_cm();
  int distanceRight = sonarRight.ping_cm();

  Serial.print("Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm | ");

  Serial.print("Right: ");
  Serial.print(distanceRight);
  Serial.print(" cm | ");

  // --- MPU6050 Yaw Calculation ---
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  float gz_deg = gz / 131.0;
  yawAngle += gz_deg * dt;

  Serial.print("Yaw: ");
  Serial.print(yawAngle, 1);
  Serial.print(" deg | ");

  // --- Pixy2 Block Detection ---
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks > 0) {
    int x = pixy.ccc.blocks[0].m_x;
    int width = pixy.ccc.blocks[0].m_width;

    Serial.print("Block X: ");
    Serial.print(x);
    Serial.print(" Width: ");
    Serial.print(width);

    // --- Servo Steering ---
        // --- Smooth Proportional Steering Control ---
    const int PIXY_CENTER = 158;    // Pixy2 X midpoint (315 / 2)
    const int DEAD_ZONE = 10;       // Tolerance band (adjustable)
    const int MAX_SERVO_LEFT = 150; // Max left turn angle
    const int MAX_SERVO_RIGHT = 40; // Max right turn angle

    int error = x - PIXY_CENTER;    // Difference from center

    // Dead zone to avoid small jitters
    if (abs(error) < DEAD_ZONE) {
      error = 0;
    }

    // Map Pixy2 X error to servo angle smoothly
    int servoAngle = map(error, -PIXY_CENTER, PIXY_CENTER, MAX_SERVO_LEFT, MAX_SERVO_RIGHT);

    // Limit servoAngle to safe bounds
    servoAngle = constrain(servoAngle, MAX_SERVO_RIGHT, MAX_SERVO_LEFT);

    steeringServo.write(servoAngle);


    // --- Move Forward ---
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 150);

  } else {
    Serial.print("No Block ");
    // --- Stop Motor ---
    analogWrite(ENB, 0);
  }

  Serial.println();
  delay(100);
}
