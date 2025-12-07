#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// --- PIN DEFINITIONS ---
// Left Motor (Motor A)
const int IN1 = 27;
const int IN2 = 26;
const int ENA = 23;

// Right Motor (Motor B)
const int IN3 = 12;
const int IN4 = 14;
const int ENB = 13;

// Sensors
const int SENSOR_LEFT = 32;
const int SENSOR_MID = 33;
const int SENSOR_RIGHT = 35;

// --- SETTINGS ---
int manualSpeed = 200;
int followSpeed = 160;
String mode = "MANUAL";

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot_Car");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MID, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  Serial.println("Robot Started. Connect Bluetooth now.");
}

void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    handleBluetooth(cmd);
  }

  if (mode == "AUTO") {
    lineFollowLogic();
  }
}

// --- LINE FOLLOWING LOGIC ---
void lineFollowLogic() {
  int left = digitalRead(SENSOR_LEFT);
  int mid = digitalRead(SENSOR_MID);
  int right = digitalRead(SENSOR_RIGHT);

  if (mid == 1 && left == 0 && right == 0) {
    moveForward(followSpeed);
  }
  else if (left == 1) {
    turnLeft(followSpeed);
  }
  else if (right == 1) {
    turnRight(followSpeed);
  }
  else {
    stopMotors(); 
  }
}

// --- BLUETOOTH HANDLER ---
void handleBluetooth(char cmd) {
  if (cmd == 'X') {
    mode = "AUTO";
    SerialBT.println("Mode: Line Follow");
  }
  else if (cmd == 'x') {
    mode = "MANUAL";
    stopMotors();
    SerialBT.println("Mode: Manual");
  }

  if (mode == "MANUAL") {
    switch (cmd) {
      case 'F': moveForward(manualSpeed); break;
      case 'B': moveBackward(manualSpeed); break;
      case 'L': turnLeft(manualSpeed); break;
      case 'R': turnRight(manualSpeed); break;
      case 'S': stopMotors(); break;
    }
  }
}

// --- MOTOR FUNCTIONS ---
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void turnLeft(int speed) {
  // Right motor forward, left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void turnRight(int speed) {
  // Left motor forward, right motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}
