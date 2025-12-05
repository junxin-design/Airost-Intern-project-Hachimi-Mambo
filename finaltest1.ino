#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// --- PIN DEFINITIONS ---
// Right Motors
const int IN1 = 27;
const int IN2 = 26;
const int ENA = 23; // PWM Speed Control

// Left Motors
const int IN3 = 12;
const int IN4 = 14;
const int ENB = 13; // PWM Speed Control

// Sensors
const int SENSOR_LEFT = 32;
const int SENSOR_MID = 33;
const int SENSOR_RIGHT = 35;

// --- SETTINGS ---
int manualSpeed = 200;    // Speed for remote control (0-255)
int followSpeed = 160;    // Speed for line following (usually slower)
String mode = "MANUAL";   // Starts in Manual mode

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot_Car"); // Bluetooth Name

  // Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensor Pins
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MID, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  Serial.println("Robot Started. Connect Bluetooth now.");
}

void loop() {
  // Check for Bluetooth Commands
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    handleBluetooth(cmd);
  }

  // Execute Logic based on Mode
  if (mode == "AUTO") {
    lineFollowLogic();
  } 
  // If MANUAL, the handleBluetooth function handles movement directly
}

// --- LINE FOLLOWING LOGIC ---
void lineFollowLogic() {
  // Read Sensors (Assuming 1 = Black Line, 0 = White Surface)
  // NOTE: Adjust '!' if your sensors use 0 for Black.
  int left = digitalRead(SENSOR_LEFT);
  int mid = digitalRead(SENSOR_MID);
  int right = digitalRead(SENSOR_RIGHT);

  if (mid == 1 && left == 0 && right == 0) {
    // Center on line -> Forward
    moveForward(followSpeed);
  } 
  else if (left == 1) {
    // Drifting Right -> Turn Left
    turnLeft(followSpeed);
  } 
  else if (right == 1) {
    // Drifting Left -> Turn Right
    turnRight(followSpeed);
  } 
  else if (left == 0 && mid == 0 && right == 0) {
    // Line lost -> Stop or inch forward (Stop is safer)
    stopMotors();
  }
}

// --- BLUETOOTH HANDLER ---
void handleBluetooth(char cmd) {
  // Mode Switching
  if (cmd == 'X') {
    mode = "AUTO";
    SerialBT.println("Mode: Line Follow");
  } 
  else if (cmd == 'x') {
    mode = "MANUAL";
    stopMotors();
    SerialBT.println("Mode: Manual");
  }

  // Manual Controls (Only work if mode is MANUAL)
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
  // Right Motor Fwd
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  // Left Motor Fwd
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
  // Right Fwd, Left Back (or Stop) for tight turn
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); // Left moves backward for spin turn
  analogWrite(ENB, speed);
}

void turnRight(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH
  ); // Right moves backward
  analogWrite(ENA, speed);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
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