#include <BluetoothSerial.h>
#include <ESP32Servo.h>

BluetoothSerial SerialBT;

// ====================== MOTOR PINS ======================
const int IN1 = 27;
const int IN2 = 26;
const int ENA = 23;

const int IN3 = 12;
const int IN4 = 14;
const int ENB = 13;

// ====================== SENSOR PINS ======================
const int SENSOR_LEFT  = 32;
const int SENSOR_MID   = 33;
const int SENSOR_RIGHT = 35;

// ====================== SERVO PINS ======================
const int servo1Pin = 18;
const int servo2Pin = 19;

Servo servo1;
Servo servo2;

// ====================== SETTINGS ======================
int manualSpeed = 200;
int followSpeed = 160;
String mode = "MANUAL";
int servoAngle = 90;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot_Car");

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensors
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MID, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  // SERVO setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50);
  servo1.attach(servo1Pin, 500, 2400);

  servo2.setPeriodHertz(50);
  servo2.attach(servo2Pin, 500, 2400);

  updateServos(90);

  Serial.println("Robot Ready. Connect via Bluetooth");
}

void loop() {
  if (SerialBT.available()) {
    String msg = SerialBT.readStringUntil('\n');  // read whole input
    msg.trim();

    // If input is a number → it is a servo angle
    if (msg.toInt() > 0 || msg == "0") {
      int angle = msg.toInt();
      if (angle >= 0 && angle <= 180) {
        updateServos(angle);
      }
    }
    else if (msg.length() == 1) {
      char cmd = msg[0];
      handleBluetooth(cmd);
    }
  }

  if (mode == "AUTO") {
    lineFollowLogic();
  }
}

// ====================== SERVO CONTROL ======================
void updateServos(int angle) {
  servoAngle = angle;
  servo1.write(angle);
  servo2.write(angle);
}

// ====================== LINE FOLLOW ======================
void lineFollowLogic() {
  int L = digitalRead(SENSOR_LEFT);
  int M = digitalRead(SENSOR_MID);
  int R = digitalRead(SENSOR_RIGHT);

  if (M == 1 && L == 0 && R == 0) {
    moveForward(followSpeed);
  }
  else if (L == 1) {
    turnLeft(followSpeed);
  }
  else if (R == 1) {
    turnRight(followSpeed);
  }
  else {
    stopMotors();
  }
}

// ====================== BLUETOOTH COMMANDS ======================
void handleBluetooth(char cmd) {
  if (cmd == 'X') {
    mode = "AUTO";
    SerialBT.println("AUTO MODE");
  }
  else if (cmd == 'x') {
    mode = "MANUAL";
    stopMotors();
    SerialBT.println("MANUAL MODE");
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

// ====================== MOTOR FUNCTIONS ======================
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
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void turnRight(int speed) {
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
