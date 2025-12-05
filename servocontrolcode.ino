#include <BluetoothSerial.h>
#include <ESP32Servo.h>

// Initialize Bluetooth Serial object
BluetoothSerial SerialBT;

// Create Servo objects
Servo servo1;
Servo servo2;

// Define the GPIO pins for the servos
const int servo1Pin = 18;
const int servo2Pin = 19;

// Variable to store the incoming angle
int targetAngle = 90; // Start at middle position

void setup() {
  Serial.begin(115200); // For Serial Monitor debugging

  // Start Bluetooth with a name
  SerialBT.begin("ESP32_Dual_Servo"); 
  Serial.println("Bluetooth Device is Ready to Pair");

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach servos to pins
  // Standard SG90 pulse width: 500us to 2400us
  servo1.setPeriodHertz(50); 
  servo1.attach(servo1Pin, 500, 2400);
  
  servo2.setPeriodHertz(50);
  servo2.attach(servo2Pin, 500, 2400);

  // Initialize servos to neutral position
  updateServos(90);
}

void loop() {
  // Check if data is available via Bluetooth
  if (SerialBT.available()) {
    
    // Read the incoming integer
    // We use parseInt() to grab the whole number sent from the phone
    int input = SerialBT.parseInt();

    // Small delay to ensure buffer is cleared usually not needed but good for stability
    delay(10); 

    // Validate if the input is within servo range (0 to 180)
    // We check input > 0 because parseInt returns 0 if it times out or fails
    if (input >= 0 && input <= 180) {
      targetAngle = input;
      
      Serial.print("Received Angle: ");
      Serial.println(targetAngle);
      
      updateServos(targetAngle);
    }
  }
}

// Function to handle the opposite direction logic
void updateServos(int angle) {
  // Servo 1 moves normally
  servo1.write(angle);
  
  // Servo 2 moves in the opposite direction (Inverse)
  // If angle is 0, Servo 2 goes to 180. If angle is 180, Servo 2 goes to 0.
  int inverseAngle = angle;
  servo2.write(inverseAngle);
}