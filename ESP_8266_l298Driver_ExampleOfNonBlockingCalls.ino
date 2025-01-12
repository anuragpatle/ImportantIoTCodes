#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>  // Include the Servo library

// Define motor pins
#define MOTOR1_PIN1 5        // GPIO5 D1
#define MOTOR1_PIN2 4        // GPIO4
#define MOTOR1_ENABLE_PIN 0  // GPIO0

Servo myServo1;  // Create a Servo object for the first servo
Servo myServo2;  // Create a Servo object for the second servo

// Variables for non-blocking servo control
const long servo1Interval = 15;  // Interval for first servo movement in milliseconds
const long servo2Interval = 20;  // Interval for second servo movement in milliseconds

// Variables for non-blocking motor control
const long motorInterval = 15000;           // Interval for motor running time in milliseconds
const long stopInterval = 10000;            // Interval for motor stopping time in milliseconds

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);

  // Initialize motor control pins as outputs
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR1_ENABLE_PIN, OUTPUT);

  // Attach the servos to GPIO pins
  myServo1.attach(2);  // Attach the first servo to GPIO2 (D4)
  myServo2.attach(14); // Attach the second servo to GPIO14 (D5)

  // Confirm initialization
  Serial.println("Setup completed");
}

void loop() {
  // Run the servos back and forth in a non-blocking manner
  runServo1NonBlocking();
  runServo2NonBlocking();

  // Run the motor functions in a non-blocking manner
  handleMotorNonBlocking();
}

void moveForward() {
  // Set the motor speed to 25% and set the direction to forward
  analogWrite(MOTOR1_ENABLE_PIN, 255);  // Set speed to 25%
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, HIGH);
}

void stopMotor() {
  // Stop the motor by setting speed to 0% and turning off both direction pins
  analogWrite(MOTOR1_ENABLE_PIN, 0);  // Set speed to 0%
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
}

void runServo1NonBlocking() {
  // Static variables to preserve state between function calls
  static unsigned long previousServo1Millis = 0;  // Stores the last time the first servo was moved
  static int servo1Pos = 0;                       // Current position of the first servo
  static bool servo1Forward = true;               // Direction of the first servo movement

  unsigned long currentMillis = millis();  // Get the current time

  // Handle millis() overflow
  if (currentMillis < previousServo1Millis) {
    previousServo1Millis = currentMillis;
  }

  // Check if it's time to move the first servo
  if (currentMillis - previousServo1Millis >= servo1Interval) {
    previousServo1Millis = currentMillis;  // Save the current time

    // Move the first servo in the forward direction
    if (servo1Forward) {
      servo1Pos += 1;  // Increment the first servo position
      if (servo1Pos >= 120) {
        servo1Forward = false;  // Change direction at the limit
      }
    } else {  // Move the first servo in the backward direction
      servo1Pos -= 1;  // Decrement the first servo position
      if (servo1Pos <= 0) {
        servo1Forward = true;  // Change direction at the limit
      }
    }
    myServo1.write(servo1Pos);  // Write the new position to the first servo

    // Debugging: Print the current first servo position and direction
    Serial.print("Servo1 position: ");
    Serial.print(servo1Pos);
    Serial.print(" Direction: ");
    Serial.println(servo1Forward ? "Forward" : "Backward");
  }
}

void runServo2NonBlocking() {
  // Static variables to preserve state between function calls
  static unsigned long previousServo2Millis = 0;  // Stores the last time the second servo was moved
  static int servo2Pos = 0;                       // Current position of the second servo
  static bool servo2Forward = true;               // Direction of the second servo movement

  unsigned long currentMillis = millis();  // Get the current time

  // Handle millis() overflow
  if (currentMillis < previousServo2Millis) {
    previousServo2Millis = currentMillis;
  }

  // Check if it's time to move the second servo
  if (currentMillis - previousServo2Millis >= servo2Interval) {
    previousServo2Millis = currentMillis;  // Save the current time

    // Move the second servo in the forward direction
    if (servo2Forward) {
      servo2Pos += 2;  // Increment the second servo position by 2 for different speed
      if (servo2Pos >= 90) {
        servo2Forward = false;  // Change direction at the limit
      }
    } else {  // Move the second servo in the backward direction
      servo2Pos -= 2;  // Decrement the second servo position by 2 for different speed
      if (servo2Pos <= 0) {
        servo2Forward = true;  // Change direction at the limit
      }
    }
    myServo2.write(servo2Pos);  // Write the new position to the second servo

    // Debugging: Print the current second servo position and direction
    Serial.print("Servo2 position: ");
    Serial.print(servo2Pos);
    Serial.print(" Direction: ");
    Serial.println(servo2Forward ? "Forward" : "Backward");
  }
}

void handleMotorNonBlocking() {
  // Static variables to preserve state between function calls
  static unsigned long previousMotorMillis = 0;  // Stores the last time the motor state was changed
  static bool motorRunning = false;              // Flag to indicate if the motor is running
  static bool motorStopped = false;              // Flag to indicate if the motor is stopped

  unsigned long currentMillis = millis();  // Get the current time

  // Handle millis() overflow
  if (currentMillis < previousMotorMillis) {
    previousMotorMillis = currentMillis;
  }

  // Start the motor if it's not running and not stopped
  if (!motorRunning && !motorStopped) {
    moveForward();                          // Move the motor forward
    previousMotorMillis = currentMillis;    // Save the current time
    motorRunning = true;                    // Set the motor running flag

    // Debugging: Print motor running status
    Serial.println("Motor running");
  }

  // Stop the motor if the running interval has passed
  if (motorRunning && (currentMillis - previousMotorMillis >= motorInterval)) {
    stopMotor();                            // Stop the motor
    previousMotorMillis = currentMillis;    // Save the current time
    motorRunning = false;                   // Clear the motor running flag
    motorStopped = true;                    // Set the motor stopped flag

    // Debugging: Print motor stopped status
    Serial.println("Motor stopped");
  }

  // Reset the motor stopped flag if the stop interval has passed
  if (motorStopped && (currentMillis - previousMotorMillis >= stopInterval)) {
    motorStopped = false;                   // Clear the motor stopped flag

    // Debugging: Print motor ready to run again status
    Serial.println("Motor ready to run again");
  }
}
