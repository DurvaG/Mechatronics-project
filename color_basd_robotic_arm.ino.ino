#include <Servo.h>

// Pin definitions
const int ldrPin = A0; // LDR analog input pin
const int redPin = 9;  // RGB LED Red
const int greenPin = 10; // RGB LED Green
const int bluePin = 11; // RGB LED Blue

// Servo motor pins
const int servoPin1 = 5;
const int servoPin2 = 6;

Servo servo1;
Servo servo2;

// Thresholds for color detection (adjust based on your setup)
int redThreshold = 700;
int greenThreshold = 600;
int blueThreshold = 500;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Setup RGB LED pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Attach servos
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  
  // Set initial servo positions
  servo1.write(90); // Neutral position for base rotation
  servo2.write(90); // Neutral position for arm movement
}

void loop() {
  int ldrValue;

  // Read and detect RED color
  setRGB(255, 0, 0); // Turn on red LED
  delay(500);        // Give some time for color detection
  ldrValue = analogRead(ldrPin);
  Serial.print("Red LDR Value: ");
  Serial.println(ldrValue);
  if (ldrValue > redThreshold) {
    moveArm("red");
    return;
  }

  // Read and detect GREEN color
  setRGB(0, 255, 0); // Turn on green LED
  delay(500);        // Give some time for color detection
  ldrValue = analogRead(ldrPin);
  Serial.print("Green LDR Value: ");
  Serial.println(ldrValue);
  if (ldrValue > greenThreshold) {
    moveArm("green");
    return;
  }

  // Read and detect BLUE color
  setRGB(0, 0, 255); // Turn on blue LED
  delay(500);        // Give some time for color detection
  ldrValue = analogRead(ldrPin);
  Serial.print("Blue LDR Value: ");
  Serial.println(ldrValue);
  if (ldrValue > blueThreshold) {
    moveArm("blue");
    return;
  }

  // Turn off the RGB LED after scanning
  setRGB(0, 0, 0);
  delay(1000);
}

// Function to set RGB LED color
void setRGB(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

// Function to move the arm based on detected color
void moveArm(String color) {
  if (color == "red") {
    // Move to the red object's position
    servo1.write(45);  // Rotate base to specific position
    servo2.write(120); // Lower arm for red object
  }
  else if (color == "green") {
    // Move to the green object's position
    servo1.write(90);  // Rotate base to specific position
    servo2.write(120); // Lower arm for green object
  }
  else if (color == "blue") {
    // Move to the blue object's position
    servo1.write(135); // Rotate base to specific position
    servo2.write(120); // Lower arm for blue object
  }

  // Pause for a while to mimic object manipulation
  delay(2000);
  
  // Reset arm to initial position
  servo1.write(90); // Return base to neutral position
  servo2.write(90); // Lift arm back
}
