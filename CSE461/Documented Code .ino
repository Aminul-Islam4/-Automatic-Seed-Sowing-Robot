#include <Servo.h>
#include <NewPing.h> // Include the NewPing library for ultrasonic sensor

#include <Arduino.h> // Include Arduino library for Serial communication

// Define pins for the rotary encoder
#define CLK_PIN 2
#define DT_PIN 3

#define BOTTOM_IR_SENSOR_RIGHT A0
#define BOTTOM_IR_SENSOR_LEFT A1

#define TRIGGER_PIN 11 // Define ultrasonic sensor trigger pin
#define ECHO_PIN 12    // Define ultrasonic sensor echo pin
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

// Right motor
int enableRightMotor = 5;
int rightMotorPin1 = 7;
int rightMotorPin2 = 8;

// Left motor
int enableLeftMotor = 6;
int leftMotorPin1 = 9;
int leftMotorPin2 = 10;

int relayPin = 13; // Pin for relay

volatile int encoderPos = 0;
int lastEncoderPos = 0; // Variable to store last encoder position
float wheelCircumference = 400.0; // in centimeters, adjust according to your wheel size

Servo myservo; // create servo object to control a servo

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup()
{
  // Initialize Serial communication
  Serial.begin(9600);

  // Set up rotary encoder pins as inputs
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), updateEncoder, CHANGE);

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(BOTTOM_IR_SENSOR_RIGHT, INPUT);
  pinMode(BOTTOM_IR_SENSOR_LEFT, INPUT);

  pinMode(relayPin, OUTPUT); // Set relay pin as output 

  myservo.attach(4); // attaches the servo on pin 4 to the servo object


}


void loop()
{
  // Check if there's a change in encoder position
  if (encoderPos != lastEncoderPos)
  {
    // Calculate distance traveled
    float distance = encoderPos * (wheelCircumference / 360.0);

    // Display distance in the Serial Monitor
    Serial.print("Encoder Count: ");
    Serial.print(encoderPos);
    Serial.print("\tDistance: ");
    Serial.print(distance);
    Serial.println(" cm");

    // Check if the distance difference is within 65 to 68 cm
    if (distance >= 65 && distance <= 68) {
      // Stop the motor for 5 seconds
      rotateMotor(0, 0);
      delay(5000); // 5 seconds delay

      // Operate servo motor twice quickly
      for (int i = 0; i < 2; i++) {
        myservo.write(90); // Move servo to 90 degrees position
        delay(500); // Wait for 0.5 seconds
        myservo.write(0); // Move servo to 0 degrees position
        delay(500); // Wait for 0.5 seconds
      }

      digitalWrite(relayPin, LOW); // Activate relay
      delay(2000); // Keep relay active for 2 seconds
      digitalWrite(relayPin, HIGH); // Deactivate relay


      // Reset encoder count
      encoderPos = 0;

      // Resume line following
      lineFollowerControl();
    }

    // Update last encoder position
    lastEncoderPos = encoderPos;
  }

  int obstacleDistance = checkObstacle(); // Check for obstacles

  if (obstacleDistance > 0 && obstacleDistance < 20) // If obstacle detected within 20cm
  {
    // Stop and turn away from the obstacle
    rotateMotor(0, 0); // Stop
  }
  else
  {
    // No obstacle detected, continue line following
    lineFollowerControl();
  }
}

void lineFollowerControl()
{
  static int MOTOR_SPEED = 150;
  static int TURNING_MOTOR_SPEED = 200;

  int rightIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(BOTTOM_IR_SENSOR_LEFT);

  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW)
  {
    rotateMotor(-TURNING_MOTOR_SPEED, TURNING_MOTOR_SPEED);
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH)
  {
    rotateMotor(TURNING_MOTOR_SPEED, -TURNING_MOTOR_SPEED);
  }
  else
  {
    rotateMotor(0, 0);
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else if (rightMotorSpeed >= 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  else if (leftMotorSpeed >= 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}

int checkObstacle()
{
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int distance = sonar.ping_cm(); // Send ping, get distance in centimeters.
  return distance;
}

// Interrupt service routine to update encoder position
void updateEncoder()
{
  static int lastState = 0;
  int currentState = (digitalRead(CLK_PIN) << 1) | digitalRead(DT_PIN);
  int stateChange = (lastState << 2) | currentState;

  if (stateChange == 1 || stateChange == 50 || stateChange == 70 || stateChange == 100)
  {
    encoderPos--;
  }
  else if (stateChange == 2 || stateChange == 50 || stateChange == 70 || stateChange == 100)
  {
    encoderPos++;
  }

  lastState = currentState & 0b11;
}
