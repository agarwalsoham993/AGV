// Define the pins for the motor inputs
const int L_input1 = 5; // Left motor input 1
const int L_input2 = 4; // Left motor input 2
const int R_input1 = 2; // Right motor input 1
const int R_input2 = 3; // Right motor input 2

// Define the pins for the ultrasonic sensors
const int trig_r = 11; // Right ultrasonic sensor trigger pin
const int echo_r = 10; // Right ultrasonic sensor echo pin
const int trig_l = 13; // Left ultrasonic sensor trigger pin
const int echo_l = 12; // Left ultrasonic sensor echo pin

const int setpoint = 10; // in cm

// Function to stop the robot
void Stop() {
  // Set all motor inputs to LOW to stop the motors
  digitalWrite(L_input1, LOW);
  digitalWrite(L_input2, LOW);
  digitalWrite(R_input1, LOW);
  digitalWrite(R_input2, LOW);
}

// Function to move the robot forward
void Forward() {
  // Set the left and right motor inputs to move forward
  digitalWrite(L_input1, HIGH);
  digitalWrite(L_input2, LOW);
  digitalWrite(R_input1, HIGH);
  digitalWrite(R_input2, LOW);
}

// Function to turn the robot left
void Left() {
  // Set the left motor to move forward and the right motor to stop
  digitalWrite(L_input1, HIGH);
  digitalWrite(L_input2, LOW);
  digitalWrite(R_input1, LOW);
  digitalWrite(R_input2, LOW);
}

// Function to turn the robot right
void Right() {
// Set the right motor to move forward and the left motor to stop
  digitalWrite(L_input1, LOW);
  digitalWrite(L_input2, LOW);
  digitalWrite(R_input1, HIGH);
  digitalWrite(R_input2, LOW);
}

void setup() {
  // Set the motor pins as outputs
  pinMode(L_input1, OUTPUT);
  pinMode(L_input2, OUTPUT);
  pinMode(R_input1, OUTPUT);
  pinMode(R_input2, OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  
  // Set the ultrasonic sensor pins as inputs and outputs
  pinMode(trig_r, OUTPUT);
  pinMode(echo_r, INPUT);
  pinMode(trig_l, OUTPUT);
  pinMode(echo_l, INPUT);
  
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
}

void loop() {
  // Read the distances from the left and right ultrasonic sensors
  analogWrite(8,255);
  analogWrite(9,255);
  digitalWrite(trig_l, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_l, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_l, LOW);
  
  long duration = pulseIn(echo_l, HIGH);
  int distance_l = duration * 0.034 / 2;
  
  digitalWrite(trig_r, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_r, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_r, LOW);
  
  duration = pulseIn(echo_r, HIGH);
  int distance_r = duration * 0.034 / 2;
  
  // Print the distances to the serial monitor for debugging
  Serial.print("Left Distance: ");
  Serial.println(distance_l);
  Serial.print("Right Distance: ");
  Serial.println(distance_r);
  
  // Navigation logic based on sensor readings
  if (distance_l < setpoint || distance_r < setpoint) {
    // If an obstacle is detected on either side, stop the robot
    Stop();
  } else if (distance_l > distance_r) {
    // If the left side has more clearance, turn right
    Right();
  } else if (distance_r > distance_l) {
    // If the right side has more clearance, turn left
    Left();
  } else {
    // If both sides have equal clearance, move forward
    Forward();
  }
}
