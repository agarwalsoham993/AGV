const int r_inp1 = 2;  // input of motor driver
const int r_inp2 = 3;
const int l_inp1 = 5;
const int l_inp2 = 4;

const int en_r = 9;  // Enable pin for right motor
const int en_l = 6;   // Enable pin for left motor

const int trig_r = 11;    // right trig
const int echor = 10;   // right echo
const int trig_l = 13;   // left trig
const int echol = 12;   // left echo

int distance_r;
int distance_l;

long duration;

// PID constants
const float Kp = 1;   // proportional gain
const float Ki = 0.01;   // integ gain
const float Kd = 0.01;  // Derivative gain


float error_r = 0;
float prev_error_r = 0;
float error_l = 0;
float prev_error_l = 0;
float prev_distance_r = 0;
float prev_distance_l = 0;

float integ_r = 0;
float integ_l= 0;

//time variables
double prev_time = 0;
double dt = 0;

const float noise = 1; // in cm
const float setpoint = 12.0; //setpoint distance for the robot to maintain(in cm)

//functions 

void setup() {
  pinMode(l_inp1, OUTPUT);
  pinMode(l_inp2, OUTPUT);

  pinMode(r_inp1, OUTPUT);
  pinMode(r_inp2, OUTPUT);

  pinMode(en_r, OUTPUT);
  pinMode(en_l, OUTPUT);

  pinMode(trig_r, OUTPUT);
  pinMode(echor, INPUT);

  pinMode(trig_l, OUTPUT);
  pinMode(echol, INPUT);

  Serial.begin(9600);
}

void loop() {

  double time = millis();
  dt = (time - prev_time)/1000.00;
  prev_time = time;
  
  // Read ultrasonic sensor values
  digitalWrite(trig_r, LOW);
  delayMicroseconds(10);
  digitalWrite(trig_r, HIGH);
  delayMicroseconds(50);
  digitalWrite(trig_r, LOW);

  digitalWrite(echor, LOW);
  duration = pulseIn(echor, HIGH);

  distance_r = duration * 0.034 / 2;
  distance_r = (distance_r + prev_distance_r)/2;
  prev_distance_r = distance_r;

  digitalWrite(trig_l, LOW);
  delayMicroseconds(10);
  digitalWrite(trig_l, HIGH);
  delayMicroseconds(50);
  digitalWrite(trig_l, LOW);

  digitalWrite(echol, LOW);
  duration = pulseIn(echol, HIGH);
  
  distance_l = duration * 0.034 / 2;
  distance_l = (distance_l + prev_distance_l)/2;
  prev_distance_l = distance_l;

  
  // Calculate error term
  error_r = distance_r - setpoint;
  error_l = distance_l - setpoint;
  
  // deadband to the error
  if (error_l < (-noise)) {
      error_l = error_l + noise  ; // Error is negative (too close)
    } else if (error_l > (noise)) {
      error_l = error_l - noise ; // Error is positive (too far)
    } else {
      error_l = 0; // Sensor is within range, but another sensor isn't.
    }
  
  if (error_r < (-noise)) {
      error_r = error_r + noise ; // Error BROADBANDING
    } else if (error_r > (noise)) {
      error_r = error_r - noise ; 
    } else {
      error_r = 0; 
    }

  Serial.print("Right distance: ");
  Serial.println(distance_r);
  Serial.print("Left Distance: ");
  Serial.println(distance_l);

  Serial.print("error_r: ");
  Serial.println(error_r);
  Serial.print("error_l: ");
  Serial.println(error_l);

    if (error_r ==0 && error_l == 0){
        //stop
        digitalWrite(l_inp1, LOW);
        digitalWrite(l_inp2, LOW);
        digitalWrite(r_inp1, LOW);
        digitalWrite(r_inp2, LOW);
    }
    else{
        // PID calculation
        float prop_r = Kp * error_r;
        integ_r += Ki * (error_r * dt);
        float derivative_r = Kd * ((error_r - prev_error_r))/dt; 
        prev_error_r = error_r;
        float pidOutput_r = prop_r + integ_r + derivative_r;

        // Limit PID output to prevent saturation
        pidOutput_r = constrain(pidOutput_r, -255, 255);

        Serial.print("right PID Output : ");
        Serial.println(pidOutput_r);

        float prop_l = Kp * error_l;
        integ_l += Ki * (error_l * dt);
        float derivative_l = Kd * ((error_l - prev_error_l)/dt);
        prev_error_l = error_l;
        float pidOutput_l = prop_l + integ_l + derivative_l;

        // Limit PID output to prevent saturation
        pidOutput_l = constrain(pidOutput_l, -255, 255);

        Serial.print("left PID Output : ");
        Serial.println(pidOutput_l);
      
        // Adjust motor speeds based on PID output
        if (pidOutput_r > 0) {
            //right motor forward
            analogWrite(en_r, pidOutput_r);
            digitalWrite(r_inp1, HIGH);
            digitalWrite(r_inp2, LOW);
        }
        else {
            //right motor backward
            analogWrite(en_r, -(pidOutput_r));
            digitalWrite(r_inp1, LOW);
            digitalWrite(r_inp2, HIGH);
        }
        if(pidOutput_l > 0){
            //left motor forward
            analogWrite(en_l,pidOutput_l);
            digitalWrite(l_inp1, HIGH);
            digitalWrite(l_inp2, LOW);
        }
        else {
            //left motor backward
            analogWrite(en_l, -pidOutput_l);
            digitalWrite(l_inp1, LOW);
            digitalWrite(l_inp2, HIGH);
        }
    }
    delay(100);
}
