// --- Constants ---
#define OBSTACLE_DIST 40
#define MAX_SENSOR_DIST 60
#define FRONT_OBSTACLE_THRESHOLD 7 // Threshold for front obstacle

// Initial headstart
const int LAUNCH_SPEED = 255;
const int LAUNCH_DURATION = 1000; //duration in ms

// --- PID Tuning ---
int BASE_SPEED = 250;
float Kp = 4, Kd = 5 , Ki = 0;
int P, D, I, previousError, PIDvalue;

// --- Pinout ---
// D5, PD4 for Motor Direction
// D6, D3 for Motor PWM
// D12 for Ultrasonic Trig
// D11, D10, D9 for Ultrasonic Echo

// --- Setup ---
void setup() {
  Serial.begin(9600);

  // Motor/Sensor Pin Setup
  DDRD |= (1 << PD5) | (1 << PD4) | (1 << PD6) | (1 << PD3); // Motor Pins
  DDRB |= (1 << PB4); // Ultrasonic Trig
  DDRB &= ~((1 << PB3) | (1 << PB2) | (1 << PB1)); // Ultrasonic Echos

  // Timer Setup for PWM
  TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); TCCR0B |= (1 << CS01);
  TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); TCCR2B |= (1 << CS21);

  Serial.println("Setup Complete (Wall Follower & Obstacle Avoidance). Starting...");
  delay(5000);
  
  Serial.println("Launch!");
  setMotorSpeeds(LAUNCH_SPEED, LAUNCH_SPEED);
  delay(LAUNCH_DURATION);
}


// --- Main Loop ---
void loop() {
  // --- 1. Read Sensors and Check for Obstacles ---
  int d_center = getDistance(PB4, PB3);
  int d_left = getDistance(PB4, PB2);   
  int d_right = getDistance(PB4, PB1);  

  // --- Obstacle Avoidance Logic ---
  // If an obstacle is detected head-on, override PID and evade.
  if (d_center > 0 && d_center < FRONT_OBSTACLE_THRESHOLD) {
    Serial.println("!!! FRONT OBSTACLE DETECTED - EVADING !!!");
    stopMotors();
    delay(100);

    // Check left and right for the best escape route
    if (d_left > d_right) {
      Serial.println("More space on the left. Turning left.");
      turnLeft(200);
      delay(340);   // ** TUNE THIS DURATION for a 90-degree turn **
    } else {
      Serial.println("More space on the right. Turning right.");
      turnRight(200);
      delay(340);   // ** TUNE THIS DURATION for a 90-degree turn **
    }
    stopMotors();
    delay(100);
    previousError = 0; // Reset PID to prevent lurching
    I = 0;
    return; // Restart the loop to re-evaluate the new position
  }

  // --- 2. Normal PID Steering Logic ---
  // This part only runs if no obstacle was found.
  int leftDist = (d_left == -1) ? MAX_SENSOR_DIST : constrain(d_left, 0, MAX_SENSOR_DIST);
  int rightDist = (d_right == -1) ? MAX_SENSOR_DIST : constrain(d_right, 0, MAX_SENSOR_DIST);
  
  int error = leftDist - rightDist;
  P = error; D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;
  int leftSpeed = constrain(BASE_SPEED - PIDvalue, 0, 255);
  int rightSpeed = constrain(BASE_SPEED + PIDvalue, 0, 255);
  setMotorSpeeds(leftSpeed, rightSpeed);
}


// --- Ultrasonic Distance ---
int getDistance(uint8_t trigPin, uint8_t echoPin) {
  PORTB &= ~(1 << trigPin); delayMicroseconds(2);
  PORTB |= (1 << trigPin); delayMicroseconds(10);
  PORTB &= ~(1 << trigPin);
  long duration = pulseIn(echoPin + 8, HIGH, 30000);
  if (duration == 0) return -1;
  return duration / 58;
}

// --- Motor Control ---
void setMotorSpeeds(uint8_t leftSpeed, uint8_t rightSpeed) {
  PORTD |= (1 << PD5) | (1 << PD4);
  OCR0A = leftSpeed; OCR2B = rightSpeed;
}

void stopMotors() { OCR0A = 0; OCR2B = 0; }
void turnLeft(uint8_t speed) {
  PORTD &= ~(1 << PD5); PORTD |= (1 << PD4);
  OCR0A = speed; OCR2B = speed;
}
void turnRight(uint8_t speed) {
  PORTD |= (1 << PD5); PORTD &= ~(1 << PD4);
  OCR0A = speed; OCR2B = speed;
}