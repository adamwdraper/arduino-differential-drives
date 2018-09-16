#include <NewPing.h>

// Pin assignments
#define BUTTON 2
#define AIN1 3
#define AIN2 4
#define APWM 5
#define STBY 6
#define ECHO_PIN 7   // Arduino pin tied to echo pin on the ultrasonic sensor.
#define LED_GREEN 8
#define LED_RED 9
#define TRIGGER_PIN  10  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define BPWM 11
#define BIN1 12
#define BIN2 13

// Constants for motor control functions
#define STEPTIME 600 
#define STRAIGHTSPEED 200
#define TURNSPEED 120
#define TURNTIME 300
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

// Boolean (True or False) variable to control whether or not
// the motors move
bool obstacleDetected = false;
int isOn = 0;  // the current reading from the input pin

// Debounce button presses
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 500;    // the debounce time; increase if the output flickers

// Array to track current PWM values for each channel (A and B)
int pwms[] = {APWM, BPWM};

// Offsets to be used to compensate for one motor being more powerful
byte leftOffset = 0;
byte rightOffset = 0;

// Variable to track remaining time
unsigned long pwmTimeout = 0;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Function to write out pwm values
void writePwms ( int left, int right) {
  analogWrite (pwms[0], left);
  analogWrite (pwms[1], right);
}

// Move the robot forward for STEPTIME
void goForward() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  writePwms (STRAIGHTSPEED-leftOffset,STRAIGHTSPEED-rightOffset);
  pwmTimeout = millis() + STEPTIME;
}

// Move the robot backward for STEPTIME
void goBack() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  writePwms (STRAIGHTSPEED-leftOffset,STRAIGHTSPEED-rightOffset);
  pwmTimeout = millis() + STEPTIME;
}

// Turn the robot left for TURNTIME
void goLeft() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  writePwms (TURNSPEED,TURNSPEED);
  pwmTimeout = millis() + TURNTIME;
}

// Turn the robot right for TURNTIME
void goRight () {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  writePwms (TURNSPEED,TURNSPEED);
  pwmTimeout = millis() + TURNTIME;
}

// Stop the robot (using standby)
void stop() {
  digitalWrite(STBY, LOW); 
}

// LED functions
void ledOn(int pin) {
  digitalWrite(pin, HIGH);
}

void toggleLeds() {
  if (isOn) {
    if (obstacleDetected) {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);
    } else {
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_RED, LOW);
    }
  } else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
  }
}

// Arduino setup function
void setup() {
  Serial.begin(9600);
  
  // Initialize pins as outputs
  pinMode (STBY, OUTPUT);
  pinMode (AIN1, OUTPUT);
  pinMode (AIN2, OUTPUT);
  pinMode (APWM, OUTPUT);
  pinMode (BIN1, OUTPUT);
  pinMode (BIN2, OUTPUT);
  pinMode (BPWM, OUTPUT);

  // LED
  pinMode (LED_GREEN, OUTPUT);
  pinMode (LED_RED, OUTPUT);

  // Define pin #12 as input and activate the internal pull-up resistor
  pinMode(BUTTON, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUTTON), startStop, FALLING);
}

// Loop (code betwen {}'s repeats over and over again)
void loop() {
  delay(50);

  Serial.println(isOn);
  Serial.println(obstacleDetected);
  
  if (isOn) {
    // Trigger ultrasonic ping
    int distance = sonar.ping_cm();
  
    // If there is an object within 10cm of the sensor
    // NOTE: The NewPing library returns 0 if no object is detected.
    if (distance <= 25 && distance != 0){
      obstacleDetected = true;
    } else {
      obstacleDetected = false;
    }

    toggleLeds();

    if (obstacleDetected) {
      goRight();

      delay(750);
    } else {
      goForward();
    }
  } else {
    toggleLeds();
        
    stop();
  }
}

void startStop() {
  // If the switch changed, due to noise or pressing:
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // reset the debouncing timer
    lastDebounceTime = millis();

    isOn = !isOn;
  }
}
