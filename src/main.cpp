#include <Arduino.h>

#define LED_14 13
#define LED_2 8
#define LED_LINEFINDER 7

#define RF 5
#define RB 6
#define LF 9
#define LB 10

#define LINE_R A7
#define LINE_L A6
#define LINE_THRESHOLD 130  // edit this to calibrate the line sensors. The brighter the environment, the higher the value

#define SONAR_TRG A3
#define SONAR_ECHO 4
#define SONAR_MAX_WAIT_TIME 7000  // 15000 ms to match the max distance of the sensor, it should be as little as possible to not use up all the processing time.
#define ATTACK_DISTANCE_LIMIT 30  // sonar limit in cm

#define SPEED 0x8F

bool detectLine();
void turnAround(byte);
void detectFoe();
void attack();
void defend();
void move(short, short);
void moveRandomly();
float ping();

unsigned long currentMillisMainTimer = millis();
unsigned long endTimeMainTimer = currentMillisMainTimer;

// Variables for duration and distance
volatile unsigned long pulseDuration;  // Stores pulse duration
float prevDistance = 0;                 // Previous distance reading
unsigned long prevTimeSonar = 0;        // Previous time reading
byte pings = 0; // only after a certain amount of sonar pings the measurement is arrurate

void setup() {
  //Setup: Initalization 
  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);
   
  // Front LED
  pinMode(LED_LINEFINDER, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_14, OUTPUT);

  // Linefinder Sensor pins
  pinMode(LINE_R, INPUT);
  pinMode(LINE_L, INPUT);

  pinMode(SONAR_TRG, OUTPUT); // Set TRIG pin as an output
  pinMode(SONAR_ECHO, INPUT);  // Set ECHO pin as an input with internal pull-up resistor

  // these are just here to show that I tried with interrupts, unfortunately digital read does not work here. (always 0)
  // attachInterrupt(digitalPinToInterrupt(LINE_L), leftLineInterrupt, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(LINE_R), rightLineInterrupt, CHANGE);

  // Attach interrupt to ECHO pin does not work here, as both hardware interrupt supported pins are taken by the wheel sensors...
  // attachInterrupt(digitalPinToInterrupt(SONAR_ECHO), echoInterrupt, CHANGE);

  // create seed for random numbers, taking the analog value of pin 0. 
  randomSeed(analogRead(A0));

  //Then set the start value of the signals to zero: 
  analogWrite(RF, 0);
  analogWrite(RB, 0);
  analogWrite(LF, 0);
  analogWrite(LB, 0);

  digitalWrite(LED_LINEFINDER, HIGH);
  
  Serial.begin(9600); //Enable serial for debugging

  delay(2000);  //Startup delay
  Serial.println("Serial ready");

  move(255,255);
}


// ensure that at no time both forward and backward pins for the same motor is set
// that state could be used for breaking but here that is not necessary
void move(short speedRight, short speedLeft) {
  short absSpeed = (speedLeft >= 0) ? speedLeft : -speedLeft;
  analogWrite((speedLeft >= 0) ? LB : LF, 0);
  analogWrite((speedLeft >= 0) ? LF : LB, absSpeed);

  absSpeed = (speedRight >= 0) ? speedRight : -speedRight;
  analogWrite((speedRight >= 0) ? RB : RF, 0);
  analogWrite((speedRight >= 0) ? RF : RB, absSpeed);
}


bool detectLine() {
  bool rightLine = analogRead(LINE_L) <= LINE_THRESHOLD;
  bool leftLine = analogRead(LINE_R) <= LINE_THRESHOLD;
  
  // Serial.println(analogRead(LINE_L));
  //either fully backup or depending what sensor hits first backup in optimal direction.
  if (rightLine && leftLine) {
    turnAround(0);
    return true;
  }
  else if (rightLine) {
    turnAround(1);
    return true;
  }
  else if (leftLine) {
    turnAround(2);
    return true;
  }

  return false;
}


float ping() {
  // use sonar sensor to find a foe
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(SONAR_TRG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRG, LOW);

  // measure duration of pulse from ECHO pin
  // it would be so much better to use interrupts instead but the only two supported pins on the arduino are already in use
  unsigned long pulseDuration = pulseIn(SONAR_ECHO, HIGH, SONAR_MAX_WAIT_TIME);
  
  // Calculate distance from pulse duration
  return pulseDuration * 0.034 / 2;  // Speed of sound in air is approximately 34 cm/ms
}


void detectFoe() {
  // Calculate time since last measurement
  currentMillisMainTimer = millis();
  float deltaTime = (currentMillisMainTimer - prevTimeSonar) / 1000.0; // Convert milliseconds to seconds
  
  float distance = ping();  // distance in centimeters

  // Calculate velocity as change in distance over time
  if (distance != 0 && prevDistance != 0 && pings <= 5) {
    pings++;
  }
  else {
    pings = 0;
  }
  float velocity = (distance - prevDistance) / deltaTime;

  // Print distance & velocity to serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm\t");

  Serial.print("Velocity: ");
  Serial.print(velocity);
  Serial.println(" cm/s");

  // if distance is smaller than a certain value, a foe is detected
  if (pings >= 5 && distance <= ATTACK_DISTANCE_LIMIT) {    
    Serial.println("Foe detected...");
    if (velocity < -20.0) {
      defend(); // if the foe is coming at us at a certain speed, defend
    }
    else {
      attack();  // if the foe is not going in our direction, attack
    }
  }
  
  // Update previous distance and time
  prevDistance = distance;
  prevTimeSonar = currentMillisMainTimer;
}


void turnAround(byte direction) {
  Serial.println("Turning around...");

  digitalWrite(LED_2, HIGH);  // turn on LED to indice turning around
  if (direction == 0) {
    move(255, -255);
  }
  else if (direction == 1) {
     move(-255, 150);
  } 
  else if (direction == 2) {
     move(150, -255);
  }

  // generate a random delay before continuing
  delay(random(400, 1000));

  // start speeding up again
  move(255, 255);

  digitalWrite(LED_2, LOW);
}


// Attacking consists of driving forward full-steam ahead until we hit a line or lose the target
void attack() {
  Serial.println("Attacking!");
  digitalWrite(LED_14, HIGH);  // turn on LED to indicate attacking

  move(255, 255);
  // do this until "lock" is lost or the robot moves over a line
  // this is a subroutine so it needs to do the line checking while running
  while(ping() != 0.0) {
    if (detectLine()) break;
  }
  digitalWrite(LED_14, LOW);
}


// defending basically consists of turning into a random direction for a random amount of time
void defend() {
  Serial.println("Defending!");
  digitalWrite(LED_2, HIGH);  // turn on LED to indicate defending

  bool direction = random(2);
  if (direction) {
    move(255, -255);
  } else {
     move(-255, 255);
  }

  // do not use delay here as we need to call other functions in this subroutine
  unsigned long currentMillis = millis();
  unsigned long endTime = currentMillis + random(200, 1500);
  
  // this is a subroutine so it needs to do the line checking while running
  while (currentMillis < endTime) {
    detectLine(); // this ensures that at no time the bot leaves the area on itself
    currentMillis = millis();
  }

  move(255, 255);
  digitalWrite(LED_2, LOW); 
}


void moveRandomly() {
  // this part randomly changes the direction of the bot to ensure unpredictabiltiy
  currentMillisMainTimer = millis();
  if (currentMillisMainTimer >= endTimeMainTimer) {
    // Generate a random number to determine which motor will be slower
    bool leftMotorSlower = random(2); // Randomly selects either 0 or 1
    
    // Define speeds for both motors
    unsigned short leftMotorSpeed, rightMotorSpeed;
    if (leftMotorSlower) {
      leftMotorSpeed = random(100, 255);  // Random speed for the left motor (slower)
      rightMotorSpeed = 255; // Full speed for the right motor
    } else {
      leftMotorSpeed = 255; // Full speed for the left motor
      rightMotorSpeed = random(100, 255); // Random speed for the right motor (slower)
    }
    
    // Move the motors with the calculated speeds
    move(leftMotorSpeed, rightMotorSpeed);
    endTimeMainTimer = millis() + random(1000, 5000);
  }
}


void loop() {
  // the functions beyond have have their own subroutines so no further handling is required
  detectLine();
  detectFoe();
  moveRandomly();
}
