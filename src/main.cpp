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
#define LINE_THRESHOLD 60

#define SONAR_TX 0
#define SONAR_RX 0

#define SPEED 0x8F

void detectLine();
void turnAround(byte);
void detectFoe();
void attack();
void defend();
void move(short, short);
void moveRandomly();


unsigned long currentMillisMainTimer = millis();
unsigned long endTimeMainTimer = currentMillisMainTimer;

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

  // these are just here to show that I tried with interrupts, unfortunately digital read does not work here. (always 0)
  // attachInterrupt(digitalPinToInterrupt(LINE_L), leftLineInterrupt, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(LINE_R), rightLineInterrupt, CHANGE);

  // create seed for random numbers, taking the analog value of pin 0. 
  randomSeed(analogRead(0));

  //Then set the start value of the signals to zero: 
  analogWrite(RF, 0);   
  analogWrite(RB, 0);   
  analogWrite(LF, 0);   
  analogWrite(LB, 0);

  digitalWrite(LED_LINEFINDER, HIGH);

  //Enable serial for debugging
  Serial.begin(9600);

  //Startup delay:
  delay(2000);
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


void detectLine() {
  bool rightLine = analogRead(LINE_L) <= LINE_THRESHOLD;
  bool leftLine = analogRead(LINE_R) <= LINE_THRESHOLD;
  
  // Serial.println(analogRead(LINE_L));
  //either fully backup or depending what sensor hits first backup in optimal direction.
  if (rightLine && leftLine) {
    turnAround(0);
  }
  else if (rightLine) {
    turnAround(1);
  }
  else if (leftLine) {
    turnAround(2);
  }
}


// TODO: this when sensor is ready
void detectFoe() {
  // use sonar sensor to find a foe
  digitalWrite(LED_14, HIGH);  // turn on LED to indice a foe has been found
  // Serial.println("Foe detected...");

  // if the foe is coming at us at a certain speed, defend

  // if the foe is not going in our direction, attack
  digitalWrite(LED_14, LOW);
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
  delay(random(200, 1000));

  // start speeding up again
  // speedRight = 0;
  // speedLeft = 0;
  move(255, 255);

  digitalWrite(LED_2, LOW);
}


// TODO: THIS
// Attacking consists of driving forward full-steam ahead until we hit a line or lose the target
void attack() {
  Serial.println("Attacking!");
  digitalWrite(LED_14, HIGH);  // turn on LED to indicate attacking

  move(255, 255);
  // do this until "lock" is lost or the robot moves over a line
  // this is a subroutine so it needs to do the line checking while running
  while(true) {
    detectLine();
    delay(10);
  } 

  move(255, 255);

  digitalWrite(LED_14, LOW);
}


// defending basically consists of turning into a random direction for a random amount of time
void defend() {
  Serial.println("Defending!");
  digitalWrite(LED_14, HIGH);  // turn on LED to indicate defending

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
    delay(10);
  }

  move(255, 255);
  digitalWrite(LED_14, LOW); 
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

  delay(10);
  // defend();
}
