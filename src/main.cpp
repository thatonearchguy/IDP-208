#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Arduino_LSM6DS3.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>
#include <DFRobot_URM09.h>
#include <TimerOne.h>
#include "pathFinder.h"

#define ABS_FORWARD 2
#define ABS_BACKWARD 4
#define ABS_LEFT 3
#define ABS_RIGHT 1
#define REVERSE 5

#define RED_STATION 5
#define GREEN_STATION 6
#define BASE_STATION 0

#define RED 1
#define BLACK 0

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftWheel = AFMS.getMotor(1);
Adafruit_DCMotor *rightWheel = AFMS.getMotor(2);
Adafruit_TCS34725 TCS = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
DFRobot_URM09 URM09; 
Servo doorServo;

//Pin definitions

int colDetectPin = 1;
int leftJctPin = 2;
int rightJctPin = 3;
int redLedPin = 4;
int greenLedPin = 5;
int blueLedPin = 6;
//colour sensor is i2C connected
//IMU is i2C connected
//motor shield is i2C connected
int servoPin = 9;

//Calibration Values
unsigned long lastMillis;
int timer0_of_ms = 3;
int timer2_overflows = 0;
int blueLedRate = 2; //Hz
int timer2_of_ms = 16;
int blue_led_timeouts = ((1000) / (blueLedRate * timer2_of_ms));

//Servo
int servoOpenAngle = servoOpenAngle;
int servoCloseAngle = 0;


//PID
uint8_t avgMotorSpeed = 150; 

float Kp = 1;
float Ki = 0.3;
float Kd = 0.6;

float prop;
float integ;
float deriv;
int lastError;
int correction;

bool insideEdge; //used to tell PID which edge to follow (inside or outside)

//Colour sensor
int idealLightValue = 350;
uint8_t numAvgSamples = 5;

//Junction detection
bool leftJctDetect = false;
bool rightJctDetect = false;
bool tJctDetect = false;

//Pathfinding
uint8_t currDirect = ABS_FORWARD; //default is forwards
uint8_t currNode = 0;
node graph[numVert][numVert];
int bestPath[numVert];
direction bestPathDirections[numVert];
int distance[numVert];
bool nearBlock = false;
bool nearStation = false;
uint8_t destinationNode;

uint8_t blockIndices[] = {7, 12, 14, 17};
uint8_t blocksCollected = 0;

//Ranging Sensor
int block_threshold_mm = 50; //adjust for chassis geometry
int wall_threshold_mm = 80; //adjust for station geometry

int station_reverse_timeout_ms = 500;

//IMU
const int integration_interval = 3000; //microseconds - 3ms, tune depending on integration accuracy.
float yawAngle = 0;
float yawData;
bool turnReady = false;
int newDirect;
bool integrating = false;

bool verify_turn();
void jct_int_handler();
void make_turn(uint8_t new_direc);
void get_next_turn_dummy(int newDirection);
void calculate_pid(int valAvg, int *correct);
void calculate_angle();
float get_rotation_data();

ISR(TIMER0_COMPA_vect)
{
  OCR0A += 192;
  if(integrating) {
    calculate_angle();
  }
}


ISR(TIMER2_OVF_vect)
{
  timer2_overflows++;
  if(timer2_overflows >= blue_led_timeouts)
  {
    if(leftWheel->getSpeed() + rightWheel->getSpeed() == 0)
    {
      digitalWrite(blueLedPin, LOW);
    }
    else
    {
      digitalWrite(blueLedPin, !digitalRead(blueLedPin));
    }
    timer2_overflows = 0;
  }
  
}


void setup() {

  Serial.begin(9600);

  //timer0 - repurposed for delay and numerical integration.
  //we don't need millis or micros in our program, and Servo.h uses timer1 so we're safe to take over timer0.
  //but to not break delay, we must reimplement delay with the new timer overflow value. 
  noInterrupts();


  OCR0A = 192;
  TIMSK0 |= B00000011; // Enable Timer Overflow Interrupt

  //use timer2 to blink blue LED while in motion.
  //timer2 on ATMega328p is 8 bit only, so maximum interval with 1024 prescaler is 16ms. so we simply wait for 31 interrupts before toggling the LED. not ideal... but hey ho that's what we gotta do in the microcontroller world.
  //can't use timer1 because we need it for the servo and the numerical integration :)))
  TCCR2A = 0;          // Init Timer2A
  TCCR2B = 0;          // Init Timer2B
  TCCR2B |= B00000111; // Prescaler = 1024
  TIMSK2 |= B00000001; // Enable Timer Overflow Interrupt
  interrupts();

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  if(!TCS.begin()) {
    Serial.println("Could not find colour sensor. Check wiring.");
    while (1);
  }

  if(!URM09.begin()) {
    Serial.println("Could not find ranging sensor. Check wiring.");
    while (1);
  }

  //Small distances, initialise with MEASURE_RANG_150
  URM09.setModeRange(MEASURE_MODE_AUTOMATIC, MEASURE_RANG_150);

  //set line sensor pins
  pinMode(colDetectPin, INPUT);
  pinMode(leftJctPin, INPUT);
  pinMode(rightJctPin, INPUT);

  //set colour led output pins
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftJctPin), jct_int_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(rightJctPin), jct_int_handler, RISING);
  
  correction = 0;

  Timer1.initialize(integration_interval);


  initialise(graph);

  //set route for first package#
  destinationNode = 7;
  dijkstra(graph, 0, destinationNode, bestPath, bestPathDirections, distance);
  returnDirection(graph, bestPath, bestPathDirections);

  leftWheel->run(FORWARD);
  rightWheel->run(FORWARD);
}

void calculate_angle()
{
  yawAngle += 0.5 * (yawData + get_rotation_data()) * (integration_interval / 1e6);
}

void jct_int_handler()
{
  leftJctDetect = digitalRead(leftJctPin);
  rightJctDetect = digitalRead(rightJctPin);
  tJctDetect = leftJctDetect & rightJctDetect;
  
  turnReady = true;

}


void calculate_pid(int valAvg, int* correct)
{
  int error = idealLightValue - valAvg;
  prop = error;
  integ += error;
  deriv = error - lastError;

  *correct = int(Ki * integ + Kp * prop + Kd * deriv); 
  lastError = error;
}


void get_next_turn_dummy(int newDirection)
{
  //connect to pathfinding logic,
  //get absolute value of next turn
  if (newDirection++ > 4) newDirection = 1;
}

void get_next_turn(int newDirection)
{
  currNode ++;
  if(bestPath[currNode] == destinationNode) {
    if (destinationNode == RED_STATION || destinationNode == GREEN_STATION)
    {
      nearStation = true;
    }
    else
    {
      nearBlock = true;
    }
  }
  newDirection = bestPathDirections[currNode];
}




float get_rotation_data()
{
  float x, y, z;
  if(IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(x, y, z);
  }
  return z;
}


uint16_t get_colour_data()
{
  uint16_t r, g, b, c;
  int valSum = 0;
  for(int i = 0; i < numAvgSamples; i ++)
  {
    TCS.getRawData(&r, &g, &b, &c);
    valSum += c;
  }
  return int(valSum/numAvgSamples);
}

void pid_motor_regulate(int correction)
{
  leftWheel->setSpeed(constrain((avgMotorSpeed - correction), 0, 255));
  rightWheel->setSpeed(constrain((avgMotorSpeed + correction), 0, 255));
}


void make_turn(uint8_t newDirect)
{
  if(newDirect == REVERSE)
  {
    leftWheel->run(BACKWARD);
    rightWheel->run(BACKWARD);
    //do not update currDirect to allow next turn to happen correctly.
  }
  else
  {
    yawAngle = 0;
    Timer1.attachInterrupt(calculate_angle); //start numerically integrating rotation
    //this will make 180 degree rotations always go anticlockwise, but with the current geometry 180 degree turns should never be carried out.
    int desiredAngle = (((currDirect - newDirect)+2)%4-2) * (PI/2); //gyro is in radians. one edge case - modulus any change above 180 degrees to save time
    
    leftWheel->setSpeed(avgMotorSpeed);
    rightWheel->setSpeed(avgMotorSpeed);

    if(desiredAngle > 0)
    {
      leftWheel->run(FORWARD);
      rightWheel->run(BACKWARD);
    }
    else
    {
      leftWheel->run(BACKWARD);
      rightWheel->run(FORWARD);
    }

    //account for negative (counterclockwise) case
    while(abs(yawAngle) < abs(desiredAngle)) //wait

    Timer1.detachInterrupt(); //release timer1 for PWM use
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);

    leftWheel->run(FORWARD);
    rightWheel->run(FORWARD);
    currDirect = newDirect; //orientation does not change in reverse.
  }

}


void loop(void) {

  uint16_t valAvg = get_colour_data();
  calculate_pid(valAvg, &correction);
  pid_motor_regulate(correction);

  Serial.print("Current angle: ");
  Serial.println(yawAngle);

  if(turnReady)
  {
    /*

    OPTIMISATION/FAILSAFE
    check numerical integration of accelerometer x/y and ensure
    total travelled distance is within expected value (compare against weight)

    */
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
    get_next_turn_dummy(newDirect);
    make_turn(newDirect);
    turnReady = !turnReady;
  }

  if(nearBlock)
  {
    //could probably refactor but cba - it's more readable this way too

    avgMotorSpeed = 100; //slow down robot
    //move forward under PID until distance sensor trips
    while(int(URM09.getDistance() * 10) > block_threshold_mm) //can substitute for ToF sensor depending on chassis geometry and accuracy requirements. 
    {
      uint16_t valAvg = get_colour_data();
      calculate_pid(valAvg, &correction);
      pid_motor_regulate(correction);

    }
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);

    doorServo.write(servoOpenAngle); //extra tolerance
    delay(50);
    avgMotorSpeed = 50; // super slow to prevent hitting the front of the block
  
    while (int(URM09.getDistance() * 10) < block_threshold_mm)
    {
      uint16_t valAvg = get_colour_data();
      calculate_pid(valAvg, &correction);
      pid_motor_regulate(correction);
    }
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
    //close door
    doorServo.write(servoCloseAngle);
    delay(50); //wait for servo to move. 

    bool red = digitalRead(colDetectPin);

    int stationNode;
    if(red)
    {
      stationNode = 5;
      digitalWrite(redLedPin, HIGH);
    }
    else
    {
      stationNode = 6;
      digitalWrite(greenLedPin, HIGH);
    }
    //block is now captured, load route for appropriate destination.
    dijkstra(graph, destinationNode, stationNode, bestPath, bestPathDirections, distance);
    destinationNode = stationNode;
    get_next_turn(newDirect);
    
    delay(6000); //wait for 5+ seconds as required by task specification

    make_turn(newDirect); //this should hopefully always be a reverse when going from the block - which will be done under PID.  
    blocksCollected++; 

  }

  if(nearStation)
  {
    // could probably refactor but cba - it's more readable this way too

    avgMotorSpeed = 100; // slow down robot

    while (int(URM09.getDistance() * 10) < wall_threshold_mm)
    {
      uint16_t valAvg = get_colour_data();
      calculate_pid(valAvg, &correction);
      pid_motor_regulate(correction);
    }
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
    // close door
    doorServo.write(servoOpenAngle);
    delay(50); // wait for servo to move.

    leftWheel->run(BACKWARD);
    rightWheel->run(BACKWARD);

    leftWheel->setSpeed(50);
    rightWheel->setSpeed(50);

    delay(station_reverse_timeout_ms);

    doorServo.write(servoCloseAngle);
    delay(50);

    // block is now delivered after , load route for appropriate destination.
    int blockNode = blockIndices[blocksCollected];

    dijkstra(graph, destinationNode, blockNode, bestPath, bestPathDirections, distance);

    destinationNode = blockNode;

    get_next_turn(newDirect);
    make_turn(newDirect); // this should hopefully be a reverse, and after calibration manual reverse should go far back enough to allow rest to be done under PID.
  }

}