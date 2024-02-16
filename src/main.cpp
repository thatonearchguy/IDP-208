#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Arduino_LSM6DS3.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>
#include <DFRobot_URM09.h>

#define ABS_FORWARD 2
#define ABS_BACKWARD 4
#define ABS_LEFT 3
#define ABS_RIGHT 1

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftWheel = AFMS.getMotor(1);
Adafruit_DCMotor *rightWheel = AFMS.getMotor(2);
Adafruit_TCS34725 TCS = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
DFRobot_URM09 URM09; 

//Pin definitions

int colDetectPin = 1;
int leftJctPin = 2;
int rightJctPin = 3;

//Calibration Values

//PID
uint8_t avgMotorSpeed = 150; 

float Kp = 1;
float Ki = 0;
float Kd = 0.6;

float prop;
float integ;
float deriv;
int lastError;
int correction;

//Colour sensor
int idealLightValue = 350;
uint8_t numAvgSamples = 5;

//Junction detection
bool leftJctDetect = false;
bool rightJctDetect = false;
bool tJctDetect = false;

//Pathfinding
uint8_t currDirect;
uint8_t currNode;

//Ranging Sensor
int block_threshold_mm = 50; //adjust for chassis geometry

bool verify_turn(uint8_t new_direc, uint8_t currNode, uint8_t nextNode, uint8_t currDirect);
void jct_int_handler();
void make_turn(uint8_t new_direc);
int get_next_turn(int currDirect);

void setup() {

  Serial.begin(9600);
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

  attachInterrupt(digitalPinToInterrupt(leftJctPin), jct_int_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(rightJctPin), jct_int_handler, RISING);
  
  correction = 0;
}

void jct_int_handler()
{
  leftJctDetect = digitalRead(leftJctPin);
  rightJctDetect = digitalRead(rightJctPin);
  tJctDetect = leftJctDetect & rightJctDetect;
  /*
  //check if junction directions are what we expect for the current node
  int new_direc = get_next_turn(currDirect);
  make_turn(new_direc);
  */
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

int get_next_turn(int currDirect)
{
  //connect to pathfinding logic
  //and check if junction directions are what we expect for the current node
  //talk to motors and get it to swivel the bot 90 degrees in the desired direction
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

void set_motor_speed(int correction)
{
  leftWheel->setSpeed(constrain((avgMotorSpeed - correction), 0, 255));
  rightWheel->setSpeed(constrain((avgMotorSpeed + correction), 0, 255));
}

void loop(void) {
  leftWheel->run(FORWARD);
  rightWheel->run(FORWARD);
  uint16_t valAvg = get_colour_data();
  calculate_pid(valAvg, &correction);
  set_motor_speed(correction);

  if(leftJctDetect & rightJctDetect)
  {
    Serial.println("T/Cross Junction Detected");
  }

  else if(rightJctDetect)
  {
    Serial.println("Right Junction Detected!");
  }

  else if(leftJctDetect)
  {
    Serial.println("Left Junction Detected!");
  }


}