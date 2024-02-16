#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Arduino_LSM6DS3.h>
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_TCS34725.h>
#include <DFRobot_URM09.h>
#include <TimerOne.h>

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
uint8_t currNode;

//Ranging Sensor
int block_threshold_mm = 50; //adjust for chassis geometry

//IMU
const int integration_interval = 3000; //microseconds - 3ms, tune depending on integration accuracy.
float yawAngle = 0;
float yawData;
bool turnReady = false;
int newDirect;

bool verify_turn();
void jct_int_handler();
void make_turn(uint8_t new_direc);
void get_next_turn(int newDirection);
void calculate_pid(int valAvg, int *correct);
void calculate_angle(int yawData);
float get_rotation_data();

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

  Timer1.initialize(integration_interval);

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


void get_next_turn(int newDirection)
{
  //connect to pathfinding logic,
  //get absolute value of next turn
  if (newDirection++ > 4) newDirection = 1;
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
  yawAngle = 0;
  Timer1.attachInterrupt(calculate_angle); //start numerically integrating rotation
  int desiredAngle = (currDirect - newDirect) * (PI/2); //gyro is in radians
  
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

  leftWheel->fullOff();
  rightWheel->fullOff();

  currDirect = newDirect;

}

void loop(void) {
  leftWheel->run(FORWARD);
  rightWheel->run(FORWARD);
  uint16_t valAvg = get_colour_data();
  calculate_pid(valAvg, &correction);
  pid_motor_regulate(correction);

  Serial.print("Current angle: ");
  Serial.println(yawAngle);

  if(turnReady)
  {
    /*
    check numerical integration of accelerometer x/y and ensure
    total travelled distance is within expected value (compare against weight)

    */
    leftWheel->fullOff();
    rightWheel->fullOff();
    get_next_turn(newDirect);
    make_turn(newDirect);
    turnReady = !turnReady;
  }

}