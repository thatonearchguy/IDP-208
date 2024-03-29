#include <Arduino.h>

// Digital Pin definitions
// line sensors

#define ABS_FORWARD 1
#define ABS_BACKWARD 3
#define ABS_LEFT 2
#define ABS_RIGHT 0
#define REVERSE 4
#define FINISH 5
#define INVALID 6

#define RED_STATION 6
#define GREEN_STATION 5
#define BASE_STATION 0

#define RED 1
#define BLACK 0

const uint8_t crashSensorPin = 0;
const uint8_t colDetectPin = 1; // line sensor for box colour detection
const uint8_t leftJctPin = 2; // left line sensor
const uint8_t rightJctPin = 3; // left line sensor
// led 
const uint8_t redLedPin = 4; 
const uint8_t greenLedPin = 5;
const uint8_t blueLedPin = 6;

const uint8_t buttonPin = 7; // start program button
const uint8_t servoPin = 9; // servo motor for box collection mechanism

// colour sensor is i2C connected
// IMU is i2C connected
// motor shield is i2C connected
// distance sensor is i2C connected

// Calibration Values
// for angle calculation from gyroscope (not used)
unsigned long lastMillis;
const uint8_t timer0_of_ms = 3; 
// for flasjing blue LED during movement of robot
uint8_t timer2_overflows = 0; // initialising timer for flashing loop
const uint8_t blueLedUpdRate = 4; // flashing rate Hz
const uint8_t timer2_of_ms = 16; // ISR (looping) period set manually
const uint8_t blue_led_timeouts = uint8_t((1000) / (blueLedUpdRate * timer2_of_ms)); // determines when led flashes

//Colour sensor
int idealLightValue = 240; // threshold value for white
uint8_t numAvgSamples = 1; // take average of numAvgSamples light sensor reading

//Ranging Sensor
uint8_t block_threshold_mm = 42; //adjust for chassis geometry - 42mm door
uint8_t block_interior_threshold_mm = 42;
uint8_t wall_threshold_mm = 88; //adjust for station geometry

uint16_t station_reverse_timeout_ms = 150;
uint16_t station_approach_timeout_ms = 2900;

//Servo
const uint8_t servoOpenAngle = 63;
const uint8_t servoCloseAngle = 115;

//PID
uint8_t avgMotorSpeed = 220; 
const uint16_t rotDelayTime = 220;
uint16_t rotColLineThreshold = 140;
uint16_t forwardDelayTime = 280;
float Kp = 0.2;
float Ki = 0;
float Kd = 0.2;
const int max_integ_val = 300;
const int min_integ_val = -300;
const uint16_t jctTimeout = 600;    
unsigned long distance_pid_timeout = 1400;