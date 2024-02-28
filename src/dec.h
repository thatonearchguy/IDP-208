#include <Arduino.h>
#include "pathfinder.h"


float prop;
float integ;
float deriv;
int lastError;
int correction;

bool insideEdge; //used to tell PID which edge to follow (inside or outside)
bool finishedRun = false;

//Junction detection
bool leftJctDetect = false;
bool rightJctDetect = false;
bool tJctDetect = false;

//Pathfinding
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

//IMU
float yawAngle = 0;
float yawData;
bool turnReady = false;
uint8_t newDirect = 1;
bool integrating = false;

//------------------------Declare helper functions---------------------------
bool verify_turn();
void jct_int_handler();
void make_turn(uint8_t* new_direc);
void get_next_turn_dummy_anticlockwise(uint8_t* newDirection);
void get_next_turn_dummy_clockwise(uint8_t* newDirection);
void calculate_pid(int valAvg, int *correct);
void calculate_angle();
float get_rotation_data();
void delay_under_pid(uint16_t timeout);
void distance_under_pid(uint8_t threshold);
void start_new_journey(uint8_t* sourceNode, uint8_t* destinationNode, uint8_t* newDirect);
void celebrate_and_finish();
void set_motor_directions(int* desAng);
void delay_under_manual(uint16_t timeout);
uint16_t get_colour_data();
void pid_motor_regulate(int correction);
void get_next_turn(uint8_t* newDirection);