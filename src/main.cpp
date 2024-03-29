//------------------------Include packages---------------------------
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Arduino_LSM6DS3.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>
//#include <DFRobot_TCS34725.h>
#include <DFRobot_VL53L0X.h>
#include <DFRobot_URM09.h>
#include "pathFinder.h"
#include "calib.h" //calibration constants
#include "dec.h" //global variables & function prototypes
#include <avr/wdt.h>


#define DEBUG 0 //added debug switch to save some SRAM (optimise out all the serial calls)
#define EDGE_FLIPPING 0
#define DISABLE_SERVO 0

#if DEBUG
  #define LOG_NEWLINE(msg) Serial.println(msg)
  #define LOG_INLINE(msg) Serial.print(msg)
#else
  #define LOG_NEWLINE(msg)
  #define LOG_INLINE(msg)
#endif

//------------------------Declare / Initialise components---------------------------
// motor
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftWheel = AFMS.getMotor(1); // port 1
Adafruit_DCMotor *rightWheel = AFMS.getMotor(2); // port 2

// Light sensor
// period: 2.4ms, gain: 4x
Adafruit_TCS34725 TCS = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X); 
DFRobot_VL53L0X VL53; 
// DFRobot_URM09 URM09; 

Servo doorServo; // servo motor for box collection mechanism

uint8_t currDirect = ABS_FORWARD; //default is forwards


void setup() {
  #if DEBUG
  Serial.begin(9600);
  delay(100);
  #endif
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    LOG_NEWLINE(F("Could not find Motor Shield. Check wiring."));
    while (1);
  }

  //Binned IMU because Uno Rev2 WiFi i2C implementation is defective.
  /*
  if (!IMU.begin()) {
      LOG_NEWLINE(F("Failed to initialize IMU!"));

      while (1);
  }
  */
  LOG_NEWLINE(F("Hello! I am IDP L208!"));
  

  //Initialise ToF sensor (VL53L0X) in continuous, high accuracy mode. 
  VL53.begin(0x50);
  delay(10);
  VL53.setMode(VL53.eContinuous,VL53.eHigh);
  delay(10);
  VL53.start();
  delay(50);

  if(!TCS.begin()) {
    LOG_NEWLINE(F("Could not find colour sensor. Check wiring."));
    while (1);
  }

  //set line sensor pins
  pinMode(colDetectPin, INPUT);
  pinMode(leftJctPin, INPUT);
  pinMode(rightJctPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(crashSensorPin, INPUT);

  //set colour led output pins
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);

  correction = 0;

  initialise(graph);

  //set route for first package
  uint8_t baseNode = BASE_STATION;
  start_new_journey(&baseNode, &destinationNode, &newDirect);

  //doorServo.attach(servoPin);
  leftWheel->run(FORWARD);
  rightWheel->run(FORWARD);

  //timer0 - repurposed for delay and numerical integration.
  //we don't need millis() or micros() in our program, and Servo.h uses timer1 so we're safe to take over timer0.
  //but to not break delay, we must adjust the microseconds per tick value in Arduino core's wiring.c
  noInterrupts();

  //platformio.ini has a compiler flag that will reveal the appropriate timer setup code for ATMega328p or ATMega4809 depending on which Arduino is being used. 
  #ifdef ARDUINO_UNO
  //other modifications made in wiring.c
  OCR0A = 192;
  TIMSK0 |= B00000011; // Enable Timer Overflow Interrupt

  //use timer2 to blink blue LED while in motion.
  //timer2 on ATMega328p is 8 bit only, so maximum interval with 1024 prescaler is 16ms. so we simply wait for 15 interrupts before toggling the LED. not ideal... but hey ho that's what we gotta do in the microcontroller world.
  //can't use timer1 because we need it for the servo (Servo.h takes exclusive control of timer1)
  TCCR2A = 0;          // Init Timer2A
  TCCR2B = 0;          // Init Timer2B
  TCCR2B |= B00000111; // Prescaler = 1024
  TIMSK2 |= B00000001; // Enable Timer Overflow Interrupt
  PCICR |= B00000100; // Enables Ports D Pin Change Interrupts
  PCMSK2 |= ((1 << crashSensorPin) | (1 << leftJctPin) | (1 << rightJctPin)); //ASSUMES WE ARE CONNECTED ON D BANK!!
  //PCMSK2 |= B00001101; // PCINT16 (pin 0), PCINT18 (pin 2), PCINT1NT19 (pin 3)

  #endif

  #ifdef ARDUINO_WIFI
  //TCB0 and TCB1 are free on ATMega4809
  //using 64x scaler of TCA, results in 50ms tick

  TCB0.CCMP = 0x1D3; //set compare value for 3ms tick

  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm | TCB_RUNSTDBY_bm; //specify TCA 64x scaler, start TCB0 and use Run Standby mode.

  TCB0.INTCTRL = TCB_CAPT_bm;


  TCB1.CCMP = 0x7A12; 

  TCB1.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm | TCB_RUNSTDBY_bm; //specify TCA 64x scaler, start TCB1 and use Run Standby mode.

  TCB1.INTCTRL = TCB_CAPT_bm;

  attachInterrupt(digitalPinToInterrupt(leftJctPin), jct_int_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(rightJctPin), jct_int_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(crashSensorPin), jct_int_handler, FALLING);

  #endif


  interrupts();

  digitalWrite(redLedPin, HIGH);
  digitalWrite(greenLedPin, HIGH); //indicate that robot is ready to operate, put in position and press button to begin. 
  //delay(1000);
  while (!digitalRead(buttonPin));
  delay(200); //debounce
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
  delay_under_manual(600);
  turnReady = false;
}

void loop(void) {
  
  uint16_t valAvg = get_colour_data();
  calculate_pid(valAvg, &correction);
  pid_motor_regulate(correction);

  //In the main loop we poll turnReady, nearBlock, and nearStation flags which are set by interrupts and interrupt handlers to clean up the flow of the code and make it more performant.

  if(turnReady) //if a junction was detected
  {
    //leftWheel->setSpeed(0);
    //rightWheel->setSpeed(0);
    if(!finishedRun) //if we haven't picked up all the blocks
    {
      get_next_turn(&newDirect); //get the next turn from the navigation
      make_turn(&newDirect); //make the robot turn to this new direction
      turnReady = !turnReady; //reset flag
    }
    else //if we're finished we've reached the starting box
    {
      celebrate_and_finish();
    }
  }

  if(recovery)
  {
    //WORKAROUND - chassis and motor movement is very unreliable, we put the junction detection on a timeout so if we detect within 2 seconds we need to recover instead of turn. 
    int desiang = (clockwise_recovery) ? 90 : -90;
    set_motor_directions(&desiang);
    leftWheel->setSpeed(100);
    rightWheel->setSpeed(100);
    
    uint16_t valAvg = get_colour_data();
    while(valAvg < idealLightValue)
    {
      valAvg = get_colour_data();
      LOG_NEWLINE(valAvg);
    } //wait until line detected again
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
    leftWheel->run(FORWARD);
    rightWheel->run(FORWARD);
    delay(40);
    recovery = false;
    turnReady = false; //prevent robot then making the next turn after the line has been found again. 
    jctDetectTime += 500; //catch subsequent recoveries
  }
  
  if(nearBlock)
  {

    avgMotorSpeed = 150; //slow down robot
    //move forward under PID until distance sensor trips
    open_door();
    delay_under_pid(550);
     //give time for robot to straighten out before we start looking at distance to avoid tripping on surroundings.

    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);

    
    distance_under_pid(wall_threshold_mm); //distance sensor can only see after door is opened.
    
    //scan block colour (sensor can be placed next to the castor)
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
    bool red = digitalRead(colDetectPin);
    
    close_door();

    uint8_t stationNode;

    if(red)
    {
      stationNode = RED_STATION;
      digitalWrite(redLedPin, HIGH);
    }
    else
    {
      stationNode = GREEN_STATION;
      digitalWrite(greenLedPin, HIGH);
    }
    //block is now captured, load route for appropriate destination.
    start_new_journey(&destinationNode, &stationNode, &newDirect);

    destinationNode = stationNode;
    delay(1333); //wait for 5+ seconds as required by task specification

    make_turn(&newDirect); //this should hopefully always be a reverse when going from the block - which will be done under PID.  
    blocksCollected++; 
    nearBlock = false;
    turnReady = false;
    recovery = false;

  }
  
  
  if(nearStation)
  {
    avgMotorSpeed = 170; // slow down robot

    delay_under_pid(station_approach_timeout_ms);

    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);

    open_door();
    //reverse out
    delay_under_manual(230, false);

    delay(100);
    delay_under_manual(station_reverse_timeout_ms, true);
    
    close_door();
    uint8_t blockNode = 0; //Zero corresponds to the base station - (will be overriden by the actual block index if not all of them have been picked up)
    //this also has the advantage of very easily being able to add the extension task of just taking blocks from 0 to the stations.

    turnReady = false;
    recovery = false; //prevent tripping on the junction edge affecting the result. 
    start_new_journey(&destinationNode, &blockNode, &newDirect);
    // block is now delivered after , load route for appropriate destination.

    destinationNode = blockNode;
    digitalWrite(redLedPin, LOW); //don't care about the colour, just write both GPIOs to low.
    digitalWrite(greenLedPin, LOW);
    make_turn(&newDirect); // this should hopefully be a reverse, and after calibration manual reverse should go far back enough to allow rest to be done under PID.
    nearStation = false;
    avgMotorSpeed = 220;
  }
  
}

//------------------------Define helper functions---------------------------

/// @brief Function to change the position of the servo motor to open door
/// @param void
/// @return void
void open_door() {
  #if !DISABLE_SERVO
  doorServo.write(servoCloseAngle);
  doorServo.attach(servoPin);
  for (int pos = servoCloseAngle; pos>= servoOpenAngle; pos--){
    doorServo.write(pos);
    delay(10);
  }
  doorServo.detach();
  delay(100);
  #endif
}

/// @brief Function to change the position of the servo motor to close door 
/// @param void
/// @return void
void close_door() {
  #if !DISABLE_SERVO
  //doorServo.write(servoOpenAngle);
  doorServo.attach(servoPin);
  for (int pos = servoOpenAngle; pos<= servoCloseAngle; pos++){
    doorServo.write(pos);
    delay(10);
  }
  doorServo.detach();
  delay(100);
  #endif
}

/// @brief Function to numerically integrate gyro data to calculate the yaw angle 
/// @param void
/// @return void
void calculate_angle()
{
  yawAngle += 0.5 * (yawData + get_rotation_data()) * (timer0_of_ms / 1e3);
}

/// @brief Interrupt handler for junction detection 
/// @param void
/// @return void
void jct_int_handler()
{
  leftJctDetect = digitalRead(leftJctPin);
  rightJctDetect = digitalRead(rightJctPin);
  tJctDetect = leftJctDetect & rightJctDetect;
  if((millis() - jctDetectTime > jctTimeout))
  {
    //jctDetectTime = millis();
    turnReady = true;
  }
  else if(!turnReady)
  {
    recovery = true;
    if(leftJctDetect) clockwise_recovery = false;
    else if(rightJctDetect) clockwise_recovery = true;
  }


}

/// @brief PID calculator for line following
/// @param int valAvg - averaged colour temperature reading, int* correct - pointer to correction variable
/// @return void
void calculate_pid(int valAvg, int* correct)
{
  int error = idealLightValue - valAvg;
  prop = error;
  integ += error;
  integ > max_integ_val ? max_integ_val : integ; //clamp between bounds to prevent wind-up
  integ < min_integ_val ? min_integ_val : integ;
  deriv = error - lastError;

  *correct = int(Ki * integ + Kp * prop + Kd * deriv); 
  lastError = error;
}

/// @brief Decrements direction angle for a dummy clockwise rotation pattern
/// @param uint8_t* newDirection - pointer to direction variable
/// @return void
void get_next_turn_dummy_clockwise(uint8_t* newDirection)
{
  //connect to pathfinding logic,
  //get absolute value of next turn
  (*newDirection)--; //int overflow when 0 decremented.
  if (*newDirection == 255) *newDirection = 3;
}

/// @brief Increments direction angle for a dummy anticlockwise rotation pattern
/// @param uint8_t* newDirection - pointer to direction variable
/// @return void
void get_next_turn_dummy_anticlockwise(uint8_t* newDirection)
{
  //connect to pathfinding logic,
  //get absolute value of next turn
  (*newDirection) ++;
  if (*newDirection > 3) *newDirection = 0;
}

/// @brief Updates direction angle with that of the next node, generated by the pathfinding.
/// @param uint8_t* newDirection - pointer to direction variable
/// @return void
void get_next_turn(uint8_t* newDirection)
{
  currNode ++;
  if(bestPath[currNode+1] == destinationNode) {
    if (destinationNode == RED_STATION || destinationNode == GREEN_STATION)
    {
      nearStation = true;
      LOG_NEWLINE("Near station!!");
    }
    else if (destinationNode == BASE_STATION) 
    {
      LOG_NEWLINE("Near base station!");
      //no nearBlock
    }
    else
    {
      nearBlock = true;
      LOG_NEWLINE("Near block!!");
    }
  }
  LOG_INLINE("New node: ");
  LOG_NEWLINE(bestPath[currNode]);
  *newDirection = bestPathDirections[currNode];
}



/// @brief Gets z axis gyroscope data for numerical integration to calculate angle
/// @param void
/// @return float - z axis data
float get_rotation_data()
{
  float x, y, z;
  if(IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(x, y, z);
  }
  return z;
}

/// @brief Gets colour data from the colour sensor and averages by numAvgSamples
/// @param void
/// @return uint16_t
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

/// @brief Applies correction to the motor speeds to follow the line
/// @param int correction - correction variable
/// @return void
void pid_motor_regulate(int correction)
{
  if(insideEdge)
  {
    leftWheel->setSpeed(constrain((avgMotorSpeed - correction), 0, 255));
    rightWheel->setSpeed(constrain((avgMotorSpeed + correction), 0, 255));
  }
  else
  {
    leftWheel->setSpeed(constrain((avgMotorSpeed + correction), 0, 255));
    rightWheel->setSpeed(constrain((avgMotorSpeed - correction), 0, 255));
  }
}

/// @brief Runs PID line following for a certain time
/// @param uint16_t timeout - how long to PID follow for
/// @return void
void delay_under_pid(uint16_t timeout)
{
  unsigned long time1 = millis();
  while(millis() - time1 < timeout) {
      uint16_t valAvg = get_colour_data();
      calculate_pid(valAvg, &correction);
      pid_motor_regulate(correction);

    }
}

/// @brief Runs robot forwards blindly for a certain time
/// @param uint16_t timeout - how long to move for, bool reverse - whether to run the motors forwards or in reverse.
/// @return void
void delay_under_manual(uint16_t timeout, bool reverse = false)
{
  leftWheel->setSpeed(225);
  rightWheel->setSpeed(225);

  //hardcode distance to the pivot point of the robot
  leftWheel->run(BACKWARD);
  rightWheel->run(BACKWARD);

  if(!reverse)
  {
    leftWheel->run(FORWARD);
    rightWheel->run(FORWARD);
    leftWheel->setSpeed(210);
    rightWheel->setSpeed(210);
  }

  delay(timeout); //hardcode time

  leftWheel->setSpeed(0);
  rightWheel->setSpeed(0);
}

/// @brief Runs PID line following until a certain distance is met
/// @param uint8_t threshold - distance in mm to line follow until
/// @return void
void distance_under_pid(uint8_t threshold)
{
  unsigned long time = millis();
  while((int(VL53.getDistance()) > threshold) && (millis() - time < distance_pid_timeout))
  {
    uint16_t valAvg = get_colour_data();
    calculate_pid(valAvg, &correction);
    pid_motor_regulate(correction);
   }
}

/// @brief Calculates the angle of rotation needed to turn to a specified direction
/// @param uint8_t* currDirect - pointer to current direction variable, uint8_t* newDirect - pointer to the variable containing the new direction, int* desiredAngle - pointer to variable to fill with the required rotation angle.
/// @return void
void calculate_angle_to_rotate(uint8_t* currDirect, uint8_t* newDirect, int* desiredAngle) {
    *desiredAngle = (*currDirect - *newDirect);
    if(abs(*desiredAngle) == 3)
    {
        *desiredAngle = (*desiredAngle % 2) * -1;
        //gyro is in radians. one edge case - modulus any change above 180 degrees to save time
    }
    *desiredAngle *= 90;
    //WORKAROUND - avoid hitting wall on rotation.
    if(abs(*desiredAngle) == 180 && bestPath[currNode] == 1)
    {
      *desiredAngle *= -1; 
    }
    LOG_INLINE("Angle to rotate: ");
    LOG_NEWLINE(*desiredAngle);
}

/// @brief Sets the correct motor directions for rotating to a new direction.
/// @param int* desAng - pointer to variable containing the rotation angle
/// @return void
void set_motor_directions(int* desAng)
{
  int desiredAngle = *desAng;
  if(desiredAngle > 0)
  {
    LOG_NEWLINE("Rotating right");
    leftWheel->run(BACKWARD); //motors are connected backwards so this makes no sense just roll with it
    rightWheel->run(FORWARD);
  }
  else if (desiredAngle < 0)
  {
    LOG_NEWLINE("Rotating left");
    leftWheel->run(FORWARD);
    rightWheel->run(BACKWARD);
  }

}


/// @brief Changes the direction of the robot to the specified new direction. Makes initial rotation manually before scanning for new line with the colour sensor
/// @param uint8_t* newDirect - pointer to new direction variable
/// @return void
void make_turn(uint8_t* newDirect)
{
  LOG_INLINE("Turning, ");
  LOG_INLINE("current direction: ");
  LOG_INLINE(currDirect);
  LOG_INLINE(" new direction: ");
  LOG_NEWLINE(*newDirect);
  if(*newDirect == REVERSE)
  {
    //digitalWrite(redLedPin, HIGH);
    //WORKAROUND - robot control parameters differ massively when in reverse due to flipped geometry. Kp -> 0 effectively disables PID while utilising existing loop.
    Kp = 0;
    Kd = 0; 
    leftWheel->run(BACKWARD);
    rightWheel->run(BACKWARD);
    leftWheel->setSpeed(210);
    rightWheel->setSpeed(200);
    delay(350);
    forwardDelayTime = 150;
    //insideEdge = false;
    //do not update currDirect to allow next turn to happen correctly.
  } 
  else
  {
    //insideEdge = true;
    Kp = 0.2; //when not reversing, ensure Kp is always set to the appropriate value.
    Kd = 0.2;
    if (*newDirect == FINISH)
    {
      finishedRun = true;
      *newDirect = ABS_BACKWARD;
    }
    if (*newDirect == INVALID)
    {
      leftWheel->setSpeed(0);
      rightWheel->setSpeed(0);
      while(1);
    }
    yawAngle = 0;
    int desiredAngle;
    
    calculate_angle_to_rotate(&currDirect, newDirect, &desiredAngle);
    
    //WORKAROUND - wheel slip when reversing, we move forwards less as a result.
    if(abs(desiredAngle) > 90)
    {
      delay_under_manual(240); //move forward until pivot hit

    }
    else
    {
      delay_under_manual(forwardDelayTime);
    }

    if(desiredAngle == 0)
    {
      LOG_NEWLINE("Going straight!");
      leftWheel->run(FORWARD);
      rightWheel->run(FORWARD);
      avgMotorSpeed = 220;
      //currDirect = *newDirect; direction remains same
      return; //skip rotation if going straight
    }
    
    //WORKAROUND - colour sensor can lose the correct edge in certain circumstances. We use logic to alternate the edge followed.
    #if EDGE_FLIPPING
    if(insideEdge)
    {
      if(desiredAngle < 0)
      {
        insideEdge = false;
      }
    }
    else
    {
      if(desiredAngle > 0)
      {
        insideEdge = true;
      }
    }
    #endif
    set_motor_directions(&desiredAngle);
    leftWheel->setSpeed(140);
    rightWheel->setSpeed(140);
    
    //WORKAROND - Motor stalls after extended period of rotation at 120, we increase this to 150 for 180 degree turns. 
    if(abs(desiredAngle) > 90)
    {
      leftWheel->setSpeed(150);
      rightWheel->setSpeed(150);
    }
    

    //WORKAROUND - IMU unavailable so we use the colour sensor to detect when we've reached the line again. 
    for(int i = 0; i < abs(desiredAngle); i+=90) //for loop will run this a second time to complete 180 degree turns correctly.
    {
      delay(rotDelayTime); //allow robot to move away from original line before we start scanning with the colour sensor. 
      uint16_t valAvg = get_colour_data();
      while(valAvg < rotColLineThreshold)
      {
        valAvg = get_colour_data();
        //LOG_NEWLINE(valAvg);
      } //wait until line detected again
    }


    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
    leftWheel->run(FORWARD);
    rightWheel->run(FORWARD);
    currDirect = *newDirect; //orientation does not change in reverse.
    avgMotorSpeed = 220;
    forwardDelayTime = 280;
    jctDetectTime = millis(); 
  }
}


uint8_t nextClosestBlock(int distance[numVert], uint8_t blockIndices[], status blockStatus[numBlocks]) {
    if (blocksCollected == numBlocks) {
        return 0;
    }
    int minDis = MAX_DIST;
    int closestBlockIndex = 0;
    for (int i = 0; i < numBlocks; i++) {
        if (blockStatus[i] == NOTCOMPLETED && distance[blockIndices[i]] < minDis) {
            minDis = distance[blockIndices[i]];
            closestBlockIndex = i;
        }
    }
    blockStatus[closestBlockIndex] = COMPLETED;
    //WORKAROUND - one of the nodes is way longer than the others
    if(blockIndices[closestBlockIndex] == 14)
    {
      wall_threshold_mm = 50;
      distance_pid_timeout = 2900;
    }
    else if(blockIndices[closestBlockIndex] == 17)
    {
      wall_threshold_mm = 50;
      distance_pid_timeout = 1800;
    }
    else if(blockIndices[closestBlockIndex] == 12)
    {
      wall_threshold_mm = 50;
      distance_pid_timeout = 800;
    }
    else if(blockIndices[closestBlockIndex] == 7 || blockIndices[closestBlockIndex] == 0)
    {
      wall_threshold_mm = 50;
      distance_pid_timeout = 1100;
    }
    return blockIndices[closestBlockIndex];
}

void start_new_journey(uint8_t* sourceNode, uint8_t* destinationNode, uint8_t* newDirect)
{
  dijkstra(graph, *sourceNode, bestPath, bestPathDirections, distance, parent);
  if (*destinationNode == 0) {
    // destination not yet set, need to find closest block to set destination
    // all blocks collected have been collected, nextClosestBlock will just return 0 (home square)
    *destinationNode = nextClosestBlock(distance, blockIndices, blockStatus);
  }
  getOptimalPath(parent, *destinationNode, bestPath, graph, bestPathDirections);
  //returnDirection(graph, bestPath, bestPathDirections);
  currNode = 0;
  *newDirect = bestPathDirections[currNode];
}

void celebrate_and_finish() {
//PID impossible due to geometry of starting region, so we manually drive until we cross the threshold.
      leftWheel->setSpeed(140);
      rightWheel->setSpeed(150);
      delay(1300); //calibrate for however long it takes to cross the threshold.
      leftWheel->setSpeed(0);
      rightWheel->setSpeed(0);

      delay(1000);
      leftWheel->run(BACKWARD);
      rightWheel->run(FORWARD);
      leftWheel->setSpeed(140);
      rightWheel->setSpeed(140);

      delay(400);

      leftWheel->setSpeed(0);
      rightWheel->setSpeed(0);
      wdt_disable();
      wdt_enable(WDTO_15MS);

      while(1); //We're now finished, infinite loop until power turned off.

      //IF WE HAVE TIME - ADD A LITTLE LIGHT SHOW??
}

#ifdef ARDUINO_UNO

ISR(PCINT2_vect)
{
  jct_int_handler();
}

//interrupt service routine for COMPA ATMega328p interrupt on Timer0. Used for numerically integrating gyroscope data at 333Hz
ISR(TIMER0_COMPA_vect)
{
  //preload counter value to trip every 3ms with prescaler - see datasheet
  OCR0A += 192;
  if(integrating) {
    calculate_angle(); 
  }
}

//interrupt service routine for Timer2
ISR(TIMER2_OVF_vect)
{
  timer2_overflows++;
  //since max timer length is 16ms, we count how many times the timer has overflowed (15) before we toggle the state of the LED.
  //gives us a near perfect 2Hz flash rate.
  if(timer2_overflows >= blue_led_timeouts)
  {
    //getSpeed() is a custom method implemented in the Adafruit Motor Shield library that keeps track of the *commanded* motor speed in a private attribute.
    if(leftWheel->getSpeed() + rightWheel->getSpeed() == 0)
    {
      //satisfies requirement to only have LED flashing during motion.
      digitalWrite(blueLedPin, LOW);
    }
    else
    {
      digitalWrite(blueLedPin, !digitalRead(blueLedPin));
    }
    timer2_overflows = 0;
  }
  
}
#endif

#ifdef ARDUINO_WIFI

//ATMega4809 has 16-bit timers that can specify a significantly higher timeout.

//We hijack TCB0 and TCB1 as they are unused in the Arduino framework. We can't set a period but we can set a compare value and the slowest prescaler (64x off TCA) to give up to 100ms tick rate.
ISR(TCB0_INT_vect) 
{
  if(integrating) {
      calculate_angle();
  }
  //tell TCB0 to interrupt again once the compare value is reached. No need to increment counter like on 328p as we can set an upper bound for interrupting.
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

ISR(TCB1_INT_vect)
{
  //16-bit timer allows us to interrupt less frequently and maintain 2hz flash rate. We opt for a 50ms tick, so every 5 = 250ms between each state change of LED.
  timer2_overflows++;
  if(timer2_overflows >= 5)
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
  TCB1.INTFLAGS = TCB_CAPT_bm;
}


#endif