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

#define DEBUG 0 //added debug switch to save some SRAM (optimise out all the serial calls)

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
  delay(20);
  VL53.setMode(VL53.eContinuous,VL53.eHigh);
  delay(20);
  VL53.start();
  delay(100);

  if(!TCS.begin()) {
    LOG_NEWLINE(F("Could not find colour sensor. Check wiring."));
    while (1);
  }

  //set line sensor pins
  pinMode(colDetectPin, INPUT);
  pinMode(leftJctPin, INPUT);
  pinMode(rightJctPin, INPUT);
  pinMode(buttonPin, INPUT);

  //set colour led output pins
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftJctPin), jct_int_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(rightJctPin), jct_int_handler, RISING);

  correction = 0;

  initialise(graph);

  //set route for first package
  destinationNode = 7;
  dijkstra(graph, 0, destinationNode, bestPath, bestPathDirections, distance);
  returnDirection(graph, bestPath, bestPathDirections);

  doorServo.attach(servoPin);
  leftWheel->run(FORWARD);
  rightWheel->run(FORWARD);

  //timer0 - repurposed for delay and numerical integration.
  //we don't need millis or micros in our program, and Servo.h uses timer1 so we're safe to take over timer0.
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

  #endif


  interrupts();

  digitalWrite(redLedPin, HIGH);
  digitalWrite(greenLedPin, HIGH); //indicate that robot is ready to operate, put in position and press button to begin. 
  //delay(1000);
  while (!digitalRead(buttonPin));
  delay(200); //debounce
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
}

void loop(void) {
  
  uint16_t valAvg = get_colour_data();
  calculate_pid(valAvg, &correction);
  pid_motor_regulate(correction);

  //In the main loop we poll turnReady, nearBlock, and nearStation flags which are set by interrupts and interrupt handlers to clean up the flow of the code and make it more performant.

  if(turnReady) //if a junction was detected
  {
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
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
  
  if(nearBlock)
  {
    //could probably refactor but cba - it's more readable this way too

    avgMotorSpeed = 140; //slow down robot
    //move forward under PID until distance sensor trips
    delay_under_pid(1500);
     //give time for robot to straighten out before we start looking at distance to avoid tripping on surroundings.
    distance_under_pid(block_threshold_mm);

    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);

    //open_door();
    //doorServo.write(servoOpenAngle); //extra tolerance
    //delay(400);
    /*
    //in case line ends early, we use manual operation to drive as close as possible to the block. 
    while (int(VL53.getDistance()) > block_interior_threshold_mm)
    {
      leftWheel->setSpeed(130);
      rightWheel->setSpeed(140);
    }
    
    
    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
    */
    //close door
    //close_door();
    
    /*doorServo.write(servoCloseAngle);
    delay(400); //wait for servo to move. */
    //scan block colour (sensor can be placed next to the castor)
    bool red = digitalRead(colDetectPin);

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
    //delay(6000); //wait for 5+ seconds as required by task specification

    make_turn(&newDirect); //this should hopefully always be a reverse when going from the block - which will be done under PID.  
    blocksCollected++; 
    nearBlock = false;
    
  }
  
  
  if(nearStation)
  {
    // could probably refactor but cba - it's more readable this way too

    avgMotorSpeed = 140; // slow down robot

    delay_under_pid(station_approach_timeout_ms);

    leftWheel->setSpeed(0);
    rightWheel->setSpeed(0);
    // open door
    //open_door();
    /* doorServo.write(servoOpenAngle);
    delay(400); // wait for servo to move. */

    //reverse out
    delay_under_manual(station_reverse_timeout_ms, true);

    
    //close_door();
    /* doorServo.write(servoCloseAngle);
    delay(50); */

    uint8_t blockNode = 0; //YIPEEE RETURN TO BASE (will be overriden by the actual block index if not all of them have been picked up)
    //this also has the advantage of very easily being able to add the extension task of just taking blocks from 0 to the stations.
    // block is now delivered after , load route for appropriate destination.
    if(!(blocksCollected >= sizeof(blockIndices)/sizeof(blockIndices[0])))
    {
      blockNode = blockIndices[blocksCollected];
    }

    start_new_journey(&destinationNode, &blockNode, &newDirect);
  
    destinationNode = blockNode;
    digitalWrite(redLedPin, LOW); //don't care about the colour, just write both GPIOs to low.
    digitalWrite(greenLedPin, LOW);
    make_turn(&newDirect); // this should hopefully be a reverse, and after calibration manual reverse should go far back enough to allow rest to be done under PID.
    nearStation = false;
    avgMotorSpeed = 225;
  }
  
}



//------------------------Define helper functions---------------------------
// Function to change the position of the servo motor to open door
// Parameters: void
// Returns: void
void open_door() {
  for (int pos = servoCloseAngle; pos>= servoOpenAngle; pos--){
    doorServo.write(pos);
    delay(3);
  }
}
// Function to change the position of the servo motor to close door 
// Parameters: void
// Returns: void
void close_door() {
  for (int pos = servoOpenAngle; pos<= servoCloseAngle; pos++){
    doorServo.write(pos);
    delay(5);
  }
}

// Function to numerically integrate gyro data to calculate the yaw angle 
// Parameters: void
// Returns: void
void calculate_angle()
{
  yawAngle += 0.5 * (yawData + get_rotation_data()) * (timer0_of_ms / 1e3);
}

// Interrupt handler for junction detection 
// Parameters: void
// Returns: void
void jct_int_handler()
{
  leftJctDetect = digitalRead(leftJctPin);
  rightJctDetect = digitalRead(rightJctPin);
  tJctDetect = leftJctDetect & rightJctDetect;
  
  turnReady = true;

}

// PID calculator for line following
// Parameters: int valAvg - averaged colour temperature reading, int* correct - pointer to correction variable
// Returns: void
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

// Decrements direction angle for a dummy clockwise rotation pattern
// Parameters: uint8_t* newDirection - pointer to direction variable
// Returns: void
void get_next_turn_dummy_clockwise(uint8_t* newDirection)
{
  //connect to pathfinding logic,
  //get absolute value of next turn
  (*newDirection)--; //int overflow when 0 decremented.
  if (*newDirection == 255) *newDirection = 3;
}

// Increments direction angle for a dummy anticlockwise rotation pattern
// Parameters: uint8_t* newDirection - pointer to direction variable
// Returns: void
void get_next_turn_dummy_anticlockwise(uint8_t* newDirection)
{
  //connect to pathfinding logic,
  //get absolute value of next turn
  (*newDirection) ++;
  if (*newDirection > 3) *newDirection = 0;
}

// Updates direction angle with that of the next node, generated by the pathfinding.
// Parameters: uint8_t* newDirection - pointer to direction variable
// Returns: void
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



// Gets gyroscope data for numerical integration to calculate angle
// Parameters: void
// Returns: float
float get_rotation_data()
{
  float x, y, z;
  if(IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(x, y, z);
  }
  return z;
}

// Gets colour data from the colour sensor and averages by numAvgSamples
// Parameters: void
// Returns: uint16_t
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

// Applies correction to the motor speeds to follow the line
// Parameters: int correction - correction variable
// Returns: void
void pid_motor_regulate(int correction)
{
  leftWheel->setSpeed(constrain((avgMotorSpeed - correction), 0, 255));
  rightWheel->setSpeed(constrain((avgMotorSpeed + correction), 0, 255));
}


void delay_under_pid(uint16_t timeout)
{
  unsigned long time1 = millis();
  while(millis() - time1 < timeout) {
      uint16_t valAvg = get_colour_data();
      calculate_pid(valAvg, &correction);
      pid_motor_regulate(correction);

    }
}

void delay_under_manual(uint16_t timeout, bool reverse = false)
{
  leftWheel->setSpeed(180);
  rightWheel->setSpeed(190);

  //hardcode distance to the pivot point of the robot
  if(!reverse)
  {
    leftWheel->run(FORWARD);
    rightWheel->run(FORWARD);
  }
  
  leftWheel->run(BACKWARD);
  rightWheel->run(BACKWARD);

  delay(timeout); //hardcode time

  leftWheel->setSpeed(0);
  leftWheel->setSpeed(0);
}

void distance_under_pid(uint8_t threshold)
{
  while(int(VL53.getDistance()) > threshold)
  {
    uint16_t valAvg = get_colour_data();
    calculate_pid(valAvg, &correction);
    pid_motor_regulate(correction);
   }
}


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


void set_motor_directions(int* desAng)
{
  int desiredAngle = *desAng;
  //considered refactoring but this is a lot more readable.
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


//god i hate this function if only we had the imu :((((

// Changes the direction of the robot to the specified new direction. Makes initial rotation manually before scanning for new line with the colour sensor
// Parameters: uint8_t* newDirect - pointer to new direction variable
// Returns: void



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
    Kp = 0.00; //WORKAROUND - robot control parameters differ massively when in reverse due to flipped geometry. Kp -> 0 effectively disables PID while utilising existing loop.
    leftWheel->run(BACKWARD);
    rightWheel->run(BACKWARD);
    //do not update currDirect to allow next turn to happen correctly.
  } 
  else
  {
    Kp = 0.2; //when not reversing, ensure Kp is always set to the appropriate value.
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
    
    delay_under_manual(forwardDelayTime); //move forward until pivot hit

    if(desiredAngle == 0)
    {
      LOG_NEWLINE("Going straight!");
      leftWheel->run(FORWARD);
      rightWheel->run(FORWARD);
      //currDirect = *newDirect; direction remains same
      return; //skip rotation if going straight
    }

    set_motor_directions(&desiredAngle);

    leftWheel->setSpeed(155);
    rightWheel->setSpeed(155);

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
    avgMotorSpeed = 225;
  }

}

void start_new_journey(uint8_t* sourceNode, uint8_t* destinationNode, uint8_t* newDirect)
{
  dijkstra(graph, *sourceNode, *destinationNode, bestPath, bestPathDirections, distance);
  returnDirection(graph, bestPath, bestPathDirections);
  currNode = 0;
  *newDirect = bestPathDirections[currNode];
}

void celebrate_and_finish() {
//PID impossible due to geometry of starting region, so we manually drive until we cross the threshold.
      leftWheel->setSpeed(140);
      rightWheel->setSpeed(140);
      delay(800); //calibrate for however long it takes to cross the threshold.
      leftWheel->setSpeed(0);
      rightWheel->setSpeed(0);
      while(1); //We're now finished, infinite loop until power turned off.

      //IF WE HAVE TIME - ADD A LITTLE LIGHT SHOW??
}

#ifdef ARDUINO_UNO
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
