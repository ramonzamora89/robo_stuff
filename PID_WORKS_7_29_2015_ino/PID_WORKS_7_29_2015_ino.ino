

#include <QTRSensors.h> //library for Sensor Array
#include <SPI.h> //library for motor encoders
#include <Sabertooth.h>// Libraries for the motor controllers
#include <SoftwareSerial.h>

// *********************
// Define hardware pins
// *********************
#define sabertoothEstop 4 // This is connected to S2 on the motor controllers. 
// When this pin is pulled low the motors will stop.
//this is pin 4 on the arduino
//PIN 4 STOPS ALL MOTORS when pulled low (?)

//Declaring the sensor info
#define NUM_SENSORS   8     // number of sensors used on pololu
#define TIMEOUT       2500  // waits for 2500 milliseconds for sensor outputs to go low
#define EMITTER_PIN   12    // emitter is controlled by digital pin 2 - **Something is wrong here I think***

// sensors 0 through 7 are connected to digital pins 22-36 EVEN, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  22, 24, 26, 28, 30, 32, 34, 36
  },NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

// Declaration of the software serial UART and motor controller objects

SoftwareSerial SWSerial(2, 3); // RX on pin 2 (unused), TX on pin 3 (to S1).
//PIN 2 IS UNUSED IN THAT IT DOES NOT RECEIVE A SIGNAL BACK FROM THE MOTOR CONTROLLERS. THE ARDUINO ONLY TRANSMITS


// Slave Select pins for encoders 1 and 2
// Feel free to reallocate these pins to best suit your circuit
const int slaveSelectEnc1 = 7;
const int slaveSelectEnc2 = 8;

// These hold the current encoder count.
signed long encoder1count = 0;
signed long encoder2count = 0;
//accounting for variation between the two encoders, we'll pray that an average is good enough
signed long encoderCountAvg = 0;


//---------------------------------
//variables for PID
float Kp = 1.7;
float Ki = 0.04;
int offset = 35;
int Tp = 127;
float turnVariable = 0;
int error = 0;
float spin = 0;
float integral = 0;

//motor control vars------------------------
int ySpeed = 0; //initial speed
int idealYSpeed = 180; //straight forward speed.
int turnYSpeed = idealYSpeed - 40; //turning forward speed - slower as to not lose line

Sabertooth frontSaber(128, SWSerial); // Address 128, front motors, 1 = left, 2 = right
Sabertooth rearSaber(129, SWSerial); // Address 129, rear motors, 1 = left, 2 = right
// check that ours is not switched -eb


  //movement variables again...
  int movementDelay = 15; // Delay in between movement commands
  int speedDelta = 64; //if I had to guess this could be 128 so that speeds tops at 255
  int topForwardSpeed = 127 + speedDelta;
  int topBackwardSpeed = 127 - speedDelta;

void initEncoders() {

  // Set slave selects as outputs
  pinMode(slaveSelectEnc1, OUTPUT);
  pinMode(slaveSelectEnc2, OUTPUT);

  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(slaveSelectEnc1, HIGH);
  digitalWrite(slaveSelectEnc2, HIGH);

  SPI.begin();

  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc1, LOW);       // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc1, HIGH);      // Terminate SPI conversation

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(slaveSelectEnc2, LOW);       // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(slaveSelectEnc2, HIGH);      // Terminate SPI conversation
}

long readEncoder(int encoder) {

  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;

  // Read encoder 1
  if (encoder == 1) {
    digitalWrite(slaveSelectEnc1, LOW);     // Begin SPI conversation
    SPI.transfer(0x60);                     // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc1, HIGH);    // Terminate SPI conversation
  }

  // Read encoder 2
  else if (encoder == 2) {
    digitalWrite(slaveSelectEnc2, LOW);     // Begin SPI conversation
    SPI.transfer(0x60);                      // Request count
    count_1 = SPI.transfer(0x00);           // Read highest order byte
    count_2 = SPI.transfer(0x00);
    count_3 = SPI.transfer(0x00);
    count_4 = SPI.transfer(0x00);           // Read lowest order byte
    digitalWrite(slaveSelectEnc2, HIGH);    // Terminate SPI conversation
  }

  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;

  return count_value;
}

void clearEncoderCount() {

  // Set encoder1's data register to 0
  digitalWrite(slaveSelectEnc1, LOW);     // Begin SPI conversation
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc1, HIGH);    // Terminate SPI conversation

  delayMicroseconds(100);  // provides some breathing room between SPI conversations

  // Set encoder1's current data register to center
  digitalWrite(slaveSelectEnc1, LOW);     // Begin SPI conversation
  SPI.transfer(0xE0);
  digitalWrite(slaveSelectEnc1, HIGH);    // Terminate SPI conversation

  // Set encoder2's data register to 0
  digitalWrite(slaveSelectEnc2, LOW);     // Begin SPI conversation
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(slaveSelectEnc2, HIGH);    // Terminate SPI conversation

  delayMicroseconds(100);  // provides some breathing room between SPI conversations

  // Set encoder2's current data register to center
  digitalWrite(slaveSelectEnc2, LOW);     // Begin SPI conversation
  SPI.transfer(0xE0);
  digitalWrite(slaveSelectEnc2, HIGH);    // Terminate SPI conversation

}


//******************************************************************************
// Sets up our serial com, hardware pins, SPI bus for ethernet shield and motor controller.
// RETURNS: Nothing
//******************************************************************************
void setup()
{
  delay(3000);    // Short delay to powerup the motor controllers

  Serial.begin(9600);    // Serial for the debug output
  SWSerial.begin(9600);  // Serial for the motor controllers
  //-eb can we change this to a faster baud rate?
  //BAUD RATE IS THE NUMBER OF SIGNALING EVENTS THAT OCCURS PER SECOND. IN OTHER WORDS, IT IS THE NUMBER OF CHANGES IN BINARY STATE PER SECOND

  frontSaber.autobaud(); // Allows the motor controllers to detect the baud rate
  rearSaber.autobaud();  // we're using and adjust appropriately

  // Initialize GPIO inputs and outputs
  //sets pin 4 to output so we can stop the motors
  pinMode(sabertoothEstop, OUTPUT);

  allStop();		// Make sure all motors are stopped for safety

  //getting the encoders up and running
  initEncoders();       Serial.println("Encoders Initialized...");
  clearEncoderCount();  Serial.println("Encoders Cleared...");

  //variables for calibrating
  int i = 0; // Counters
  int j = 0;
  //int movementDelay = 15; // Delay in between movement commands
  //int speedDelta = 64; //if I had to guess this could be 128 so that speeds tops at 255
  //int topForwardSpeed = 127 + speedDelta;
  //int topBackwardSpeed = 127 - speedDelta;
  
  //unsigned int avgMin = 0;
  //unsigned int avgMax = 0;

  //calibrating
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode

  for (int i = 0; i < 3; i++)  // move back and forth seven times
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
    Serial.println("calibrating: Start offline on robot's left side");

    //    // *************************
    //    // Right and Left Motion
    //    // *************************
  for (int i = 127; i < 167; i++) //why are we using floats here?
    {    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)

      commandMotors(127, i, 127, 1);
    }
    allStop();

    for (int i = 127; i > 87;  i--)
    {    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)

      commandMotors(127, i, 127, 1);
    }
    allStop(); 
        qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)

    
  }//for
  digitalWrite(13,LOW); // turn on Arduino's LED to indicate we are in calibration mode

    
  //printing calibration figures
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print('\t');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++){
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print('\t');
  }
  
  Serial.println();
  //figuring out initial position
  //unsigned int position = qtrrc.readLine(sensorValues);
  //Serial.println(position);
  
  Serial.println("Running");
  Serial.println();
}

void loop() {


  // read calibrated sensor values and obtain a measure of the line position from 0 to 7000
  unsigned int position = qtrrc.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  Serial.println(position); // comment this line out if you are using raw values
/** SPEEDOMETER -- DOES NOT WORK YET
//calculate the encoder values
    encoder1count = readEncoder(1);
    encoder2count = readEncoder(2);
  int encoderAvg = ((encoder1count*-1)+encoder2count)/2;
  
  
  float feetTravelled;  
  float ftPerSec;
  //calculate speed... 1920 encoder units= 1 ft
  feetTravelled = encoderAvg/1920;
  ftPerSec = feetTravelled/(millis()*1000);
  Serial.println(ftPerSec);
  **/

  //PID SHIT
  int location = position/100; //getting the position and error down to a managable number
  error = offset - location; //calculate the error

  if (error < 5 && error > -5) { //reset the integral when on the line
    integral = 0;
  } else {
    integral = integral + error;   //calulate the integral
 }

//calculate the angle to get back to line (spin)
  turnVariable = Kp * error + (Ki * integral); //get the PI-derived turnVariable
  spin = Tp + int(turnVariable); //adjust the spin based on Tp(127) and the turnVariable

//calcuulate the speed based on the amount we're turning
  if(abs(error)> 16){ //if turning sharply (abs(in either direction)), SLOW down
    ySpeed = turnYSpeed;
  } else {
    ySpeed = idealYSpeed;
  }
  

//MOVEMENT************
  commandMotors(ySpeed, 127, spin, 1); //move it or lose it

Serial.print("error: "); Serial.println(error);
  //Serial.print("spin: "); Serial.println(spin);
  //Serial.print("integral: "); Serial.println(integral);
  //Serial.print("turnVariable: "); Serial.println(turnVariable);
  //Serial.print("ftPerSec: "); Serial.println(ftPerSec);

delay(250);

}//end loop


//******************************************************************************
// Sets the speed of all motor controllers to zero and sets all ESTOPs
// RETURNS: NONE
//******************************************************************************
void allStop()
{
  digitalWrite(sabertoothEstop, LOW);

  frontSaber.motor(1, 0);
  frontSaber.motor(2, 0);
  rearSaber.motor(1, 0);
  rearSaber.motor(2, 0);
}



//******************************************************************************
//commandMotors
// Processes all motor commands.
// The function expects three values;
//    yAxis is our forward and back movement
//    xAxis is our left and right movement
//    spin is our third axis
//    mode is the type of movement we're sending (tank versus vectoring)
//
// To understand the input of this function you need to envision a grid with 0,0
// in the bottom left, 127,127 in the center and 255,255 in the top right. The location
// of the data point passed into this function relative to the 127,127 center point
// is the direction that the robot is commanded to vector
//
// RETURNS: NONE
//******************************************************************************
void commandMotors(int yAxis, int xAxis, int spin, int mode)
{
  // Initialize our local variables
  int leftFrontPower = 0;
  int leftRearPower = 0;
  int rightFrontPower = 0;
  int rightRearPower = 0;
  int maxMotorPower = 0;
  double tempScale = 0;

  // Motor Constants
  int motorValueMax = 255;
  int motorValueMin = 0;
  int motorZero = 127;

  // Bound our incoming data to a safe and expected range
  if (yAxis > motorValueMax) {
    yAxis = motorValueMax;
  }
  else  if (yAxis < motorValueMin) {
    yAxis = motorValueMin;
  }
  if (xAxis > motorValueMax) {
    xAxis = motorValueMax;
  }
  else  if (xAxis < motorValueMin) {
    xAxis = motorValueMin;
  }
  if (spin > motorValueMax) {
    spin = motorValueMax;
  }
  else  if (spin < motorValueMin) {
    spin = motorValueMin;
  }

  // Shift incoming data to straddle 0
  yAxis = yAxis - 127;
  xAxis = xAxis - 127;
  spin = spin - 127;

  // A mode value of 1 passed into this function changes the motor mixing to
  // vectoring mode
  if (mode == 1)
  {
    // *************************
    // Front and Back Motion
    //all wheels equal power forward
    // *************************
    leftFrontPower = leftFrontPower + yAxis;
    leftRearPower = leftRearPower + yAxis;
    rightFrontPower = rightFrontPower + yAxis;
    rightRearPower = rightRearPower + yAxis;
    //    Serial.println("y calculation");
    //    Serial.print("LFP: "); Serial.println(leftFrontPower);
    //    Serial.print("LRP: "); Serial.println(leftRearPower);
    //      Serial.print("RFP: "); Serial.println(rightFrontPower);
    //        Serial.print("RRP: "); Serial.println(rightRearPower);
    // *************************
    // Left and Right Motion
    //front and rear wheels opposite sign
    //left and right opposite sign
    // *************************
    leftFrontPower = leftFrontPower + xAxis;
    leftRearPower = leftRearPower - xAxis;
    rightFrontPower = rightFrontPower - xAxis;
    rightRearPower = rightRearPower + xAxis;
    //        Serial.println("x calculation");
    //    Serial.print("LFP: "); Serial.println(leftFrontPower);
    //    Serial.print("LRP: "); Serial.println(leftRearPower);
    //      Serial.print("RFP: "); Serial.println(rightFrontPower);
    //        Serial.print("RRP: "); Serial.println(rightRearPower);

    // *************************
    // Spin
    // *************************
    leftFrontPower = leftFrontPower + spin;
    leftRearPower = leftRearPower + spin;
    rightFrontPower = rightFrontPower - spin;
    rightRearPower = rightRearPower - spin;
    //        Serial.println("spin calculation");
    //    Serial.print("LFP: "); Serial.println(leftFrontPower);
    //    Serial.print("LRP: "); Serial.println(leftRearPower);
    //      Serial.print("RFP: "); Serial.println(rightFrontPower);
    //        Serial.print("RRP: "); Serial.println(rightRearPower);

    // After our mixing above our motor powers are most likely going to exceed
    // our maximum values. We need to find our maximum and scale everything down
    // to values that our motor controller can understand
    maxMotorPower = max(abs(leftFrontPower), abs(leftRearPower));
    maxMotorPower = max(maxMotorPower, abs(rightFrontPower));
    maxMotorPower = max(maxMotorPower, abs(rightRearPower));

    // Scale down by the maximum value if we exceed 127
    if (maxMotorPower > 127)
    {
      tempScale = (double)127 / (double)maxMotorPower;
      leftFrontPower = tempScale * (double)leftFrontPower;
      leftRearPower = tempScale * (double)leftRearPower;
      rightFrontPower = tempScale * (double)rightFrontPower;
      rightRearPower = tempScale * (double)rightRearPower;
    }

    // Cleans up our output data
    leftFrontPower = boundAndDeadband(leftFrontPower);
    leftRearPower = boundAndDeadband(leftRearPower);
    rightFrontPower = boundAndDeadband(rightFrontPower);
    rightRearPower = boundAndDeadband(rightRearPower);

    // Raises the ESTOP lines before commanding the motors
    digitalWrite(sabertoothEstop, HIGH);

    //        Serial.println("FInal Values");
    //    Serial.print("LFP: "); Serial.println(leftFrontPower);
    //    Serial.print("LRP: "); Serial.println(leftRearPower);
    //      Serial.print("RFP: "); Serial.println(rightFrontPower);
    //        Serial.print("RRP: "); Serial.println(rightRearPower);
    // Applies our calculated and bounded values to our drive motor controllers
    frontSaber.motor(1, rightFrontPower);
    frontSaber.motor(2, leftFrontPower);
    rearSaber.motor(1, rightRearPower);
    rearSaber.motor(2, leftRearPower);
  }

  // If the mode value is not "1" then we are in tank mode
  else
  {
    // Applies our calculated and bounded values to our drive motor controllers
    frontSaber.drive(yAxis);
    frontSaber.turn(xAxis);

    rearSaber.drive(yAxis);
    rearSaber.turn(xAxis);
  }
}

//******************************************************************************
// Cleans up our values for the motor controllers
// The motor controllers only accept a value range of -127 to 127. We also apply
// a deadband so the robot doesn't drift when idle
//
// RETURNS: Cleaned up value
//******************************************************************************
int boundAndDeadband (int inputValue)
{
  if (inputValue < -127)  {
    inputValue = -127;
  }
  if (inputValue > 127)   {
    inputValue = 127;
  }
  if ((inputValue < 5) && (inputValue > -5)) {
    inputValue = 0;
  }

  return inputValue;
}
