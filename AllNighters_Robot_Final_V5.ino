/*
   AllNighters_Robot_Final_V4.ino

   Calvin Weaver, Hussein Alawami, Jason Duncan
   10/27/2016

   This is the robot code for our final ECE160 project. This program (chronologically)
   waits for a button input (read from a wireless transceiver card), executes a flexible
   autonomous strategy, waits for another button input, then reads values from the wireless
   transceiver card in a loop (for the driver controlled period).

   During autonomous, two indicator LEDs are toggled based on if the robot is currently
   reading from one of the sensors. Additionally, when the robot is using the tracking function
   during tele-operated mode the blue LED is toggled.

*/

#include<SharpIR.h>//import the required libraries
#include<NewPing.h>
#include<Servo.h>

#include<SPI.h>
#include<nRF24L01.h>
#include<RF24.h>

#define RADIO_CHANNEL 75//the radio communication channel
#define RADIO_DELAY_VALUE 100//the delay value for optimal radio communication (read)

#define PIN_AVOIDANCE_SENSOR 5

#define PIN_SERVO_LEFT A0//the pin the left wheel servo is connected to
#define PIN_SERVO_RIGHT A1//the pin the right wheel servo is connected to
#define PIN_SERVO_GRIP A2//the pin the gripper servo is connected to

#define PIN_RADIO_CE 9//the pin the radio ce pin is connected to
#define PIN_RADIO_CSN 10//the pin the radio csn pin is connected to

#define PIN_IR_LEFT A4//the pin the left IR sensor is connected to
#define PIN_IR_RIGHT A5//the pin the right IR sensor is connected to

#define PIN_ULTRA_LEFT 3//the pin the left ultrasonic sensor is connected to
#define PIN_ULTRA_RIGHT 6//the pin the right ultrasonic sensor is connected to

#define PIN_LED_LOCKON 7//the pin the lockon indicator LED is connected to
#define PIN_LED_ULTRA 8//the pin the ultrasonic indicator LED is connected to

SharpIR irLeft(PIN_IR_LEFT, 25, 93, 1080);//define the sensors
SharpIR irRight(PIN_IR_RIGHT, 25, 93, 1080);
NewPing ultraLeft(PIN_ULTRA_LEFT, PIN_ULTRA_LEFT);
NewPing ultraRight(PIN_ULTRA_RIGHT, PIN_ULTRA_RIGHT);
Servo servoLeft, servoRight, servoGrip;//define the servos

const uint64_t pipe = 0xE8E8F0F0E1LL;//value used by the radio
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);

void setup() {
  pinMode(PIN_IR_LEFT, INPUT);//initialize the IR sensors
  pinMode(PIN_IR_RIGHT, INPUT);

  pinMode(PIN_AVOIDANCE_SENSOR, INPUT);//initialize the obstacle avoidance sensor (for autonomous)

  pinMode(PIN_LED_LOCKON, OUTPUT);//initialize the indicator LEDS
  pinMode(PIN_LED_ULTRA, OUTPUT);

  servoLeft.attach(PIN_SERVO_LEFT);//initialize the servos
  servoRight.attach(PIN_SERVO_RIGHT);
  servoGrip.attach(PIN_SERVO_GRIP);

  radio.begin();//open radio communication
  radio.setChannel(RADIO_CHANNEL);
  radio.openReadingPipe(1, pipe);//set radio to read mode
  radio.startListening();

  Serial.begin(9600);//open serial communication
}

int toRead[5];//array to manage the read data
int valueLeft, valueRight, valueGrip;//the current servo values
bool grip = false;//if the robot is gripping
bool toggledGrip = false;//if grip button press has been checked this update

int avoidValue;//the value read by the obstacle avoidance sensor

int driveStage = 0;//the current drive stage (0 = pre-autonomous, 1 = autonomous, 2 = tele-operated)
bool toggledDriveStage = false;//if drive state button press has been checked this update

int irLeftRead, irRightRead;//the values read from the IR sensors
bool lockOn = false;//if the robot is currently in lockon mode (used for indicator LED)
bool ultra = false;//if the robot is currently in ultrasonic mode (used for indicator LED)

void loop() {
  if (radio.available()){//read data from the wireless transceiver
    bool done = false;
    while (!done) done = radio.read(toRead, sizeof(toRead));
  }
  printReadData();//print the data read to serial

  avoidValue = digitalRead(PIN_AVOIDANCE_SENSOR);//read the obstacle avoidance sensor value

  //run the current drive stage
  if (toRead[3]) {
    if (!toggledDriveStage) {
      toggledDriveStage = true;
      if (driveStage < 2) driveStage++;//progress the drive stage if the appropriate button is pressed
    }
  } else toggledDriveStage = false;
  if (driveStage == 0) {
    tickIdle();//call the idle drive stage function
  }
  else if (driveStage == 1) {
    //call the autonomous drive stage function
    tickAutonomous();//uncomment to execute the primary autonomous strategy
    //tickAutonomous2();//uncomment to execute the alternate autonomous strategy
  }
  else if (driveStage == 2) {
    tickTeleop();//call the tele-operated drive stage function
  }

  //set the servo values to their designated variables
  valueLeft = constrain(map(valueLeft, -100, 100, 1300, 1700), 1300, 1700);
  valueRight = constrain(map(valueRight, -100, 100, 1700, 1300), 1300, 1700);
  servoLeft.writeMicroseconds(valueLeft);
  servoRight.writeMicroseconds(valueRight);
  servoGrip.write(grip ? 120 : 30);

  //set the lockon LED to its current value
  digitalWrite(PIN_LED_LOCKON, lockOn);
  lockOn = false;

  //set the ultra LED to its current value
  digitalWrite(PIN_LED_ULTRA, ultra);
  ultra = false;

  delay(RADIO_DELAY_VALUE);//delay a specified amount so the wireless transceiver doesn't lag
}

//the function for pre-autonomous
void tickIdle() {
  grip = false;//disable the gripper
  valueLeft = 0;
  valueRight = 0;
}

//the current stage of autonomous mode, this value is changed so we can
//execute multiple commands in sequence
int autoStage = 0;

int autoProgress = 0;//the progress of each autonomous stage, used for timings

//the function for the primary autonomous strategy
void tickAutonomous() {
  if (autoStage == 0) {
    if (autoProgress < 2 * (1000 / RADIO_DELAY_VALUE)) { //FORWARD
      grip = true;
      valueLeft = 100;
      valueRight = 100;
    } else autoNextStage();
  }
  else if (autoStage == 1) {
    if (autoProgress < 1.5 * (1000 / RADIO_DELAY_VALUE)) { //TRACKING
      grip = false;
      valueLeft = 20;
      valueRight = 20;
      tickLockOn();
    } else autoNextStage();
  }
  else if (autoStage == 2) {
    if (autoProgress < 0.5 * (1000 / RADIO_DELAY_VALUE)) { //GRAB WAIT
      grip = true;
      valueLeft = 0;
      valueRight = 0;
    } else autoNextStage();
  }
  else if (autoStage == 3) {
    if (autoProgress < 3.7 * (1000 / RADIO_DELAY_VALUE)) { //REVERSE
      grip = true;
      valueLeft = -100;
      valueRight = -100;
    } else autoNextStage();
  }
  else if (autoStage == 4) {
    if (autoProgress < 0.8 * (1000 / RADIO_DELAY_VALUE)) { //TURN BACK WALL
      grip = true;
      valueLeft = 70;
      valueRight = -70;
    } else autoNextStage();
  }
  else if (autoStage == 5) {
    if (autoProgress < 0.8 * (1000 / RADIO_DELAY_VALUE)) { //ULTRASONIC
      grip = true;
      valueLeft = 30 + (tickUltra(3, ultraRight) ? 20 : -30);
      valueRight = 30;
    } else autoNextStage();
  }
  else if (autoStage == 6) {
                                                           //IR WALL
    grip = true;
    valueLeft = 30;
    valueRight = 30;
    if (avoidValue == HIGH || tickUltra(10)) autoNextStage();
  }
  else if (autoStage == 7) {
    if (autoProgress < 0.6 * (1000 / RADIO_DELAY_VALUE)) { //DROP NUDGE
      grip = false;
      valueLeft = 30;
      valueRight = -30;
    } else autoNextStage();
  }
  else if (autoStage == 8) {
    if (autoProgress < 1.4 * (1000 / RADIO_DELAY_VALUE)) { //TURN AWAY
      grip = false;
      valueLeft = -30;
      valueRight = 30;
    } else autoNextStage();
  }
  else if (autoStage == 9) {
    if (autoProgress < 0.6 * (1000 / RADIO_DELAY_VALUE)) { //SLOW FORWARD
      grip = false;
      valueLeft = 20;
      valueRight = 20;
    } else autoNextStage();
  }
  else if (autoStage == 10) {
    if (autoProgress < 2.0 * (1000 / RADIO_DELAY_VALUE)) { //TRACKING
      grip = false;
      valueLeft = 20;
      valueRight = 20;
      tickLockOn();
    } else autoNextStage();
  }
  else if (autoStage == 11) {
    if (autoProgress < 0.5 * (1000 / RADIO_DELAY_VALUE)) { //GRAB WAIT
      grip = true;
      valueLeft = 0;
      valueRight = 0;
    } else autoNextStage();
  }
  else if (autoStage == 12) {
    if (autoProgress < 1.6 * (1000 / RADIO_DELAY_VALUE)) { //180
      grip = true;
      valueLeft = 30;
      valueRight = -30;
    } else autoNextStage();
  }
  else if (autoStage == 13) {
    if (autoProgress < 3.5 * (1000 / RADIO_DELAY_VALUE)) { //FORWARD
      grip = true;//wait value was 3.0 before
      valueLeft = 30;
      valueRight = 30;
    if (avoidValue == HIGH) autoNextStage();
    }else autoNextStage();
  }
  else {
    grip = false;
    valueLeft = 0;
    valueRight = 0;
  }
  autoProgress++;
}

//the function for the alternate autonomous strategy
void tickAutonomous2() {
  if (autoStage == 0) {
    if (autoProgress < 2 * (1000 / RADIO_DELAY_VALUE)) { //FORWARD
      grip = true;
      valueLeft = 100;
      valueRight = 100;
    } else autoNextStage();
  }
  else if (autoStage == 1) {
    if (autoProgress < 1.5 * (1000 / RADIO_DELAY_VALUE)) { //TRACKING
      grip = false;
      valueLeft = 20;
      valueRight = 20;
      tickLockOn();
    } else autoNextStage();
  }
  else if (autoStage == 2) {
    if (autoProgress < 0.5 * (1000 / RADIO_DELAY_VALUE)) { //GRAB WAIT
      grip = true;
      valueLeft = 0;
      valueRight = 0;
    } else autoNextStage();
  }
  else if (autoStage == 3) {
    if (autoProgress < 2.715 * (1000 / RADIO_DELAY_VALUE)) { //360
      grip = true;
      valueLeft = 50;
      valueRight = -50;
    } else autoNextStage();
  }
  else if (autoStage == 4) {
    if (autoProgress < 3.7 * (1000 / RADIO_DELAY_VALUE)) { //REVERSE
      grip = true;
      valueLeft = -100;
      valueRight = -100;
    } else autoNextStage();
  }
  else if (autoStage == 5) {
    if (autoProgress < 0.8 * (1000 / RADIO_DELAY_VALUE)) { //TURN BACK WALL
      grip = true;
      valueLeft = 70;
      valueRight = -70;
    } else autoNextStage();
  }
  else if (autoStage == 6) {
    if (autoProgress < 0.8 * (1000 / RADIO_DELAY_VALUE)) { //ULTRASONIC
      grip = true;
      valueLeft = 30 + (tickUltra(3, ultraRight) ? 20 : -30);
      valueRight = 30;
    } else autoNextStage();
  }
  else if (autoStage == 7) {
    //IR WALL
    grip = true;
    valueLeft = 30;
    valueRight = 30;
    if (tickUltra(12)) autoNextStage();
    if (autoProgress < 0.6 * (1000 / RADIO_DELAY_VALUE)) { //DROP NUDGE
      grip = false;
      valueLeft = 30;
      valueRight = -30;
    } else autoNextStage();
  }
  else if (autoStage == 9) {
    if (autoProgress < 1.4 * (1000 / RADIO_DELAY_VALUE)) { //TURN AWAY
      grip = false;
      valueLeft = -30;
      valueRight = 30;
    } else autoNextStage();
  }
  else if (autoStage == 10) {
    if (autoProgress < 0.6 * (1000 / RADIO_DELAY_VALUE)) { //SLOW FORWARD
      grip = false;
      valueLeft = 20;
      valueRight = 20;
    } else autoNextStage();
  }
  else if (autoStage == 11) {
    if (autoProgress < 2.0 * (1000 / RADIO_DELAY_VALUE)) { //TRACKING
      grip = false;
      valueLeft = 20;
      valueRight = 20;
      tickLockOn();
    } else autoNextStage();
  }
  else if (autoStage == 12) {
    if (autoProgress < 0.5 * (1000 / RADIO_DELAY_VALUE)) { //GRAB WAIT
      grip = true;
      valueLeft = 0;
      valueRight = 0;
    } else autoNextStage();
  }
  else if (autoStage == 13) {
    if (autoProgress < 1.6 * (1000 / RADIO_DELAY_VALUE)) { //180
      grip = true;
      valueLeft = 30;
      valueRight = -30;
    } else autoNextStage();
  }
  else if (autoStage == 14) {
    if (autoProgress < 3.0 * (1000 / RADIO_DELAY_VALUE)) { //FORWARD
      grip = true;
      valueLeft = 30;
      valueRight = 30;
    } else autoNextStage();
  }
  else {
    grip = false;
    valueLeft = 0;
    valueRight = 0;
  }
  autoProgress++;
}

//progress the autonomous stages
void autoNextStage() {
  autoStage++;
  autoProgress = 0;
}

//the function for running tele-operated mode
void tickTeleop() {
  valueLeft = 512 - toRead[0];
  valueRight = 512 - toRead[1];

  valueGrip = toRead[2];

  valueLeft = map(valueLeft, -512, 512, -100, 100);
  valueRight = map(valueRight, -512, 512, -100, 100);

  if (valueGrip) {
    if (!toggledGrip) {
      toggledGrip = true;
      grip = !grip;
    }
  } else toggledGrip = false;

  if (toRead[4] == 1) {
    tickLockOn();
    valueLeft += 20;
    valueRight += 20;
  }
}

//when this function is called the robot will signal with the blue LED indicator and move the opposing wheel based on
//if there is an object in front of one of the IR sensors
void tickLockOn() {
  irLeftRead = irLeft.distance();//read the IR sensor values
  irRightRead = irRight.distance();

  Serial.print(irLeftRead);//print read values to serial
  Serial.print(":");
  Serial.println(irRightRead);

  //if the distance meets a certain margin and one sensor is significantly more than the other,
  //then increase the speed of the opposite servo based on a mapping of the distance that the
  //object is away from the sensor
  //
  //after much trial and error we determined that this was more effective at tracking objects 
  //than without the mapping
  if (irLeftRead < irRightRead - 2 && irLeftRead < 18) valueRight += constrain(map(irLeftRead, 10, 20, 20, 50), 20, 50);
  else if (irRightRead < irLeftRead - 2 && irRightRead < 18) valueLeft += constrain(map(irRightRead, 10, 20, 20, 50), 20, 50);

  lockOn = true;
}

//check if the ultrasonic sensor reads a value closer than the specified value, also enable the LED indicator
bool tickUltra(int dist, NewPing sensor) {
  ultra = true;
  return sensor.ping_in() < dist;
}

//check if the IR sensors read a value closer than the specified value, also enable the LED indicator
bool tickUltra(int dist) {
  lockOn = true;
  irLeftRead = irLeft.distance();
  irRightRead = irRight.distance();
  Serial.print(irLeftRead);
  Serial.print(":");
  Serial.println(irRightRead);
  return max(irLeftRead, irRightRead) < dist;
}

//print the data read from the wireless transceiver to serial
void printReadData() {
  Serial.print("R:");//print the read data to serial
  Serial.print(toRead[0]);
  Serial.print(":");
  Serial.print(toRead[1]);
  Serial.print(":");
  Serial.print(toRead[2]);
  Serial.print(":");
  Serial.print(toRead[3]);
  Serial.print(":");
  Serial.println(toRead[4]);
}

