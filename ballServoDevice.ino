//**********************************************************************************************************************
// Francois Barnard
// Created on June 12, 2018
//  
// Servo motor display device utilising laser sensors, a solenoid and steel balls.
//***********************************************************************************************************************

/* Library declarations */
#include <StaticThreadController.h>
#include <Thread.h>
#include <VarSpeedServo.h>
#include <DirectIO.h>

/* Struct declaration*/
struct servo {
  const int SERVO_PIN;
  const int STANDARD_POS;
  const int UNBLOCKED_TIME_INTERVAL;
  
  int servoPos;
  VarSpeedServo &theServo;
};

Output<5> SOLENOID_PIN(LOW);
Input<A0> SMALL_BALL_LASER_PIN(true);
Input<A1> MEDIUM_BALL_LASER_PIN(true);
Input<4> LARGE_BALL_LASER_PIN(true);

namespace {
  VarSpeedServo big;
  VarSpeedServo small;

  const int INTERRUPT_PIN = 2;
  
  const int UNBLOCK_BALL_TUBE_POS = 78;
  const int SMALL_BALL_DISK_POS = 145; //was 51
  const int MEDIUM_BALL_DISK_POS = 113;  //was 79
  const int LARGE_BALL_DISK_POS = 73;  //was 120
  const int SAFETY_DELAY_FOR_BALL = 1250;
  
  // All time values are in milliseconds (ms)
  const int SERVO_RESET_TIME = 100;
  const int SERVO_WAIT_PER_DEGREE = 1;
  const int SENSOR_THREAD_INTERVAL = 5;
  const int SMALL_SERVO_THREAD_INTERVAL = 2450;
  
  // Format: servo variable = {SERVO_PIN, STANDARD_POS, UNBLOCKED_TIME_INTERVAL, servoPos, &theServo};
  servo bigServo = {9, 38, 100, 38, big};   //std pos was 155
  servo smallServo = {6, 61, 150, 61, small};

  volatile bool isSmallBall = false;
  volatile bool isMediumBall = false;
  volatile bool isLargeBall = false;
  volatile bool servoIsAllowedToTurn = false;
  
  unsigned long timeNow = 0;
  int activeBallDiskPos;
}
/* Thread initializations */
Thread *sensorThread = new Thread();
Thread *smallServoThread =  new Thread();

StaticThreadController<2> controller (sensorThread, smallServoThread);

//*************************************************************************************************************
//  initialization
//*************************************************************************************************************
void setup() {
  /* Zeroing the servos */
  big.write(bigServo.STANDARD_POS);
  waitForSetInterval(SERVO_RESET_TIME);
  small.write(smallServo.STANDARD_POS);
  waitForSetInterval(SERVO_RESET_TIME);
  
  big.attach(bigServo.SERVO_PIN);
  small.attach(smallServo.SERVO_PIN);
  waitForSetInterval(SERVO_RESET_TIME);

  sensorThread->onRun(laserCheck);
  sensorThread->setInterval(SENSOR_THREAD_INTERVAL);

  smallServoThread->onRun(getNextBall);
  smallServoThread->setInterval(SMALL_SERVO_THREAD_INTERVAL);

  digitalWrite(INTERRUPT_PIN, HIGH);
  pinMode(INTERRUPT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), stopEverything, CHANGE);
}

//*************************************************************************************************************
//  Main
//*************************************************************************************************************
void loop() {
  controller.run();
  if (servoIsAllowedToTurn) {
    activeBallDiskPos = whichBallIsActive();
    waitForSetInterval(SAFETY_DELAY_FOR_BALL);
    SOLENOID_PIN = HIGH;
    turnServoTo(activeBallDiskPos, bigServo);
    SOLENOID_PIN = LOW;
    waitForSetInterval(bigServo.UNBLOCKED_TIME_INTERVAL);
    turnServoTo(bigServo.STANDARD_POS, bigServo);
    waitForSetInterval(SERVO_RESET_TIME);
        
    isSmallBall = isMediumBall = isLargeBall = servoIsAllowedToTurn = false;  // Reset the conditions
  }
}

//*************************************************************************************************************
//  Other functions
//*************************************************************************************************************
void laserCheck() {
  while (SMALL_BALL_LASER_PIN) {
    isSmallBall = true;
    if (MEDIUM_BALL_LASER_PIN) isMediumBall = true;
    if (LARGE_BALL_LASER_PIN) isLargeBall = true;
  }
  
  servoIsAllowedToTurn = isSmallBall;
}

void getNextBall() {
  turnServoTo(UNBLOCK_BALL_TUBE_POS, smallServo);
  waitForSetInterval(smallServo.UNBLOCKED_TIME_INTERVAL);
  turnServoTo(smallServo.STANDARD_POS, smallServo);
}

void turnServoTo(int whichHolePos, servo &whichServo) {
  if (whichHolePos == whichServo.servoPos) {
    return;
  }

  while (whichServo.servoPos != whichHolePos) {
    (whichHolePos > whichServo.servoPos) ? whichServo.servoPos++ : whichServo.servoPos--;
    whichServo.theServo.write(whichServo.servoPos);
    waitForSetInterval(SERVO_WAIT_PER_DEGREE);
  }
}

/* Alternative to avoid utilising "delay()" */
void waitForSetInterval(int timeInterval) {
  timeNow = millis();
  while (millis() - timeNow < timeInterval);
}

int whichBallIsActive() {
  if (isLargeBall) {
    return LARGE_BALL_DISK_POS;
  }
  else if (isMediumBall) {
    return MEDIUM_BALL_DISK_POS;
  }
  else {
    return SMALL_BALL_DISK_POS;
  }
}

void stopEverything() {
  cli();
  while(1);
}
