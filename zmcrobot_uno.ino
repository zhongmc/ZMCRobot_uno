#include "ZMCRobotROS.h"
#include <Arduino.h>

#include "IRSensor.h"
#include "Robot.h"

#include "DriveSupervisor.h"
#include "BlinkLed.h"
#include "MyMPU6050.h"

//#include "IRReceiver.h"
//#include "Kalman.h"
//#include "MyKey.h"
//#include "MyMenu.h"
#include "IMU.h"
#include <MsTimer2.h>


void startIMU();
bool isBatteryLow();
void sendRobotStateValue(double x, double y, double theta, double irDistance[5], double voltage );
void sendRobotStateValue(byte stateType, Position pos, double irDistance[5], double voltage);


#define VOLT_IN_PIN A0
#define ULTRASONIC_ECHO 11
#define ULTRASONIC_TRIG 10

//states
#define STATE_IDLE 0
#define STATE_MENU 1
#define STATE_DRIVE 2

#define STATE_GOTOGOAL 3
#define STATE_BALANCE 4
#define STATE_CALIBRATE 5

#define START_KEY 0
#define LEFT_KEY 1
#define RIGHT_KEY 2
#define RET_KEY 3

#define START_KEY_PIN 11
#define LEFT_KEY_PIN 10
#define RIGHT_KEY_PIN 9
#define RET_KEY_PIN 8

byte currentState = STATE_IDLE;

//MyKey myKey;

IMU mIMU;
//Supervisor supervisor;
DriveSupervisor driveSupervisor;
BlinkLed blinkLed;
//BlinkMatrixLed blinkLed;
bool bExecDrive, bExecGTG;

//PureBalanceSupervisor balanceSupervisor;
//BlinkMatrixLed blinkLed;
// IRReceiver irRecv(12);
// IRCode ircode;

Position pos;

long trigTime, echoTime;
double ultrasonicDistance;

bool doCheckBattleVoltage = true;
bool openDebug = false;
byte settingsReqQueue[8];
short queueLen = 0;

extern long count1, count2;
extern int comDataCount;

unsigned long millisPrevKey, millisPrev;

IRSensor irSensor(GP2Y0A41);

static double batteryVoltage;  // Measured battery level
static uint8_t batteryCounter; // Counter used to check if it should check the battery level

double irDistance[5];

void setup()
{

  Serial.begin(115200);
  delay(100);

  // // start the IMU and filter
  // CurieIMU.begin();
  // CurieIMU.setGyroRate(GYRO_RATE);
  // CurieIMU.setAccelerometerRate(GYRO_RATE);
  // // Set the accelerometer range to 2G
  // CurieIMU.setAccelerometerRange(2);
  // // Set the gyroscope range to 250 degrees/second
  // CurieIMU.setGyroRange(250);
  // filter.begin(GYRO_RATE);

  initMotor();

  // initialize variables to pace updates to correct rate
  //  microsPerReading = 1000000 / GYRO_RATE;  //25
  //  microsPrevious = micros();

  initBluetooth();
  //  Serial.println(F("Initialze MENU..."));
  //
  //void MyMenu::setKeyId(byte stKey, byte rtKey, byte lKey, byte rKey)
  //  menu.setKeyId(START_KEY, RET_KEY,  LEFT_KEY, RIGHT_KEY);
  //  initMenu();

  //  const int oneSecInUsec = 1000000;   // A second in mirco second unit.
  // time = oneSecInUsec / 100; // time is used to toggle the LED is divided by i
  //  CurieTimerOne.start(oneSecInUsec / GYRO_RATE, &timedBlinkIsr);  // set timer and callback

  //  Serial.println(sizeof(long));

  pinMode(13, OUTPUT);

  comDataCount = 0;
  count1 = 0;
  count2 = 0;

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(ULTRASONIC_ECHO, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  // ultr sound echo intterupt
  // attachInterrupt(digitalPinToInterrupt(ULTRASONIC_ECHO), UltrasonicEcho, CHANGE);

  SETTINGS mSettings;
  mSettings.sType = 0;

  mSettings.sType = 0;
  mSettings.kp = 5;
  mSettings.ki = 0.01;
  mSettings.kd = 0.05;

  mSettings.max_rpm = 200;
  mSettings.min_rpm = 40; //45

  mSettings.atObstacle = 0.25; //0.15
  mSettings.unsafe = 0.1;
  mSettings.dfw = 0.2;      //0.25
  mSettings.velocity = 0.2; //0.3
  driveSupervisor.init();
  bExecDrive = false;
  bExecGTG = false;

  blinkLed.init();

  mIMU.init();

  // bCount = 0;
  startIMU();

  millisPrevKey = millis();
  millisPrev = millisPrevKey; //millis();

  // driveSupervisor.setRobotPosition(0, 0, PI * mIMU.getYaw() / 180.0);
}

//
int driveCycleCount = 0;

void loop()
{

  checkSerialData();
  blinkLed.beSureToBlink();
  //ble cmd process
  // processSetingsRequire();
  //ultrasonic process
  // processUltrasonic();

  // if (irRecv.readIRCode(ircode) == 0)
  // {
  //   if (ircode.code_h + ircode.code_l == 255)
  //     irRemoteProcess(ircode.code_l);
  // }

  if (bExecDrive == true)
  {
    bExecDrive = false;
    driveSupervisor.execute(readLeftEncoder(), readRightEncoder(), mIMU.getGyro(2), 0.05); //1/20
  }

  // if (bExecGTG)
  // {
  //   bExecGTG = false;
  //   supervisor.execute(readLeftEncoder(), readRightEncoder(), 0.05);
  // }

  unsigned long millisNow = millis();
  if (millisNow - millisPrev >= 100)
  {

    millisPrev = millisNow;

    if (currentState == STATE_DRIVE)
    {
      driveSupervisor.getIRDistances(irDistance);
      pos = driveSupervisor.getRobotPosition();
      sendRobotStateValue(1, pos, irDistance, batteryVoltage);
    }
    else
    {
      driveSupervisor.readIRDistances(irDistance);
      pos = driveSupervisor.getRobotPosition();
      pos.theta = (PI * mIMU.getYaw()) / 180.0;
      sendRobotStateValue(1, pos, irDistance, batteryVoltage);
    }

    batteryCounter++;
    if (batteryCounter >= 2)
    { // Measure battery every 1s
      batteryCounter = 0;
      if (isBatteryLow())
      {
        if (doCheckBattleVoltage && isBatteryLow()) //read again
        {
          if (currentState != STATE_IDLE)
          {
            Serial.println("Bat L...");
            stopAndReset();
            currentState = STATE_IDLE;
          }
          blinkLed.slowBlink();
        }
      }
    }
  }
}

void setGoal(double x, double y, int theta, double v)
{
  // count1 = 0;
  // count2 = 0;
}

void startGoToGoal()
{
  if (currentState == STATE_GOTOGOAL)
    return;

  stopRobot(); //stop currentState
}

void ResetRobot()
{
  driveSupervisor.resetRobot();
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
}

void startDrive()
{
  // if ( currentState >= 2 )
  //   return;

  Serial.println("Start DRV!");
  currentState = STATE_DRIVE;

  // to test set goal y to 0
  driveSupervisor.reset(readLeftEncoder(), readRightEncoder());
  // Position pos = driveSupervisor.getRobotPosition();
  // pos.theta = (PI * mIMU.getYaw()) / 180.0;
  // driveSupervisor.setRobotPosition(pos.x, pos.y, pos.theta);
  //   currentState = STATE_DRIVE;

  // const int oneSecInUsec = 1000000; // A second in mirco second unit.
  // // time = oneSecInUsec / 100; // time is used to toggle the LED is divided by i
  // CurieTimerOne.start(oneSecInUsec / 20, &driveIsr); // set timer and callback
}

void driveIsr()
{
  bExecDrive = true;
  // driveSupervisor.execute(readLeftEncoder(), readRightEncoder(), 0.05); //1/20
}


void IMUIsr()
{

  mIMU.readIMU(0);           //1/GYRO_RATE
  mIMU.calculateAttitute(0); //1/GYRO_RATE

  if (currentState == STATE_DRIVE)
  {
    driveCycleCount++;
    if (driveCycleCount >= 5) //GYRO_RATE / DriveRate (200/20 ) GYRO_RATE 100
    {
      driveCycleCount = 0;
      bExecDrive = true;
    }
  }
}

void startIMU()
{
  const int oneSecInUsec = 1000000; // A second in mirco second unit.
  // time = oneSecInUsec / 100; // time is used to toggle the LED is divided by i
 
  MsTimer2::set(1000/GYRO_RATE, IMUIsr); // ms period
  MsTimer2::start();

  // CurieTimerOne.start(oneSecInUsec / GYRO_RATE, &IMUIsr); // set timer and callback
}

void SetSimulateMode(bool val)
{
  driveSupervisor.mSimulateMode = val;
  if (val)
  {
    doCheckBattleVoltage = false;
    Serial.println("simulate.");
  }
  else
  {
    doCheckBattleVoltage = true;
    Serial.println("Close simulate!");
  }
}

void SetIgnoreObstacle(bool igm)
{
  Serial.print("ignore obs:");
  Serial.println(igm);
  driveSupervisor.mIgnoreObstacle = igm;
}

void stopRobot()
{
  blinkLed.normalBlink();
  // CurieTimerOne.kill();
  currentState = STATE_IDLE;
  stopAndReset();
}

/*
void irRemoteProcess(int code)
{
  if (code == 28) //OK key
  {
    if (currentState == STATE_BALANCE)
    {
      if (drive_w != 0)
      {
        drive_w = 0;
        balanceSupervisor.setGoal(drive_v, drive_w);
      }
      else if (drive_v != 0)
      {
        drive_v = 0;
        balanceSupervisor.setGoal(drive_v, drive_w);
      }
      else
      {
        stopBalance();
        blinkLed.normalBlink();
        return;
      }

      if (drive_v == 0 && drive_w == 0)
      {
        balanceSupervisor.stopDrive();
        blinkLed.balanceBlink();
      }
    }
    else
    {
      startBalance();
    }
    return;
  }

  if (code == 22) //X
  {
    balanceSupervisor.setBeSpeedLoop(true);
    Serial.println("speed loop.");
    return;
  }

  if (code == 13) //#
  {
    balanceSupervisor.setBeThetaLoop(true);
    Serial.println("turn loop.");
    return;
  }

  if (currentState != STATE_BALANCE) //ignore speed ctrl
    return;

  bool drv = false;

  if (code == 24) //up
  {
    if (drive_v == 0)
      drive_v = 0.1;
    else
    {
      drive_v = drive_v + 0.02;
      if (drive_v > 0.3)
        drive_v = 0.3;
    }

    drv = true;
  }
  else if (code == 82) //down
  {
    if (drive_v == 0)
    {
      drive_v = -0.1;
    }
    else
    {
      drive_v = drive_v - 0.02;
      if (drive_v < -0.3)
        drive_v = -0.3;
    }

    drv = true;
  }
  else if (code == 8) //LEFT
  {
    drive_w = drive_w + 0.1;
    if (drive_w > 1.2)
    {
      drive_w = 1.2;
    }
    drv = true;
  }
  else if (code == 90)
  {
    drive_w = drive_w - 0.1;
    if (drive_w < -1.2)
    {
      drive_w = -1.2;
    }
    drv = true;
  }

  if (drv == true)
  {
    balanceSupervisor.setGoal(drive_v, drive_w);
    blinkLed.runingBlink();
  }
  if (drive_v == 0 && drive_w == 0)
  {
    balanceSupervisor.stopDrive();
    blinkLed.balanceBlink();
  }
}
*/

void setDriveGoal(double v, double w)
{
  if (currentState == STATE_DRIVE)
  {
    if (abs(v) < 0.001 && abs(w) < 0.001) //stop
    {
      stopRobot();
      Serial.println("Stop drv!");
    }
    // else
    driveSupervisor.setGoal(v, w);
  }
  else
  {
    stopRobot(); //stop currentState
    startDrive();
    driveSupervisor.setGoal(v, w);
  }
}

bool isBatteryLow()
{
  batteryVoltage = (double)analogRead(VOLT_IN_PIN) * 0.0352771 + 0.2;
  if ((batteryVoltage < 9 && batteryVoltage < 7.2) || (batteryVoltage > 9 && batteryVoltage < 11.5)) // && batteryVoltage > 5) // Equal to 3.4V per cell - don't turn on if it's below 5V, this means that no battery is connected
    return true;
  else
    return false;
}

bool waitForEcho = false;
long lastTrigTimer = 0;

void processUltrasonic()
{
  if (waitForEcho)
  {
    if (echoTime > 0)
    {
      ultrasonicDistance = 0.00017 * echoTime;
      if (ultrasonicDistance > MAX_ULTRASONIC_DIS)
        ultrasonicDistance = MAX_ULTRASONIC_DIS - 0.01;
      waitForEcho = false;
      echoTime = 0;
    }
    else if (millis() - lastTrigTimer > 50)
    {
      waitForEcho = false;
      //      Serial.println("u ...");
      ultrasonicDistance = 1.2; //MAX_ULTRASONIC_DIS;
    }
  }
  else
  {
    long curTime = millis();
    if (curTime - lastTrigTimer < 40)
      return;
    lastTrigTimer = curTime;
    waitForEcho = true;

    digitalWrite(ULTRASONIC_TRIG, HIGH); //trig the ultrosonic

    long curMicros = micros();
    while (true) //10us pules
    {
      if (micros() - curMicros > 10)
        break;
    }
    digitalWrite(ULTRASONIC_TRIG, LOW); //trig the ultrosonic
  }
}

//the ultrasonic isr service
void UltrasonicEcho()
{

  int echoSig = digitalRead(ULTRASONIC_ECHO);
  if (echoSig == HIGH)
  {
    trigTime = micros();
    echoTime = 0;
  }
  else
    echoTime = micros() - trigTime;
}
