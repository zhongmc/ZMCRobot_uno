#include "ZMCRobotROS.h"
#include "DriveSupervisor.h"


void getDoubleValues(char *buffer, int c, double *fvs);
const char *floatToStr(int idx, double val);
const char *floatToStr(int idx, signed char width, unsigned char prec, double val);


extern DriveSupervisor driveSupervisor;

// extern long trigTime, echoTime;
// extern double ultrasonicDistance;

static char comData[32];
int comDataCount = 0;

//extern Supervisor supervisor;
// extern double gp2y0a41[3][4];

extern long count1, count2;

extern bool openDebug;

extern double batteryVoltage;



void CalibrateIMU()
{
  // Serial.println("Internal sensor offsets BEFORE calibration( xAcc,yAcc,zAcc, xGyro, yGyro, zGyro ...");
  // Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
  // Serial.print("\t"); // -76
  // Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
  // Serial.print("\t"); // -235
  // Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
  // Serial.print("\t"); // 168
  // Serial.print(CurieIMU.getGyroOffset(X_AXIS));
  // Serial.print("\t"); // 0
  // Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
  // Serial.print("\t"); // 0
  // Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

  // Serial.println("About to calibrate. Make sure your board is stable and upright");
  // delay(5000);

  // // The board must be resting in a horizontal position for
  // // the following calibration procedure to work correctly!
  // Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
  // CurieIMU.autoCalibrateGyroOffset();
  // Serial.println(" Done");

  // Serial.print("Starting Acceleration calibration and enabling offset compensation...");
  // CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  // CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  // CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  // Serial.println(" Done");

  // Serial.println("Internal sensor offsets AFTER calibration...");
  // Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
  // Serial.print("\t"); // -76
  // Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
  // Serial.print("\t"); // -2359
  // Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
  // Serial.print("\t"); // 1688
  // Serial.print(CurieIMU.getGyroOffset(X_AXIS));
  // Serial.print("\t"); // 0
  // Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
  // Serial.print("\t"); // 0
  // Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
}

void printCountInfo()
{

  log("CI:%d, %d.\n", count1, count2);
}

// void motorSpeed(int pwm)
// {
//   long c1, c2, lt;
//   MoveMotor(pwm);
//   log("%d,", pwm);

//   // Serial.print(pwm);
//   // Serial.print(',');
//   delay(500);
//   c1 = count1;
//   c2 = count2;
//   lt = millis();
//   delay(1000);

//   c1 = count1 - c1;
//   c2 = count2 - c2;
//   lt = millis() - lt;
//   log("%d,%d,%d\n", lt, c1, c2);
// }



// void speedTest(int pwm0, int pwm1, int step)
// {
//   // long c1, c2, lt;
//   for (int i = pwm0; i < pwm1; i += step)
//   {
//     motorSpeed(i);
//   }
//   MoveMotor(0);
// }


void getDoubleValues(char *buffer, int c, double *fvs)
{
  char *buf = buffer;
  for (int i = 0; i < c; i++)
  {
    fvs[i] = (float)atoi(buf) / 1000.0;
    buf = strchr(buf, ',');
    if (buf == NULL)
      break;
    buf++;
    // Serial.print(fvs[i]);
    // Serial.print(',');
  }
  // Serial.println(';');
}

int formatStr(char *buf, const char *format, ...)
{
  int c = 0;
  va_list vArgList;
  va_start(vArgList, format);
  c = vsprintf(buf, format, vArgList); //_vsnprintf(buf, 256, format, vArgList);
  va_end(vArgList);
  // *(buf + c) = 0;
  return c;
}

void log(const char *format, ...)
{
  char tmp[500];
  va_list vArgList;
  va_start(vArgList, format);
  vsprintf(tmp, format, vArgList);
  va_end(vArgList);
  Serial.print(tmp);
}

char tmp[20][15];

const char *floatToStr(int idx, double val)
{

  return floatToStr(idx, (signed char)6, (unsigned char)3, val);
}

const char *floatToStr(int idx, signed char width, unsigned char prec, double val)
{
  if (idx >= 19)
    return NULL;
  dtostrf(val, width, prec, tmp[idx]);
  return tmp[idx];
}

void setPID(char *buffer)
{
  double p, i, d;
  p = atof((buffer));
  char *buf = strchr(buffer, ',');
  i = atof((buf + 1));
  buf = strchr((buf + 1), ',');
  d = atof(buf + 1);

  log("PID:%s, %s, %s;\n",
      floatToStr(0, p),
      floatToStr(1, i),
      floatToStr(2, d));

  // Serial.print("PID:");
  // Serial.print(p);
  // Serial.print(",");
  // Serial.print(i);
  // Serial.print(",");
  // Serial.println(d);

  SETTINGS settings;
  settings.sType = 1;
  settings.kp = p;
  settings.ki = i;
  settings.kd = d;
  driveSupervisor.updateSettings(settings);
}




void processCommand(char *buffer, int bufferLen)
{
  *(buffer + bufferLen) = 0;
  if (bufferLen <= 2)
  {
    if (buffer[0] == 'b') //get baundrate
      Serial.println(115200);
    else if (buffer[0] == 'p')
      Serial.println(100);
    else
      Serial.println("Na");
    return;
  }
  char ch0, ch1;
  ch0 = tolower(buffer[0]);
  ch1 = tolower(buffer[1]);

  if (ch0 == 'g' && ch1 == 'r')
  {
    Serial.println("\r\n\r\n====");
    Serial.print("BAT:");
    Serial.print(batteryVoltage);
    Serial.print(", uc dis:");
    // Serial.println(ultrasonicDistance);
    Serial.println("\r\n==");
    driveSupervisor.getRobotInfo();
  }
  else if (ch0 == 's' && ch1 == 't') //stop
  {
    // Serial.println("Stop!");
    stopRobot();
  }

  else if (ch0 == 'c' && ch1 == 'i') //count info
  {
    printCountInfo();
  }

  // else if (ch0 == 'm' && ch1 == 'l') // move left motor
  // {
  //   int pwm = atoi(buffer + 2);
  //   printCountInfo();
  //   MoveLeftMotor(pwm);
  // }

  // else if (ch0 == 'm' && ch1 == 'r') // move right motor
  // {
  //   int pwm = atoi(buffer + 2);
  //   printCountInfo();
  //   MoveRightMotor(pwm);
  // }

  // else if (ch0 == 'm' && ch1 == 'm') // move motor
  // {
  //   int pwm = atoi(buffer + 2);
  //   motorSpeed(pwm);
  //   MoveMotor(0);
  // }
  // else if (ch0 == 's' && ch1 == 'p') //speed test
  // {
  //   int pwm0, pwm1, step = 0;
  //   pwm0 = atoi(buffer + 2);
  //   char *buf = strchr((buffer + 2), ',');
  //   pwm1 = atoi(buf + 1);
  //   buf = strchr((buf + 1), ',');
  //   step = atoi(buf + 1);

  //   // Serial.print("SP:");
  //   // Serial.print(pwm0);
  //   // Serial.print(",");
  //   // Serial.print(pwm1);
  //   // Serial.print(",");
  //   // Serial.println(step);
  //   log("SP:%d,%d,%d\n", pwm0, pwm1, step);
  //   if (step == 0)
  //     step = 10;

  //   speedTest(pwm0, pwm1, step);
  // }

  else if (ch0 == 'r' && ch1 == 's') //RESET
  {
    ResetRobot();
  }

  else if (ch0 == 's' && ch1 == 'd') //set drive Goal
  {
    double v, w = 0;

    v = atof(buffer + 2);
    char *buf = strchr(buffer, ',');
    if (buf != NULL)
      w = atof(buf + 1);

    setDriveGoal(v, w);
  }
  else if (ch0 == 's' && ch1 == 'm') //simulate mode
  {
    int val = atoi(buffer + 2);
    SetSimulateMode(val);

    //     if (val == 1)
    //         SetSimulateMode(true);
    // else
    //   SetSimulateMode(false);
  }

  else if (ch0 == 'g' && ch1 == 'o') //start go to goal
  {
    startGoToGoal();
  }

  else if (ch0 == 'g' && ch1 == 'g') //set goto goal goal
  {

    double fvs[4];
    getDoubleValues(buffer + 2, 4, fvs);
    setGoal(fvs[0], fvs[1], fvs[2], fvs[3]);
  }

  else if (ch0 == 'o' && ch1 == 'd') //set obstacle distance
  {
    double ods[5];
    getDoubleValues(buffer + 2, 5, ods);
    // supervisor.setObstacleDistance(ods);
  }
  else if (ch0 == 'r' && ch1 == 'p') //set robot position
  {
    double fvs[3];
    getDoubleValues(buffer + 2, 3, fvs);
    driveSupervisor.setRobotPosition(fvs[0], fvs[1], fvs[2]);
    //to do
  }

  // else if (ch0 == 's' && ch1 == 'r') // step response
  // {
  //   int pwm = atoi(buffer + 2);
  //   stepResponseTest(pwm);
  // }

  else if (ch0 == 'p' && ch1 == 'i') //pid
  {
    setPID(buffer + 2);
  }

  else if (ch0 == 'i' && ch1 == 'o') //ignore atObstacle
  {
    int val = atoi(buffer + 2);
    SetIgnoreObstacle(val);

    // if (val == 1)
    //   SetIgnoreObstacle(true);
    // else
    //   SetIgnoreObstacle(false);
  }

  else
  {
    Serial.println("Unknow");
  }
}

void checkSerialData()
{

  //read speed setting from serial
  if (Serial.available())
  {
    while (Serial.available() > 0)
    {
      if (comDataCount > 30) //some error
      {
        Serial.print("UnKnow cmd:");
        comData[30] = 0;
        Serial.println(comData);
        comDataCount = 0;
      }
      char ch = Serial.read();
      comData[comDataCount++] = ch;
      if (ch == ';' || ch == '\r' || ch == '\n') //new command
      {
        processCommand(comData, comDataCount);
        comDataCount = 0;
      }
    }
  }
}
