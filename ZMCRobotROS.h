#ifndef _ZMC_ROBOT_ROS_H_
#define _ZMC_ROBOT_ROS_H_

#include <Arduino.h>

// TT 单相测速， dual A/B相测速， CHANGE A/B相，以改变中断计数，上升沿、下降沿都计数，精度加倍
#define MOTOR DUAL_MOTOR //TT_MOTOR DUAL_MOTOR CHANGE_MOTOR

#define MAX_ULTRASONIC_DIS 1
#define MAX_IRSENSOR_DIS 0.3
#define SPEED_DOWN_DIS 0.2 //distance to goal to speed down
#define DIS_SPEED_DOWN_SCALE 10
#define W_SPEED_DOWN_SCALE 1

typedef struct
{
  double x, y;
} Vector;

typedef struct
{
  double x, y, theta;
} Position;

typedef struct
{
  double kp, ki, kd;
} PIDParam;

long readLeftEncoder();
long readRightEncoder();

void MoveLeftMotor(int PWM);
void MoveRightMotor(int PWM);
void StopMotor();

void setGoal(double x, double y, int theta, double v);
void startGoToGoal();
void stopRobot();
void ResetRobot();
void SetSimulateMode(bool sm);

void SetIgnoreObstacle(bool igm);

void setDriveGoal(double v, double w);
void startDrive();
void stopAndReset();

void initMotor();
void initBluetooth();
void checkSerialData();
//ble cmd process
void processSetingsRequire();
void sendRobotStateValue(byte stateType, Position pos, double irDistance[5], double voltage);

void sendBalanceRobotStateValue(Position pos, double irDistance[5], double voltage);

void MoveMotor(int pwm);

void checkBLTL();
void turnAround(int pwm);
void setPID(char *buffer);

void startBalance();
void stopBalance();
void balanceUnnormal();
void balanceRecovered();
void irRemoteProcess(int code);

int formatStr(char *buf, char *fmt, ...);

void log(const char *fmt, ...);

const char *floatToStr(int idx, double val);
const char *floatToStr(int idx, char width, unsigned char prec, double val);
#endif
