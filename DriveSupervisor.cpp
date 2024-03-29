
#include "DriveSupervisor.h"
#include "ZMCRobotROS.h"

DriveSupervisor::DriveSupervisor()
{
  d_unsafe = 0.11;

  m_input.x_g = 0;
  m_input.y_g = 0;
  m_input.v = 0.3;
  m_input.theta = 0;

  //  robot.setVel2PwmParam(0, 6.4141, 14.924); // vel to pwm parameters
  //   robot.setVel2PwmParam(0,9.59,18.73);

  robot.setIRSensorType(GP2Y0A21);

  robot.setHaveIrSensor(0, true);
  robot.setHaveIrSensor(1, true);
  robot.setHaveIrSensor(2, false);
  robot.setHaveIrSensor(3, true);
  robot.setHaveIrSensor(4, true);

  mSimulateMode = false;
  mIgnoreObstacle = false;
  alpha = 0.5;
  mUseIMU = false;
  danger = false;
}

void DriveSupervisor::setIRFilter(bool open, float val)
{
  robot.setIRFilter(open, val);
}

void DriveSupervisor::setHaveIRSensor(int idx, byte val)
{
  robot.setHaveIrSensor(idx, val);
}

void DriveSupervisor::updateSettings(SETTINGS settings)
{
  if (settings.sType == 0 || settings.sType == 5)
  {
    d_unsafe = settings.unsafe;
    m_input.v = settings.velocity;
    robot.updateSettings(settings);
  }

  if (settings.sType == 0 || settings.sType == 1 || settings.sType == 2)
  {
    robot.updatePID(settings);
    m_Controller.updateSettings(settings);
  }
}

void DriveSupervisor::init()
{
  SETTINGS settings = robot.getPIDParams();
  m_Controller.updateSettings(settings);
}

// drive the robot velocity and turning w
void DriveSupervisor::setGoal(double v, double w)
{
  m_input.v = v;
  m_input.theta = w;
  m_Controller.setGoal(v, w);
}

void DriveSupervisor::resetRobot()
{
  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
  m_Controller.setGoal(m_input.v, 0, 0);
  m_Controller.reset(&robot);
}

void DriveSupervisor::reset(long leftTicks, long rightTicks)
{

  danger = false;
  if (mSimulateMode)
  {
    m_left_ticks = 0;
    m_right_ticks = 0;
    robot.reset(m_left_ticks, m_right_ticks);
  }
  else
    robot.reset(leftTicks, rightTicks);
  m_Controller.reset(&robot);
}

void DriveSupervisor::execute(long left_ticks, long right_ticks, double gyro, double dt)
{

  //  uint32_t timer = micros();

  if (mSimulateMode)
    robot.updateState((long)m_left_ticks, (long)m_right_ticks, dt);
  else
  {
    if (mUseIMU)
      robot.updateState(left_ticks, right_ticks, gyro, alpha, dt);
    else
      robot.updateState(left_ticks, right_ticks, dt);
  }

  check_states();

  if (!mSimulateMode && m_input.v > 0 && danger)
  {
    if (m_state != S_STOP)
      Serial.println("Danger!");
    m_state = S_STOP; //s_stop;
    StopMotor();
    return;
  }

  m_Controller.execute(&robot, &m_input, &m_output, dt);


  v = m_output.v;
  w = m_output.w;

  PWM_OUT pwm = robot.getPWMOut(v, w);

  if (mSimulateMode)
  {
    m_left_ticks = m_left_ticks + robot.pwm_to_ticks_l(pwm.pwm_l, dt);
    m_right_ticks = m_right_ticks + robot.pwm_to_ticks_r(pwm.pwm_r, dt);
  }
  else
  {
    MoveLeftMotor(pwm.pwm_l);
    MoveRightMotor(pwm.pwm_r);
  }

  //send robot position
  log("RP%d,%d,%d,%d,%d\n",
      (int)(10000 * robot.x),
      (int)(10000 * robot.y),
      (int)(10000 * robot.theta),
      (int)(10000 * robot.w),
      (int)(10000 * robot.velocity));

  //send IRSensor info
  IRSensor **irSensors = robot.getIRSensors();
  log("IR%d,%d,%d,%d,%d\n",
      (int)(100 * irSensors[0]->distance),
      (int)(100 * irSensors[1]->distance),
      (int)(100 * irSensors[2]->distance),
      (int)(100 * irSensors[3]->distance),
      (int)(100 * irSensors[4]->distance));
}

// extern double ultrasonicDistance;

void DriveSupervisor::check_states()
{

  IRSensor **irSensors = robot.getIRSensors();

  if (irSensors[2]->distance < d_unsafe)
    danger = true;
  else
    danger = false;
}

void DriveSupervisor::setRobotPosition(double x, double y, double theta)
{
  robot.x = x;
  robot.y = y;
  robot.theta = theta;
}

Position DriveSupervisor::getRobotPosition()
{
  Position pos;
  pos.x = robot.x;
  pos.y = robot.y;
  pos.theta = robot.theta;
  return pos;
}

void DriveSupervisor::getIRDistances(double dis[5])
{
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
}

void DriveSupervisor::readIRDistances(double dis[5])
{
  robot.readIRSensors();
  IRSensor **irSensors = robot.getIRSensors();
  for (int i = 0; i < 5; i++)
  {
    dis[i] = irSensors[i]->distance;
  }
}

void DriveSupervisor::getRobotVel(double dis[5])
{
  dis[0] = robot.vel_l;
  dis[1] = robot.vel_r;
}
