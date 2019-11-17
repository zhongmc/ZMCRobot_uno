#ifndef _REAR_DRIVE_ROBOT_H_
#define _REAR_DRIVE_ROBOT_H_

#include <Arduino.h>
#include "Robot.h"

class RearDriveRobot : public Robot
{
public:
  RearDriveRobot();

  Vel ensure_w(double v, double w);
  PWM_OUT getPWMOut(double v, double w);

  double vel_l_to_pwm(double vel);
  double vel_r_to_pwm(double vel);

  double pwm_to_ticks_r(double pwm, double dt);
  double pwm_to_ticks_l(double pwm, double dt);

  // Vel zeroMinVel(Vel vel);
};

#endif /* _REAR_DRIVE_ROBOT_H_ */
