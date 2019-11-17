#include "RearDriveRobot.h"

RearDriveRobot::RearDriveRobot()
{
  //R, L, ticksr_l, ticksr_r, minRpm, maxRpm, GP2Y0A41);
  // init(0.0325, 0.1785, 330, 360, 60, 150, GP2Y0A41); //0.0325 0.1785； 0.0325， 0.156

  // init(0.0330, 0.16, 390, 390, 50, 180, GP2Y0A41); 2019-02-09 蓝色轮子，有一边转弯差点
  //0.0325 0.1785； 0.0325， 0.156
  // init(0.0313, 0.1631, 390, 390, 50, 180, GP2Y0A21);

  //黑色轮子，长板
  // init(0.0312, 0.1666, 390, 390, 50, 180, GP2Y0A21);

  //黑色轮子，自制板
  init(0.0312, 0.162, 390, 390, 50, 180, GP2Y0A21);

  //balance car
  //  init(0.0332, 0.159, 390, 390, 50, 180, GP2Y0A21);

  //GP2Y0A21 (10-80) GP2Y0A41 (4-30) 长版
  // irSensors[0] = new IRSensor(-0.045, 0.05, PI / 2, A1, GP2Y0A21);
  // irSensors[1] = new IRSensor(0.08, 0.04, PI / 4, A2, GP2Y0A21); //0.16,0.045, PI/6 0.075, 0.035
  // irSensors[2] = new IRSensor(0.162, 0.0, 0, A3, GP2Y0A21);
  // irSensors[3] = new IRSensor(0.08, -0.04, -PI / 4, A4, GP2Y0A21);
  // irSensors[4] = new IRSensor(-0.045, -0.05, -PI / 2, A5, GP2Y0A21);

  //GP2Y0A21 (10-80) GP2Y0A41 (4-30) 自制版
  irSensors[0] = new IRSensor(-0.073, 0.066, PI / 2, A1, GP2Y0A21);
  irSensors[1] = new IRSensor(0.061, 0.05, PI / 4, A2, GP2Y0A21); // 0.16,0.045, PI/6 0.075, 0.035
  irSensors[2] = new IRSensor(0.072, 0.0, 0, A3, GP2Y0A21);
  irSensors[3] = new IRSensor(0.061, -0.05, -PI / 4, A4, GP2Y0A21);
  irSensors[4] = new IRSensor(-0.073, -0.066, -PI / 2, A5, GP2Y0A21);

  haveIrSensor[0] = true;
  haveIrSensor[1] = true;
  haveIrSensor[2] = true;
  haveIrSensor[3] = true;
  haveIrSensor[4] = true;

  mPIDSettings.kp = 5;   // 5; //25;  //20 0.5 2; 2019-01-26:   5, 0.02, 0.9; 5, 0.05, 1.2; 5,0.08,1.2 2019-02-09 5, 0.01, 0.2
  mPIDSettings.ki = 0.2; //.4; // 0.01;
  mPIDSettings.kd = 0.0; //0.02; //0.2
}

PWM_OUT RearDriveRobot::getPWMOut(double v, double w)
{

  Vel vel = ensure_w(v, w);

  int pwm_l = (int)vel_l_to_pwm(vel.vel_l);
  int pwm_r = (int)vel_r_to_pwm(vel.vel_r);

  if (v == 0)
  {
    if (abs(pwm_l) > 80)
    {
      if (pwm_l > 0)
      {
        pwm_l = 80;
        pwm_r = -80;
      }
      else
      {
        pwm_l = -80;
        pwm_r = 80;
      }
    }
  }
  else
  {
    int dif = pwm_l - pwm_r;
    if (dif > 80)
    {
      if (pwm_l > 0)
      {
        pwm_l = pwm_r + 80;
      }
      else
      {
        pwm_r = pwm_l - 80;
      }
    }
    else if (dif < -80)
    {
      if (pwm_l >= 0)
      {
        pwm_r = pwm_l + 80;
      }
      else
      {
        pwm_l = pwm_r - 80;
      }
    }
  }

  PWM_OUT pwm;
  pwm.pwm_l = pwm_l;
  pwm.pwm_r = pwm_r;
  return pwm;
}

Vel RearDriveRobot::ensure_w(double v, double w)
{

  Vel vel = uni_to_diff(v, w);

  if (v == 0)
    return vel;

  if (vel.vel_l * vel.vel_r >= 0)
    return vel;

  if (abs(vel.vel_l) > abs(vel.vel_r))
  {
    vel.vel_r = 0;
  }
  else
    vel.vel_l = 0;

  return vel;

  // if (abs(v) > 0)
  // {
  //   if (abs(w) > 1.2)
  //   {
  //     vel = uni_to_diff(v, w);
  //     vel = zeroMinVel(vel);
  //     return vel;
  //   }

  //   Vel vel_d = uni_to_diff(abs(v), w); // w_lim);

  //   vel.vel_r = vel_d.vel_r;
  //   vel.vel_l = vel_d.vel_l;

  //   if (vel.vel_l > max_vel)
  //     vel.vel_l = max_vel;
  //   if (vel.vel_r > max_vel)
  //     vel.vel_r = max_vel;

  //   if (vel.vel_l < 0)
  //     vel.vel_l = 0;
  //   if (vel.vel_r < 0)
  //     vel.vel_r = 0;

  //   if (v < 0)
  //   {
  //     vel.vel_l = -vel.vel_l;
  //     vel.vel_r = -vel.vel_r;
  //   }
  // }
  // else
  // {
  //   vel = uni_to_diff(0, w);
  //   vel = zeroMinVel(vel);
  //   // if (vel.vel_l < 0)
  //   // {
  //   //   vel.vel_l = 0;
  //   //   vel.vel_r = min_vel + 0.5;
  //   // }
  //   // else
  //   // {
  //   //   vel.vel_r = 0;
  //   //   vel.vel_l = min_vel + 0.5;
  //   // }
  // }
  // return vel;
}

/*
Vel RearDriveRobot::ensure_w(double v, double w)
{
  Vel vel;

  if (abs(v) > 0)
  {
    Vel vel_d = uni_to_diff(abs(v), w); // w_lim);

    double vel_rl_max, vel_rl_min;
    if (vel_d.vel_l > vel_d.vel_r)
    {
      vel_rl_min = vel_d.vel_r;
      vel_rl_max = vel_d.vel_l;
    }
    else
    {
      vel_rl_min = vel_d.vel_l;
      vel_rl_max = vel_d.vel_r;
    }

    if (vel_rl_max > max_vel)
    {
      vel.vel_r = vel_d.vel_r - (vel_rl_max - max_vel);
      vel.vel_l = vel_d.vel_l - (vel_rl_max - max_vel);
      if ((vel_rl_min - (vel_rl_max - max_vel)) < min_vel) //大拐弯？
      {
        vel = zeroMinVel(vel);
      }
    }
    else if (vel_rl_min < min_vel)
    {
      vel.vel_r = vel_d.vel_r + (min_vel - vel_rl_min);
      vel.vel_l = vel_d.vel_l + (min_vel - vel_rl_min);
      // if (vel_rl_max + (min_vel - vel_rl_min) > max_vel) //大拐弯
      // {
      //   vel = zeroMinVel(vel);
      // }
    }
    else
    {
      vel.vel_r = vel_d.vel_r;
      vel.vel_l = vel_d.vel_l;
    }

    if (vel.vel_l > max_vel)
      vel.vel_l = max_vel;
    else if (vel.vel_r > max_vel)
      vel.vel_r = max_vel;

    if (v < 0)
    {
      vel.vel_l = -vel.vel_l;
      vel.vel_r = -vel.vel_r;
    }
  }
  else
  {
    vel = uni_to_diff(0, w);
    vel = zeroMinVel(vel);
    // if (vel.vel_l < 0)
    // {
    //   vel.vel_l = 0;
    //   vel.vel_r = min_vel + 0.5;
    // }
    // else
    // {
    //   vel.vel_r = 0;
    //   vel.vel_l = min_vel + 0.5;
    // }
  }
  return vel;
}

Vel RearDriveRobot::zeroMinVel(Vel vel)
{
  if (vel.vel_l > vel.vel_r)
  {
    vel.vel_r = 0;
    vel.vel_l = min(max(min_vel, vel.vel_l), (min_vel + 5));
  }
  else
  {
    vel.vel_l = 0;
    vel.vel_r = min(max(min_vel, vel.vel_r), (min_vel + 5));
    //    vel.vel_r = min_vel;
  }
  return vel;
}
*/

double RearDriveRobot::vel_l_to_pwm(double vel)
{
  double nvel = abs(vel);

  if (nvel < 0.5)
    return 0;

  double pwm = 6.26 * nvel + 47;
  if (vel < 0)
    return -pwm;
  return pwm;

  // //ax^2+bx+c
  // double nvel = abs(vel);
  // if (nvel < min_vel - 0.1)
  //   return 0;

  // if (nvel > max_vel)
  //   nvel = max_vel;

  // // double retVal = 0.5729 * nvel * nvel - 5.1735 * nvel + 86.516;
  // // double retVal = 9.1631 * nvel + 27.898; //6.393 * nvel + 13.952;
  // //y = 0.1086x2 + 3.4864x + 60.919

  // double retVal = 0.1086 * nvel * nvel + 3.4864 * nvel + 60.919;

  // if (vel >= 0)
  //   return retVal;
  // else
  //   return -retVal;
}

double RearDriveRobot::vel_r_to_pwm(double vel)
{

  double nvel = abs(vel);

  if (nvel < 0.5)
    return 0;

  double pwm = 6.26 * nvel + 47;
  if (vel < 0)
    return -pwm;
  return pwm;

  // //ax^2+bx+c
  // double nvel = abs(vel);

  // if (nvel < min_vel - 0.1)
  //   return 0; //nvel = min_vel;

  // if (nvel > max_vel)
  //   nvel = max_vel;

  // // double retVal = 0.5649 * nvel * nvel - 4.3156 * nvel + 80.706;
  // // double retVal = 9.1631 * nvel + 27.898; // 6.2798 * nvel + 18.787;
  // //  y = 0.5649x2 - 4.3156x + 80.706
  // //y = 0.1194x2 + 3.0069x + 63.031
  // double retVal = 0.1194 * nvel * nvel + 3.0069 * nvel + 63.031;

  // if (vel >= 0)
  //   return retVal;
  // else
  //   return -retVal;
}

double RearDriveRobot::pwm_to_ticks_l(double pwm, double dt)
{
  double npwm = abs(pwm);
  if (npwm < 60) //14
    return 0;

  if (npwm > 220)
    npwm = 220;

  // double ticks = dt * (-0.024 * npwm * npwm + 12.097 * npwm - 426.23);
  //y = -0.0264x2 + 16.836x - 882.53
  double ticks = dt * (-0.0264 * npwm * npwm + 16.836 * npwm - 882.53);

  //5.7132x - 180.16
  //  double ticks = dt * (5.7132 * npwm - 180.16);
  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}

double RearDriveRobot::pwm_to_ticks_r(double pwm, double dt)
{

  double npwm = abs(pwm);
  if (npwm < 60)
    return 0;
  if (npwm > 220)
    npwm = 220;
  // double ticks = dt * (-0.0218 * npwm * npwm + 11.634 * npwm - 358.83);
  //5.7049x - 131.73
  //  double ticks = dt * (5.7049 * npwm - 131.73);
  //y = -0.0312x2 + 18.344x - 974.3

  double ticks = dt * (-0.0312 * npwm * npwm + 18.344 * npwm - 974.3);

  if (pwm > 0)
    return ticks;
  else
    return -ticks;
}
