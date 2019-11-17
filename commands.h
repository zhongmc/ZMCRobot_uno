#ifndef _COMMANDS_H
#define _COMMANDS_H


 //blueTooth and serial commands


#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
//robot info'
#define ROBOT_INFO     'i'  
#define MOTOR_SPEEDS   'm'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'


//   else if (ch0 == 's' && ch1 == 't') //stop
//   else if (ch0 == 'r' && ch1 == 's') //RESET
//   else if (ch0 == 'c' && ch1 == 'i') //count info

//   else if (ch0 == 'm' && ch1 == 'm') // move motor in pwm for 1 sec m pwm-l pwm-r
//   else if (ch0 == 's' && ch1 == 'p') //speed test pwm0 pwm1 step
//    else if (ch0 == 's' && ch1 == 'd') //set drive Goal v w

//   else if (ch0 == 's' && ch1 == 'm') //simulate mode
//   else if (ch0 == 'i' && ch1 == 'o') //ignore atObstacle

//   else if (ch0 == 't' && ch1 == 'l') //turn around left/ right(-pwm) test tl pwm/-pwm stopCount
//   else if (ch0 == 'm' && ch1 == 'g') //go to goal
//   else if (ch0 == 'g' && ch1 == 'o') //start go to goal
//   else if (ch0 == 'g' && ch1 == 'g') //set goto goal goal x y theta v

//   else if (ch0 == 'o' && ch1 == 'd') //set obstacle distance o0-04
//   else if (ch0 == 'r' && ch1 == 'p') //set robot position x y theta
//   else if (ch0 == 'p' && ch1 == 'i') //pid kp ki kd

//     set have IR   ir idx val=0 1 2 3 
//     Calibrate IMU
//     ir filter


#endif