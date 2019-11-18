#include "ZMCRobotROS.h"
#include <SoftwareSerial.h>
#include "DriveSupervisor.h"

extern DriveSupervisor driveSupervisor;
extern long count1, count2;


bool connectToHM10();
void SendBlePackage(byte len, byte pkgType, byte *buf);
void processCommandPackage(byte *dataBuf);
void sendRobotStateValue(double x, double y, double theta, double irDistance[5], double voltage );
void sendBalanceRobotStateValue(double x, double y, double theta, double irDistance[5], double voltage );
void floatToByte( byte *arrayBuf, double val, double scale );
void intToByte(byte *arrayBuf, int val);
double byteToFloat(byte *arrayBuf, double scale );
int byteToInt(byte *arrayBuf );
void setConfigValue(const unsigned char *cfgArray);
void processCommandPackage(byte*);
void sendSettingsData( int settingsType );





#define bluetoothTx  10
#define bluetoothRx  11

byte bleBuffer[30];
byte bleBufLen;

//settings parameters
// double atObstacle = 0.25, unsafe = 0.1, angleOff = 0, velocity = 0.5, dfw = 0.15, wheelSyncKp = 10;
// int pwm_zero = 0, pwm_diff = 0, max_pwm = 150, max_rpm = 150, min_rpm = 50;

//defined in BalancePID and SpeedPID
// extern double sKp, sKi, sKd, bKp, bKi, bKd;


extern bool doCheckBatteryVoltage;

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

bool bleConnected = false;

// remote clients will be able to get notifications if this characteristic changes
// the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications

void enumBlooth()
{
      while (bluetooth.available()) {
        char chr = bluetooth.read();
        Serial.write(chr);
      }

}

void initBluetooth()
{

  Serial.println("init BLE HM10 ...");
  bluetooth.begin(19200);
  bleConnected = false;
  /* Now activate the BLE device. It will start continuously transmitting BLE
    advertising packets and will be visible to remote BLE central devices
    until it receives a new connection */

    bool ret = connectToHM10();
    
  // assign event handlers for connected, disconnected to peripheral
  if( ret == true )
  {
    bluetooth.write("AT+NAMEZMCRobot-UNO");
    delay(100);
    enumBlooth();
    Serial.write('\n');

    bluetooth.write("AT+NOTI1");
    delay(200);
    enumBlooth();

    Serial.println("\nBLE ready...");
  }
  bleBufLen = 0;
}

void doBleHM10Loop()
{
  bool newChars = false;
  char chr;
  while (bluetooth.available()) {
    chr = bluetooth.read();
    bleBuffer[bleBufLen++] = chr; //bluetooth.read();
    // Serial.write(chr);
    // Serial.print(':');
//    Serial.print((char)chr);
//    Serial.print("; ");
    if ( bleBufLen >= 40 )
    {
      Serial.println("BLE over flow!");
      bleBufLen = 0;
    }
    newChars = true;
  }

  if( !newChars )
    return;
    
  if ( bleBufLen == 0 )
    return;

  if( newChars )
    Serial.println();

    
  //OK+LOST
  if( bleBufLen >=7 && bleBuffer[0] == 'O' && bleBuffer[1] == 'K' && bleBuffer[2] == '+' && bleBuffer[3]=='L') // the disconnect info OK+LOST
  {
     bleConnected = false;
     bleBufLen = 0;
     Serial.println("BLE disc!");
     return;
  }
  
  
  if ( ! bleConnected ) //check the OK+CONN
  {
    if( bleBufLen >=7 && bleBuffer[0] == 'O' && bleBuffer[1] == 'K' && bleBuffer[2] == '+' && bleBuffer[3]=='C')    
    {
      bleConnected = true;
      memmove(bleBuffer, bleBuffer + 7, bleBufLen - 7 );
      bleBufLen = bleBufLen - 7;
      Serial.println("BLE Con!");
    }
    if( bleBufLen > 2 && bleBuffer[0] == bleBufLen-2 ) //get a full package!
    {
      bleConnected = true;
      Serial.println("BLE...");
    }
  }

  if( !bleConnected )
    return;
    
  while ( bleBufLen > 0 )
  {

    //pkgType pkglen
    byte pkgLen = bleBuffer[0];
    byte pkgType = bleBuffer[1];
    if( pkgLen > 20 || pkgLen < 0 ) // error
    {
      Serial.println("ble pkg error!");
      bleBufLen = 0;
      break;
    }
    if ( bleBufLen >= (pkgLen + 2))
    {
      if ( pkgType == 0 ) //cmd
      {
        processCommandPackage(bleBuffer + 2 );
      }
      else if( pkgType > 0 && pkgType < 6)
      {
        setConfigValue(bleBuffer + 1);
      }
      else
      {
        Serial.println("Error pkg!");
      }
      memmove(bleBuffer, bleBuffer + pkgLen + 2, bleBufLen - pkgLen - 2);
      bleBufLen = bleBufLen - pkgLen - 2;
    }
    else  //wait for next package......
    {
      break;
    }
  }
}


void processCommandPackage(byte *dataBuf) {
  // central wrote new value to characteristic, update LED
  Serial.print("CMD:");
  //the first two chas as CMD
  const char *data = (unsigned char *)dataBuf;
  char cmd[3];
  cmd[2] = '\0';
  cmd[0] = data[0];
  cmd[1] = data[1];
  Serial.println( cmd );

 if (cmd[0] == 'S' && cmd[1] == 'T') //stop
  {
    stopRobot();
  }

  else if (cmd[0] == 'R' && cmd[1] == 'P' ) //Required for settings
  {
    Serial.println("Required for settings Of:");
    Serial.println( data[2]);

    sendSettingsData( data[2] );


  }


  // else if (cmd[0] == 'C' && cmd[1] == 'B') // check battle voltage
  // {
  //   if (data[2] == 1)
  //     doCheckBattleVoltage = true;
  //   else
  //     doCheckBattleVoltage = false;
  // }

  else if (cmd[0] == 'S' && cmd[1] == 'M') //simulate mode
  {
    if (data[2] == 1)
      SetSimulateMode(true);
    else
      SetSimulateMode(false);
  }
  else if (cmd[0] == 'I' && cmd[1] == 'O') //ignore atObstacle
  {
    if (data[2] == 1)
      SetIgnoreObstacle(true);
    else
      SetIgnoreObstacle(false);
  }

  else if (cmd[0] == 'R' && cmd[1] == 'S') //RESET
  {
    ResetRobot();
  }

  else if (cmd[0] == 'G' && cmd[1] == 'O') // action...
  {
    startGoToGoal();
  }

  // else if (cmd[0] == 'G' && cmd[1] == 'D') //start drive mode
  // {
  //   startDrive();
  // }

  else if (cmd[0] == 'G' && cmd[1] == 'G') // Go To Goal: x, y, theta
  {
    double x, y, v;
    int theta;
    x = byteToFloat((byte *)(data + 2), 100);
    y = byteToFloat((byte *)(data + 4), 100);
    theta = byteToInt((byte *)(data + 6));
    v = byteToFloat((byte *)(data + 8), 100);

    setGoal(x, y, theta, v);

    log("GTG:%s,%s,%d,%s\n",
        floatToStr(0, x),
        floatToStr(1, y),
        theta,
        floatToStr(2, v));

    // Serial.print("GTG:");
    // Serial.print(x);
    // Serial.print(",");
    // Serial.print(y);
    // Serial.print(",");
    // Serial.println(theta);
  }

  // else if (cmd[0] == 'T' && cmd[1] == 'L') //turn around
  // {
  //     int pwm = 80;
  //     pwm = atoi( (data + 2) );
  //    turnAround(pwm);
  // }

  else if (cmd[0] == 'I' && cmd[1] == 'M') //use IMU or not IM0/1,0.5;
  {
    bool val = *(data + 2) - '0';
    float alpha = 0.5;
    if (val == true)
      alpha = atof((char *)(data + 4));
    log("use IMU:%d,%s\n", val, floatToStr(0, alpha));
    driveSupervisor.mUseIMU = val;
    driveSupervisor.alpha = alpha;
    //driveSupervisor.setUseIMU(val, alpha);
  }

  else if (cmd[0] == 'I' && cmd[1] == 'F') // set ir filter IF0/1,0.6;
  {
    bool val = *(data + 2) - '0';
    float filter = atof((char *)(data + 4));
    log("IR flt:%d,%s\n", val, floatToStr(0, filter));
    driveSupervisor.setIRFilter(val, filter);
  }

  else if (cmd[0] == 'I' && cmd[1] == 'R')
  {
    short idx = *(data + 2) - '0';
    byte val = *(data + 3) - '0';
    log("S IR:%d,%d\n", idx, val);

    driveSupervisor.setHaveIRSensor(idx, val);
  }

  else if (cmd[0] == 'P' && cmd[1] == 'I') //pid
  {
    setPID((char *)(data + 2));
  }
  else if (cmd[0] == 'S' && cmd[1] == 'D') //set drive Goal
  {
    double v, w;
    v = byteToFloat((byte *)(data + 2), 100);
    w = byteToFloat((byte *)(data + 4), 100);
    log("v=%s,w=%s\n",
        floatToStr(0, v),
        floatToStr(1, w));
    setDriveGoal(v, w);
  }


}




void setConfigValue(const unsigned char *cfgArray)
{
  int settingsType = (int)cfgArray[0];
  Serial.print("cfg v BT:");
  Serial.println(settingsType);

  SETTINGS settings;
  settings.sType = settingsType;

  if (settingsType == 1)  //  || settingsType == 2 || settingsType == 3 || settingsType == 4)
  {
    settings.kp = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.ki = byteToFloat((byte *)(cfgArray + 3), 1000);
    settings.kd = byteToFloat((byte *)(cfgArray + 5), 1000);

    // log("KP:%s, KI:%s, KD:%s\n",
    //     floatToStr(0, settings.kp),
    //     floatToStr(1, settings.ki), floatToStr(2, settings.kd));
    // Serial.print("KP:");
    // Serial.print(settings.kp);
    // Serial.print(" KI:");
    // Serial.print(settings.ki);
    // Serial.print(" KD:");
    // Serial.println(settings.kd);
  }

  else if (settingsType == 5)
  {

    // settings.atObstacle = byteToFloat((byte *)(cfgArray + 1), 100);
    settings.unsafe = byteToFloat((byte *)(cfgArray + 3), 100);
    // settings.dfw = byteToFloat((byte *)(cfgArray + 5), 100);
    settings.velocity = byteToFloat((byte *)(cfgArray + 7), 100);

    // settings.max_rpm = byteToInt((byte *)(cfgArray + 9));
    // settings.min_rpm = byteToInt((byte *)(cfgArray + 11));

    settings.radius = byteToFloat((byte *)(cfgArray + 13), 1000);
    settings.length = byteToFloat((byte *)(cfgArray + 15), 1000);

    // settings.pwm_diff = (int)cfgArray[13]; //byteToInt((byte *)(cfgArray + 13) );
    // settings.pwm_zero = (int)cfgArray[14];
    // settings.angleOff = byteToFloat((byte *)(cfgArray + 15), 100);

    // log("atObs:%s, unsafe:%s, dfw:%s, v:%s, max_rmp:%d, min_rpm:%d, R:%s, L:%s\n",
    //     floatToStr(0, settings.atObstacle),
    //     floatToStr(1, settings.unsafe),
    //     floatToStr(2, settings.dfw),
    //     floatToStr(3, settings.velocity),
    //     settings.max_rpm,
    //     settings.min_rpm,
    //     floatToStr(4, settings.radius),
    //     floatToStr(5, settings.length)

    // );
  }

  driveSupervisor.updateSettings(settings);

  // if (settingsType == 1 || settingsType == 5 || settingsType == 2 || settingsType == 3 || settingsType == 4)
  // {
  //   driveSupervisor.updateSettings(settings);
  // }
  // setSettings(settings);
  //  updateConfigToMenu();
}

//type, x, y, theta, d0,d1,d2,d3,d4,voltage
void sendRobotStateValue(byte stateType, Position pos, double irDistance[5], double voltage)
{
  if (!bleConnected)
    return;

  byte buf[19];
  memset(buf, 0, 19);

  buf[0] = stateType;

  double scale = 1000;

  floatToByte(buf + 1, pos.x, scale);
  floatToByte(buf + 3, pos.y, scale);
  floatToByte(buf + 5, pos.theta, scale);

  //    Serial.print( voltage);
  //    Serial.print( ", " );

  scale = 100;
  for (int i = 0; i < 5; i++)
  {
    floatToByte(buf + 7 + 2 * i, irDistance[i], scale);
    //  Serial.print( irDistance[i] );
    //    Serial.print( "," );
  }
  //    Serial.println( ";" );

  floatToByte(buf + 17, voltage, scale);
 SendBlePackage(18, 6, buf); //len package type, dataBuf

}

void SendBlePackage(byte len, byte pkgType, byte *buf)
{
  if ( !bleConnected )
    return;

  bluetooth.write((byte)len);
  bluetooth.write( (byte)pkgType);
  for (int i = 0; i < len; i++)
    bluetooth.write(*(buf + i));
}


void sendSettingsData( int settingsType )
{

  SETTINGS settings;
  settings.sType = settingsType;
  // // 1: pid for 3 wheel; 2: pid for balance;  3: pid for speed; 4: PID theta  5: settings for robot; 6: settings for balance robot;

//   settings = driveSupervisor.getSettings(sType);


  //  int settingsType;  // 1: pid for 3 wheel; 2: pid for balance;  3: pid for speed; 4: settings for robot; 5: settings for balance robot;
  byte settingsArray[18];
  byte len;

  memset( settingsArray, 0, 18);

  settingsArray[0] = (byte)settingsType;
  len = 7;

  if (settingsType == 1) // || settingsType == 2 || settingsType == 3 || settingsType == 4)
  {

    floatToByte(settingsArray + 1, settings.kp, 100);
    floatToByte(settingsArray + 3, settings.ki, 1000);
    floatToByte(settingsArray + 5, settings.kd, 1000);
  }

  else if (settingsType == 5)
  {
    // floatToByte(settingsArray + 1, settings.atObstacle, 100);
    floatToByte(settingsArray + 3, settings.unsafe, 100);
    // floatToByte(settingsArray + 5, settings.dfw, 100);
    floatToByte(settingsArray + 7, settings.velocity, 100);

    // intToByte(settingsArray + 9, settings.max_rpm);
    // intToByte(settingsArray + 11, settings.min_rpm);

    floatToByte(settingsArray + 13, settings.radius, 1000);

    // settingsArray[13] = (byte)settings.pwm_diff;
    // settingsArray[14] = (byte)settings.pwm_zero;

    floatToByte(settingsArray + 15, settings.length, 1000);

    len = 17;
  }


  SendBlePackage(len, settingsType, settingsArray);

}




void intToByte(byte *arrayBuf, int val)
{
  if ( val > 0)
  {
    *arrayBuf = val & 0xff;
    val = (val & 0xff00) / 256;
    *(arrayBuf + 1) = val;
  }
  else
  {
    val = -val;
    *arrayBuf = val & 0xff;
    val = (val & 0xff00) / 256;
    *(arrayBuf + 1) = val | 0x80;
  }
}

void floatToByte( byte *arrayBuf, double val, double scale )
{
  int tmp = (int)( val * scale );
  if ( tmp > 0)
  {
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp;
  }
  else
  {
    tmp = -tmp;
    *arrayBuf = tmp & 0xff;
    tmp = (tmp & 0xff00) / 256;
    *(arrayBuf + 1) = tmp | 0x80;
  }
}


double byteToFloat(byte *arrayBuf, double scale )
{
  int val = *(arrayBuf + 1) & 0x7f;
  val = val * 256 + *arrayBuf;
  if ( (*(arrayBuf + 1) & 0x80) != 0)
    val = -val;
  return (double)val / scale;
}

int byteToInt(byte *arrayBuf )
{
  int val = *(arrayBuf + 1)  & 0x7f;
  val = val * 256 + *arrayBuf;
  if ( (*(arrayBuf + 1) & 0x80) != 0)
    val = -val;

  return val;
}


bool connectToHM10(){
  bool found = false;
 
    // 0 9600 1 19200 2 38400 3 57600 4 115200  5 4800 6 2400 7 1200 8 230400
   int baudrate[6] ={4800,9600,19200,38400,57600,115200};

  for(int j=0; j<6; j++)
   {
      bluetooth.begin(baudrate[j]);
      delay(100);
      Serial.print("BR:");
      Serial.println( baudrate[j] );
     // Serial.println("");
      bluetooth.write("AT");
      delay(500);
      while (bluetooth.available()) {
        found = true;
        Serial.write(bluetooth.read());
       }
       if( found == true )
       {
        Serial.println("OK!");
        return true;
       }
       delay(100);
   }
   return false;
}    