
#include "myKey.h"

MyKey::MyKey()
{
};

bool MyKey::keyAvailable()
{
  return ( keybuffSize > 0) ;
};

keypress *MyKey::getKeyPress()
{
  if ( keybuffSize > 0 )
  {
    keybuff[4] = keybuff[0];
    for ( int i = 1; i < keybuffSize; i++)
      keybuff[i - 1] = keybuff[i];

    keybuffSize--;
    return &keybuff[4];
  }
  else
    return (keypress *)0;
};

void MyKey::addKeyPress( keypress aKey)
{
  if ( keybuffSize >= 3 )
    return;
  keybuff[keybuffSize++] = aKey;
      Serial.print("Key ");
      Serial.print( aKey.keyId );
      Serial.println(" pressed!");
  
};


byte MyKey::addKey( byte keyId, byte pinIdx)
{
  if( keyCount > 3 )
    return 0;
    keys[keyCount].keyId =  keyId;
    keys[keyCount].pinIdx = pinIdx;
    pinMode( pinIdx, INPUT_PULLUP);
    keys[keyCount].keyState = 0;
    keys[keyCount].pressTime = 0;
   keyCount++;
   return keyCount;
}


void MyKey::initKey( )
{

//  keyCount = 4;
//  for ( int i = 0; i < 4; i++) //d3-d6 as keyboard
//  {
//    keys[i].keyId =  i;
//    keys[i].pinIdx = 3 + i;
//
//    Serial.print("Key ");
//    Serial.print( keys[i].keyId );
//    Serial.print(" to D");
//    Serial.println( 3 + i );
//    pinMode( 3 + i, INPUT_PULLUP);
//    keys[i].keyState = 0;
//    keys[i].pressTime = 0;
//  }
};


void MyKey::checkKey()
{
  for (int i = 0; i < keyCount; i++)
  {
    byte val = digitalRead( keys[i].pinIdx );
    if ( val == LOW && keys[i].keyState == 0 ) //key pressed
    {
      keys[i].keyState = 1;
      keys[i].pressTime = millis();
    }
    else if ( val != LOW && keys[i].keyState == 1 ) //pressed and release.. new key.
    {
      if ( millis() - keys[i].pressTime  >= 50 )
      {
        keypress akey;
        akey.keyId = keys[i].keyId;
        akey.keyType = 0;
        addKeyPress( akey );
      }
      keys[i].keyState = 0;
    }
  }
};

