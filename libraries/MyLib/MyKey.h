#ifndef _MYKEY_H_
#define _MYKEY_H_

#include <Arduino.h>


typedef struct {
  byte keyId;
  byte keyType;
} keypress;

typedef struct {
  byte keyId;
  byte pinIdx;
  byte keyState; //0 release 1 pressed
  unsigned long pressTime; //the time key been pressed
} keydef;


class MyKey {
    public:
        MyKey();
        bool keyAvailable();
        void initKey( );
        void checkKey();
        void addKeyPress( keypress aKey);
        keypress *getKeyPress();

        byte addKey( byte keyId, byte pinIdx);
    private:
        keydef keys[4];
        byte keyCount;
        keypress keybuff[5];
        byte keybuffSize = 0;
};


#endif /* _MYKEY_H_ */
