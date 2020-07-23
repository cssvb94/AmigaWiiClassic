#include <Arduino.h>
#include <Wire.h>
#include <WiiChuck.h>

Accessory wiicontr;

#define BTN_FIRE 2
#define BTN_JUMP 3
#define BTN_UP 4
#define BTN_DOWN 5
#define BTN_LEFT 6
#define BTN_RIGHT 7

/*
Amiga Joystick pinout
1 - Up          pin 4 green
2- Down         pin 5 blue
3 - Left        pin 6 purple
4 - Right       pin 7 grey
5 - NC          pin 
6 - BTN1        pin 2 white
7 - +5 VDC      pin 
8 - GROUND      pin 
9 - BTN2        pin 3 brown

SCL - green  - A4
SDA - yellow - A5
*/

void setup()
{
    // Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(BTN_FIRE, OUTPUT);
    pinMode(BTN_JUMP, OUTPUT);
    pinMode(BTN_UP, OUTPUT);
    pinMode(BTN_DOWN, OUTPUT);
    pinMode(BTN_LEFT, OUTPUT);
    pinMode(BTN_RIGHT, OUTPUT);

    digitalWrite(BTN_FIRE, HIGH);
    digitalWrite(BTN_JUMP, HIGH);

    digitalWrite(BTN_UP, HIGH);
    digitalWrite(BTN_DOWN, HIGH);
    digitalWrite(BTN_LEFT, HIGH);
    digitalWrite(BTN_RIGHT, HIGH);

    wiicontr.begin();
    if (wiicontr.type == Unknown)
    {
        /** If the device isn't auto-detected, set the type explicatly
         NUNCHUCK,
		 WIICLASSIC,
		 */
        wiicontr.type = WIICLASSIC;
    }
}

void loop()
{
    if (wiicontr.readData())
    {
        if (wiicontr.getButtonB() == 1 || wiicontr.getButtonX() == 1)
        {
            digitalWrite(BTN_FIRE, LOW);
            digitalWrite(LED_BUILTIN, HIGH);
            // Serial.println("Fire");
        }
        else
        {
            digitalWrite(BTN_FIRE, HIGH);
            digitalWrite(LED_BUILTIN, LOW);
        }

        if (wiicontr.getButtonA() == 1 || wiicontr.getButtonY() == 1)
            digitalWrite(BTN_JUMP, LOW);
        else
            digitalWrite(BTN_JUMP, HIGH);

        int xl = wiicontr.getJoyXLeft(); // 0-63
        int yl = 63 - wiicontr.getJoyYLeft();
        //
        int xr = wiicontr.getJoyXRight(); // 0 - 31
        int yr = wiicontr.getJoyYRight();

        if (wiicontr.getPadUp() == 1 || yl < 20 || yr > 25)
            digitalWrite(BTN_UP, LOW);
        else
            digitalWrite(BTN_UP, HIGH);

        if (wiicontr.getPadDown() == 1 || yl > 40 || yr < 8)
            digitalWrite(BTN_DOWN, LOW);
        else
            digitalWrite(BTN_DOWN, HIGH);

        if (wiicontr.getPadLeft() == 1 || xl < 20 || xr < 8)
            digitalWrite(BTN_LEFT, LOW);
        else
            digitalWrite(BTN_LEFT, HIGH);

        if (wiicontr.getPadRight() == 1 || xl > 40 || xr > 25)
            digitalWrite(BTN_RIGHT, LOW);
        else
            digitalWrite(BTN_RIGHT, HIGH);
    }
}