#include "main_config.h"

void mapJoystickNormal(TwoButtonJoystick &j);
JoyMappingFunc joyMappingFunc = mapJoystickNormal;

// This shall be called when CD32 mode is exited.
inline void suspendClockInterrupt()
{
    EIMSK &= ~(1 << INT1);
}

// This shall be called when CD32 mode is entered.
inline void restoreClockInterrupt()
{
    EIFR |= (1 << INTF1); // Clear any pending interrupts
    EIMSK |= (1 << INT1);
}

inline void buttonPress(byte pin)
{
    switch (pin)
    {
    case BTN_UP:
        fastDigitalWrite(BTN_UP, LOW);
        break;
    case BTN_DOWN:
        fastDigitalWrite(BTN_DOWN, LOW);
        break;
    case BTN_LEFT:
        fastDigitalWrite(BTN_LEFT, LOW);
        break;
    case BTN_RIGHT:
        fastDigitalWrite(BTN_RIGHT, LOW);
        break;
    case BTN_FIRE:
        fastDigitalWrite(BTN_FIRE, LOW);
        break;
    case BTN_JUMP:
        fastDigitalWrite(BTN_JUMP, LOW);
        break;
    }
}

inline void buttonRelease(byte pin)
{
    // Switch to Hi-Z
    switch (pin)
    {
    case BTN_UP:
        fastDigitalWrite(BTN_UP, HIGH);
        break;
    case BTN_DOWN:
        fastDigitalWrite(BTN_DOWN, HIGH);
        break;
    case BTN_LEFT:
        fastDigitalWrite(BTN_LEFT, HIGH);
        break;
    case BTN_RIGHT:
        fastDigitalWrite(BTN_RIGHT, HIGH);
        break;
    case BTN_FIRE:
        fastDigitalWrite(BTN_FIRE, HIGH);
        break;
    case BTN_JUMP:
        fastDigitalWrite(BTN_JUMP, HIGH);
        break;
    }
}

void onClockEdge()
{
    if (!(*isrButtons & 0x01))
    {
        fastDigitalWrite(PIN_BTNREGOUT, LOW);
    }
    else
    {
        fastDigitalWrite(PIN_BTNREGOUT, HIGH);
    }
    *isrButtons >>= 1U;
}

void onPadModeChange()
{
    if (fastDigitalRead(PIN_PADMODE) == LOW)
    {
        debugln(F("CD32 mode"));
        fastPinMode(PIN_BTNREGCLK, INPUT);
        fastPinMode(PIN_BTNREGOUT, OUTPUT);
        if (!(*buttonsLive & 0x01))
        {
            fastDigitalWrite(PIN_BTNREGOUT, LOW);
        }
        else
        {
            fastDigitalWrite(PIN_BTNREGOUT, HIGH);
        }

        *isrButtons = *buttonsLive >> 1U;
        restoreClockInterrupt();

        *state = ST_CD32;
        fastDigitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        debugln(F("Joystick mode"));
        if (!(*buttonsLive & BTN_RED))
        {
            buttonPress(BTN_FIRE);
        }
        else
        {
            buttonRelease(BTN_FIRE);
        }

        if (!(*buttonsLive & BTN_BLUE))
        {
            buttonPress(BTN_JUMP);
        }
        else
        {
            buttonRelease(BTN_JUMP);
        }
        // Disable INT1
        suspendClockInterrupt();
        // Set state to ST_JOYSTICK_TEMP
        *state = ST_JOYSTICK;
        fastDigitalWrite(LED_BUILTIN, LOW);
    }
}

inline void enableCD32Trigger()
{
    noInterrupts();

    /* Clear any pending interrupts, see
	 * https://github.com/arduino/ArduinoCore-avr/issues/244
	 */
    EIFR |= (1 << INTF0);

    // Enable interrupt 0 (i.e.: on pin 2)...
    attachInterrupt(digitalPinToInterrupt(PIN_PADMODE), onPadModeChange, CHANGE);

    // ... and interrupt 1 (pin 3)
    attachInterrupt(digitalPinToInterrupt(PIN_BTNREGCLK), onClockEdge, RISING);

    // ... but keep the latter on hold
    suspendClockInterrupt();
    interrupts();
}

inline void disableCD32Trigger()
{
    // Disable both interrupts, as this might happen halfway during a shift
    noInterrupts();
    detachInterrupt(digitalPinToInterrupt(PIN_PADMODE));
    detachInterrupt(digitalPinToInterrupt(PIN_BTNREGCLK));
    interrupts();
}

void handleJoystickButtons(const TwoButtonJoystick &j)
{
    noInterrupts();
    // if (*state == ST_JOYSTICK)
    // {
    if (j.b1)
    {
        buttonPress(BTN_FIRE);
    }
    else
    {
        buttonRelease(BTN_FIRE);
    }

    if (j.b2)
    {
        buttonPress(BTN_JUMP);
    }
    else
    {
        buttonRelease(BTN_JUMP);
    }
    // }
    interrupts();
}

void handleCD32PADButtons()
{
    // Use the same logic as in handleJoystickButtons()
    noInterrupts();
    // if (*state == ST_JOYSTICK)
    if (*state == ST_CD32)
    {
        /* Relying on buttonsLive guarantees we do the right thing even when
		 * useAlternativeCd32Mapping is true
		 */
        if (!(*buttonsLive & BTN_RED))
        {
            buttonPress(BTN_FIRE);
        }
        else
        {
            buttonRelease(BTN_FIRE);
        }

        if (!(*buttonsLive & BTN_BLUE))
        {
            buttonPress(BTN_JUMP);
        }
        else
        {
            buttonRelease(BTN_JUMP);
        }
    }
    interrupts();
}

void handleJoystickDirections(TwoButtonJoystick &j)
{
    // Call button mapping function
    joyMappingFunc(j);

    // Make mapped buttons affect the actual pins
    if (j.up)
    {
        buttonPress(BTN_UP);
    }
    else
    {
        buttonRelease(BTN_UP);
    }

    if (j.down)
    {
        buttonPress(BTN_DOWN);
    }
    else
    {
        buttonRelease(BTN_DOWN);
    }

    if (j.left)
    {
        buttonPress(BTN_LEFT);
    }
    else
    {
        buttonRelease(BTN_LEFT);
    }

    if (j.right)
    {
        buttonPress(BTN_RIGHT);
    }
    else
    {
        buttonRelease(BTN_RIGHT);
    }

    byte buttonsTmp = 0xFF;

    if (wiicontr.getButtonX() == 1)
        buttonsTmp &= ~BTN_GREEN;

    if (wiicontr.getButtonY() == 1)
        buttonsTmp &= ~BTN_RED;

    if (wiicontr.getButtonB() == 1)
        buttonsTmp &= ~BTN_BLUE;

    if (wiicontr.getButtonA() == 1)
        buttonsTmp &= ~BTN_YELLOW;

    if (wiicontr.getButtonPlus() == 1)
        buttonsTmp &= ~BTN_START;

    if (wiicontr.getButtonZLeft() == 1)
        buttonsTmp &= ~BTN_FRONT_L;

    if (wiicontr.getButtonZRight() == 1)
        buttonsTmp &= ~BTN_FRONT_R;

    // Atomic operation, interrupt either happens before or after this
    *buttonsLive = buttonsTmp;
}

void stateMachine()
{
    static unsigned long stateEnteredTime = 0;
    TwoButtonJoystick j = {false, false, false, false, false, false};
    if (*state == ST_NO_CONTROLLER)
    {
        if (wiicontr.readData())
        {
            *state = ST_JOYSTICK;
            enableCD32Trigger();
            debugln(F("Connection established"));
        }
        else
        {
            *state = ST_NO_CONTROLLER;
            *buttonsLive = 0x7F; // No ID sequence, all buttons released
            disableCD32Trigger();
            debugln(F("Controller connection lost"));
        }
    }

    switch (*state)
    {
    case ST_NO_CONTROLLER:
        // *state = ST_JOYSTICK;
        break;
    case ST_JOYSTICK:
        handleJoystickDirections(j);
        handleJoystickButtons(j);
        enableCD32Trigger();
        break;
    case ST_CD32:
    {
        handleJoystickDirections(j);
        handleCD32PADButtons();
        if (stateEnteredTime == 0)
        {
            // State was just entered
            stateEnteredTime = millis();
        }
        else if (millis() - stateEnteredTime > TIMEOUT_CD32_MODE)
        {
            // CD32 mode was exited once for all
            stateEnteredTime = 0;
            *state = ST_JOYSTICK;
        }
        break;
    }
    default:
        // Shouldn't be reached
        break;
    }
}

void mapJoystickNormal(TwoButtonJoystick &j)
{
    if (!wiicontr.readData())
    {
        *state = ST_NO_CONTROLLER;
        fastDigitalWrite(LED_BUILTIN, LOW);
        return;
    }

    // Use both analog axes
    int xl = wiicontr.getJoyXLeft(); // 0-63
    int yl = 63 - wiicontr.getJoyYLeft();
    //
    int xr = wiicontr.getJoyXRight(); // 0 - 31
    int yr = wiicontr.getJoyYRight();

    j.up |= (wiicontr.getPadUp() == 1 || yl < 20 || yr > 25);
    j.down |= (wiicontr.getPadDown() == 1 || yl > 40 || yr < 8);
    j.left |= (wiicontr.getPadLeft() == 1 || xl < 20 || xr < 8);
    j.right |= (wiicontr.getPadRight() == 1 || xl > 40 || xr > 25);

    // [B] is button 1 - Fire
    j.b1 = (wiicontr.getButtonB() == 1);
    // [A] is button 2 - Jump
    j.b2 = (wiicontr.getButtonA() == 1);
}

void setup()
{
    dstart(115200);

    fastPinMode(LED_BUILTIN, OUTPUT);
    fastDigitalWrite(LED_BUILTIN, LOW);

    fastPinMode(BTN_FIRE, OUTPUT);
    fastPinMode(BTN_JUMP, OUTPUT);
    fastPinMode(BTN_UP, OUTPUT);
    fastPinMode(BTN_DOWN, OUTPUT);
    fastPinMode(BTN_LEFT, OUTPUT);
    fastPinMode(BTN_RIGHT, OUTPUT);

    fastDigitalWrite(BTN_FIRE, HIGH);
    fastDigitalWrite(BTN_JUMP, HIGH);
    fastDigitalWrite(BTN_UP, HIGH);
    fastDigitalWrite(BTN_DOWN, HIGH);
    fastDigitalWrite(BTN_LEFT, HIGH);
    fastDigitalWrite(BTN_RIGHT, HIGH);

    /* 
    * This pin tells us when to toggle in/out of CD32 mode,
	* and it will always be an input 
    */
    fastPinMode(PIN_PADMODE, INPUT_PULLUP);

    wiicontr.begin();
    if (wiicontr.type == Unknown)
    {
        // If the device isn't auto-detected, set the type explicatly
        wiicontr.type = WIICLASSIC;
        debugln(F("Wii controller unknown. Set to Wii Classic"));
    }
    *state = ST_NO_CONTROLLER;
}

void loop()
{
    stateMachine();
}

// void loop()
// {
//     if (wiicontr.readData())
//     {
//         if (wiicontr.getButtonB() == 1 || wiicontr.getButtonX() == 1)
//         {
//             fastDigitalWrite(BTN_FIRE, LOW);
//             fastDigitalWrite(LED_BUILTIN, HIGH);
//         }
//         else
//         {
//             fastDigitalWrite(BTN_FIRE, HIGH);
//             fastDigitalWrite(LED_BUILTIN, LOW);
//         }

//         if (wiicontr.getButtonA() == 1 || wiicontr.getButtonY() == 1)
//             fastDigitalWrite(BTN_JUMP, LOW);
//         else
//             fastDigitalWrite(BTN_JUMP, HIGH);

//         int xl = wiicontr.getJoyXLeft(); // 0-63
//         int yl = 63 - wiicontr.getJoyYLeft();
//         //
//         int xr = wiicontr.getJoyXRight(); // 0 - 31
//         int yr = wiicontr.getJoyYRight();

//         if (wiicontr.getPadUp() == 1 || yl < 20 || yr > 25)
//             fastDigitalWrite(BTN_UP, LOW);
//         else
//             fastDigitalWrite(BTN_UP, HIGH);

//         if (wiicontr.getPadDown() == 1 || yl > 40 || yr < 8)
//             fastDigitalWrite(BTN_DOWN, LOW);
//         else
//             fastDigitalWrite(BTN_DOWN, HIGH);

//         if (wiicontr.getPadLeft() == 1 || xl < 20 || xr < 8)
//             fastDigitalWrite(BTN_LEFT, LOW);
//         else
//             fastDigitalWrite(BTN_LEFT, HIGH);

//         if (wiicontr.getPadRight() == 1 || xl > 40 || xr > 25)
//             fastDigitalWrite(BTN_RIGHT, LOW);
//         else
//             fastDigitalWrite(BTN_RIGHT, HIGH);
//     }
// }