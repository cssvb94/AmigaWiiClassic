#pragma once
#include <Arduino.h>

#define ENABLE_FAST_IO
#ifdef ENABLE_FAST_IO
#include <DigitalIO.h>
#else
#define fastDigitalRead(x) digitalRead(x)
#define fastDigitalWrite(x, y) digitalWrite(x, y)
#define fastPinMode(x, y) pinMode(x, y)
#endif

#include <WiiChuck.h>

// #define ENABLE_SERIAL_DEBUG
#ifdef ENABLE_SERIAL_DEBUG
#include <avr/pgmspace.h>
typedef const __FlashStringHelper *FlashStr;
typedef const byte *PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper *>(s)

#define dstart(spd) Serial.begin(spd)
#define debug(...) Serial.print(__VA_ARGS__)
#define debugln(...) Serial.println(__VA_ARGS__)
#else
#define dstart(...)
#define debug(...)
#define debugln(...)
#endif

Accessory wiicontr;

// Button bits for CD32 mode
const byte BTN_BLUE = 1U << 0U;    // Blue Button
const byte BTN_RED = 1U << 1U;     // Red Button
const byte BTN_YELLOW = 1U << 2U;  // Yellow Button
const byte BTN_GREEN = 1U << 3U;   // Green Button
const byte BTN_FRONT_R = 1U << 4U; // Front Right Button
const byte BTN_FRONT_L = 1U << 5U; // Front Left Button
const byte BTN_START = 1U << 6U;   // Start/Pause Button

const byte TIMEOUT_CD32_MODE = 100U;

/*
Amiga Joystick pinout
1 - Up          pin 4 green
2 - Down        pin 5 blue
3 - Left        pin 6 purple
4 - Right       pin 7 grey
5 - Joy/Pad     pin 2 orange
6 - BTN1        pin 3 white
7 - +5 VDC      pin 
8 - GROUND      pin 
9 - BTN2        pin 8 brown

SCL - green  - A4
SDA - yellow - A5
*/

/** Controller mode input pin
 *  
 * This pin switches between Amiga (HIGH) and CD32 (LOW) mode.
 * 
 * It also triggers the loading of the button status shift register.
 */
const byte PIN_PADMODE = 2; // Amiga Pin 5

const byte BTN_FIRE = 3; // BTN1 - Arduino INT1
const byte BTN_UP = 4;
const byte BTN_DOWN = 5;
const byte BTN_LEFT = 6;
const byte BTN_RIGHT = 7;
const byte BTN_JUMP = 8; // BTN2

/** Shift register output pin for CD32 mode
 * 
 * When in CD32 mode, button status is saved to an 8-bit register that gets
 * shifted out one bit at a time through this pin.
 */
const byte PIN_BTNREGOUT = BTN_JUMP;

/** Shift register clock input pin for CD32 mode
 * 
 * The shifting is clocked by rising edges on this pin.
 */
const byte PIN_BTNREGCLK = BTN_FIRE;

#define ATTR_PACKED __attribute__((packed))

enum ATTR_PACKED State
{
    ST_NO_CONTROLLER, // No controller connected
    ST_JOYSTICK,      // Two-button joystick mode
    ST_CD32,          // CD32-controller mode
};

struct TwoButtonJoystick
{
    boolean up : 1;    // Up/Forward direction
    boolean down : 1;  // Down/Backwards direction
    boolean left : 1;  // Left direction
    boolean right : 1; // Right direction
    boolean b1 : 1;    // Button 1
    boolean b2 : 1;    // Button 2
};

typedef void (*JoyMappingFunc)(TwoButtonJoystick &j);

volatile byte *buttonsLive = &GPIOR0;
volatile byte *isrButtons = &GPIOR1;

volatile State *state = reinterpret_cast<volatile State *>(&GPIOR2);
