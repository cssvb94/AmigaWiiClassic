#pragma once

#include <Arduino.h>

#define ENABLE_FAST_IO

#ifdef ENABLE_FAST_IO
// https://github.com/greiman/DigitalIO
#include <DigitalIO.h>
#else
#define fastDigitalRead(x) digitalRead(x)
#define fastDigitalWrite(x, y) digitalWrite(x, y)
#define fastPinMode(x, y) pinMode(x, y)
#endif

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

#include <WiiChuck.h>
Accessory wiicontr;

// Button bits for CD32 mode
const byte BTN_BLUE = 1U << 0U;    // Blue Button
const byte BTN_RED = 1U << 1U;     // Red Button
const byte BTN_YELLOW = 1U << 2U;  // Yellow Button
const byte BTN_GREEN = 1U << 3U;   // Green Button
const byte BTN_FRONT_R = 1U << 4U; // Front Right Button
const byte BTN_FRONT_L = 1U << 5U; // Front Left Button
const byte BTN_START = 1U << 6U;   // Start/Pause Button

const byte BTN_FIRE = 2;
const byte BTN_JUMP = 3;
const byte BTN_UP = 4;
const byte BTN_DOWN = 5;
const byte BTN_LEFT = 6;
const byte BTN_RIGHT = 7;

#define ATTR_PACKED __attribute__((packed))

enum ATTR_PACKED State
{
    ST_NO_CONTROLLER, //!< No controller connected
    ST_FIRST_READ,    //!< First time the controller is read

    // Main functioning modes
    ST_JOYSTICK,      //!< Two-button joystick mode
    ST_MOUSE,         //!< Mouse mode
    ST_CD32,          //!< CD32-controller mode
    ST_JOYSTICK_TEMP, //!< Just come out of CD32 mode, will it last?

    // States to select mapping or go into programming mode
    ST_SELECT_HELD,         //!< Select being held
    ST_SELECT_AND_BTN_HELD, //!< Select + mapping button being held
    ST_ENABLE_MAPPING,      //!< Select + mapping button released, enable mapping

    // States for programming mode
    ST_WAIT_SELECT_RELEASE,          //!< Select released, entering programming mode
    ST_WAIT_BUTTON_PRESS,            //!< Programmable button pressed
    ST_WAIT_BUTTON_RELEASE,          //!< Programmable button released
    ST_WAIT_COMBO_PRESS,             //!< Combo pressed
    ST_WAIT_COMBO_RELEASE,           //!< Combo released
    ST_WAIT_SELECT_RELEASE_FOR_EXIT, //!< Wait for select to be released to go back to joystick mode

// States for factory reset
#ifndef DISABLE_FACTORY_RESET
    ST_FACTORY_RESET_WAIT_1,
    ST_FACTORY_RESET_WAIT_2,
    ST_FACTORY_RESET_PERFORM
#endif
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
