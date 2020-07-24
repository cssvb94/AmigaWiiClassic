```cpp
#include <Arduino.h>
#include <WiiChuck.h>

Accessory wii_classic;

const byte PIN_UP = 4;    //!< Amiga Pin 1
const byte PIN_DOWN = 5;  //!< Amiga Pin 2
const byte PIN_LEFT = 6;  //!< Amiga Pin 3
const byte PIN_RIGHT = 7; //!< Amiga Pin 4
const byte PIN_BTN1 = 3;  //!< Amiga Pin 6
const byte PIN_BTN2 = 8;  //!< Amiga Pin 9

/** \brief Controller mode input pin
 *  
 * This pin switches between Amiga (HIGH) and CD32 (LOW) mode.
 * 
 * It also triggers the loading of the button status shift register.
 */
const byte PIN_PADMODE = 2; // Amiga Pin 5

/** \brief Shift register output pin for CD32 mode
 * 
 * When in CD32 mode, button status is saved to an 8-bit register that gets
 * shifted out one bit at a time through this pin.
 */
const byte PIN_BTNREGOUT = PIN_BTN2;

/** \brief Shift register clock input pin for CD32 mode
 * 
 * The shifting is clocked by rising edges on this pin.
 */
const byte PIN_BTNREGCLK = PIN_BTN1;

/** \brief CD32 mode timeout
 * 
 * Normal joystick mode will be entered if PIN_PADMODE is not toggled for this
 * amount of milliseconds.
 * 
 * Keep in mind that in normal conditions it is toggled once per frame (i.e.
 * every ~20 ms).
 */
const byte TIMEOUT_CD32_MODE = 100U;

/** \brief Dead zone for analog sticks
 *  
 * If the analog stick moves less than this value from the center position, it
 * is considered still.
 * 
 * \sa ANALOG_IDLE_VALUE
 */
const byte ANALOG_IDLE_VALUE = 0U;
const byte ANALOG_DEAD_ZONE = 50U;

#define ENABLE_FAST_IO

#ifdef ENABLE_FAST_IO
// https://github.com/greiman/DigitalIO
#include <DigitalIO.h>
#else
#define fastDigitalRead(x) digitalRead(x)
#define fastDigitalWrite(x, y) digitalWrite(x, y)
#define fastPinMode(x, y) pinMode(x, y)
#endif

//! \name Button bits for CD32 mode
//! @{
const byte BTN_BLUE = 1U << 0U;    //!< \a Blue Button
const byte BTN_RED = 1U << 1U;     //!< \a Red Button
const byte BTN_YELLOW = 1U << 2U;  //!< \a Yellow Button
const byte BTN_GREEN = 1U << 3U;   //!< \a Green Button
const byte BTN_FRONT_R = 1U << 4U; //!< \a Front \a Right Button
const byte BTN_FRONT_L = 1U << 5U; //!< \a Front \a Left Button
const byte BTN_START = 1U << 6U;   //!< \a Start/Pause Button
//! @}

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

/** \brief Structure representing a standard 2-button Atari-style joystick
 * 
 * This is used for gathering button presses according to different button
 * mappings.
 * 
 * True means pressed.
 * 
 * We use bitfields so that it all fits in a single byte.
 */
struct TwoButtonJoystick
{
    boolean up : 1;    //!< Up/Forward direction
    boolean down : 1;  //!< Down/Backwards direction
    boolean left : 1;  //!< Left direction
    boolean right : 1; //!< Right direction
    boolean b1 : 1;    //!< Button 1
    boolean b2 : 1;    //!< Button 2
};

/** \brief Joystick mapping function
 * 
 * This represents a function that should inspect the buttons currently being
 * pressed on the PSX controller and somehow map them to a #TwoButtonJoystick to
 * be sent to the DB-9 port.
 */
typedef void (*JoyMappingFunc)(TwoButtonJoystick &j);

// Default button mapping function prototype for initialization of the following
void mapJoystickNormal(TwoButtonJoystick &j);

//! \brief Joystick mapping function currently in effect
JoyMappingFunc joyMappingFunc = mapJoystickNormal;

/** \brief Button register for CD32 mode being updated
 * 
 * This shall be updated as often as possible, and is what gets sampled when we
 * get a falling edge on #PIN_PADMODE.
 * 
 * 0 means pressed, MSB must be 1 for the ID sequence.
 * 
 * We keep this in \a GPIOR0 for faster ISR code. Note that GPIOR0 is one of the
 * lowest 64 registers on ATmega88/168/328's, thus it can be used with
 * SBI/CBI/SBIC/SBIS. I don't know if GCC takes advantage of this, but we do in
 * the \a asm_isr branch.
 * 
 * \hideinitializer
 */
volatile byte *buttonsLive = &GPIOR0;

/** \brief Button register for CD32 mode currently being shifted out
 * 
 * This is where #buttonsLive gets copied when it is sampled.
 * 
 * 0 means pressed, etc.
 * 
 * We keep this in \a GPIOR1 for faster ISR code.
 * 
 * \hideinitializer
 */
volatile byte *isrButtons = &GPIOR1;

#define ATTR_PACKED __attribute__((packed))

/** \brief State machine states
 * 
 * Possible states for the internal state machine that drives the whole thing.
 * 
 * Note that we make this \a packed so that it fits in a single byte.
 */
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

/** \brief Current state of the internal state machine
 * 
 * We start out as a simple joystick.
 * 
 * We keep this in \a GPIOR2 for faster access and slimmer code.
 * 
 * \hideinitializer
 */
volatile State *state = reinterpret_cast<volatile State *>(&GPIOR2);

/** \brief Disable the ISR servicing rising clock edges
 * 
 * This shall be called when CD32 mode is exited.
 */
inline void suspendClockInterrupt()
{
    EIMSK &= ~(1 << INT1);
}

/** \brief Enable the ISR servicing rising clock edges
 * 
 * This shall be called when CD32 mode is entered.
 */
inline void restoreClockInterrupt()
{
    EIFR |= (1 << INTF1); // Clear any pending interrupts
    EIMSK |= (1 << INT1);
}

inline void buttonPress(byte pin)
{
    switch (pin)
    {
    case PIN_UP:
        fastPinMode(PIN_UP, OUTPUT);
        break;
    case PIN_DOWN:
        fastPinMode(PIN_DOWN, OUTPUT);
        break;
    case PIN_LEFT:
        fastPinMode(PIN_LEFT, OUTPUT);
        break;
    case PIN_RIGHT:
        fastPinMode(PIN_RIGHT, OUTPUT);
        break;
    case PIN_BTN1:
        fastPinMode(PIN_BTN1, OUTPUT);
        break;
    case PIN_BTN2:
        fastPinMode(PIN_BTN2, OUTPUT);
        break;
    }
}

inline void buttonRelease(byte pin)
{
    // Switch to Hi-Z
    switch (pin)
    {
    case PIN_UP:
        fastPinMode(PIN_UP, INPUT);
        break;
    case PIN_DOWN:
        fastPinMode(PIN_DOWN, INPUT);
        break;
    case PIN_LEFT:
        fastPinMode(PIN_LEFT, INPUT);
        break;
    case PIN_RIGHT:
        fastPinMode(PIN_RIGHT, INPUT);
        break;
    case PIN_BTN1:
        fastPinMode(PIN_BTN1, INPUT);
        break;
    case PIN_BTN2:
        fastPinMode(PIN_BTN2, INPUT);
        break;
    }
}

/** \brief Map horizontal movements of the left analog stick to a
 *         #TwoButtonJoystick
 * 
 * The stick is not considered moved if it moves less than #ANALOG_DEAD_ZONE.
 * 
 * \param[out] j Mapped joystick status
 */
void mapAnalogStickHorizontal(TwoButtonJoystick &j)
{
    int lx = wii_classic.getJoyXLeft();
    int8_t deltaLX = lx - ANALOG_IDLE_VALUE; // --> -127 ... +128
    j.left = deltaLX < -ANALOG_DEAD_ZONE;
    j.right = deltaLX > +ANALOG_DEAD_ZONE;

#ifdef ENABLE_SERIAL_DEBUG
    static int oldx = -1000;
    if (deltaLX != oldx)
    {
        debug(F("L Analog X = "));
        debugln(deltaLX);
        oldx = deltaLX;
    }
#endif
}

/** \brief Map vertical movements of the left analog stick to a
 *         #TwoButtonJoystick
 * 
 * The stick is not considered moved if it moves less than #ANALOG_DEAD_ZONE.
 * 
 * \param[out] j Mapped joystick status
 */
void mapAnalogStickVertical(TwoButtonJoystick &j)
{
    int ly = wii_classic.getJoyYLeft();
    int8_t deltaLY = ly - ANALOG_IDLE_VALUE;
    j.up = deltaLY < -ANALOG_DEAD_ZONE;
    j.down = deltaLY > +ANALOG_DEAD_ZONE;

#ifdef ENABLE_SERIAL_DEBUG
    static int oldy = -1000;
    if (deltaLY != oldy)
    {
        debug(F("L Analog Y = "));
        debugln(deltaLY);
        oldy = deltaLY;
    }
#endif
}

void mapJoystickNormal(TwoButtonJoystick &j)
{
    // Use both analog axes
    mapAnalogStickHorizontal(j);
    mapAnalogStickVertical(j);

    // D-Pad is fully functional as well
    j.up |= wii_classic.getPadUp() == 1;
    j.down |= wii_classic.getPadDown() == 1;
    j.left |= wii_classic.getPadLeft() == 1;
    j.right |= wii_classic.getPadRight() == 1;

    // Square is button 1
    j.b1 = wii_classic.getButtonY() == 1;

    // Cross is button 2
    j.b2 = wii_classic.getButtonB() == 1;
}

#ifndef SUPER_OPTIMIZE
void onPadModeChange()
{
#else
ISR(INT0_vect)
{
#endif
#ifdef ENABLE_INSTRUMENTATION
    fastDigitalToggle(PIN_INTERRUPT_TIMING);
#endif
    if (fastDigitalRead(PIN_PADMODE) == LOW)
    {
        // Switch to CD32 mode
        debugln(F("Joystick -> CD32"));

        // Immediately disable output on clock pin
        fastPinMode(PIN_BTNREGCLK, INPUT);

        // Output status of first button as soon as possible
        fastPinMode(PIN_BTNREGOUT, OUTPUT);
        if (!(*buttonsLive & 0x01))
        {
            fastDigitalWrite(PIN_BTNREGOUT, LOW);
        }
        else
        {
            fastDigitalWrite(PIN_BTNREGOUT, HIGH);
        }

        /* Sample input values, they will be shifted out on subsequent clock
		 * inputs.
		 * 
		 * At this point MSB must be 1 for ID sequence. Then it will be zeroed
		 * by the shift. This will report non-existing buttons 8 as released and
		 * 9 as pressed as required by the ID sequence.
		 */
#ifndef SUPER_OPTIMIZE
        *isrButtons = *buttonsLive >> 1U;
#else
        asm volatile(
            "lsr %0\n\t"
            : "=r"(*isrButtons)
            : "r"(*buttonsLive));
#endif

        // Enable INT1, i.e. interrupt on clock edges
        restoreClockInterrupt();

        // Set state to ST_CD32
        *state = ST_CD32;

#ifdef ENABLE_INSTRUMENTATION
        fastDigitalToggle(PIN_CD32MODE);
#endif
    }
    else
    {
        // Switch back to joystick mode
        debugln(F("CD32 -> Joystick"));

        /* Set pin directions and set levels according to buttons, as waiting
		 * for the main loop to do it takes too much time (= a few ms), for some
		 * reason
		 */
        /* PIN_BTN1 (aka PIN_BTNREGCLK) was an INPUT and it must either remain
		 * an input or become a low output, so this should be enough
		 */
        if (!(*buttonsLive & BTN_RED))
        {
            buttonPress(PIN_BTN1);
        }
        else
        {
            buttonRelease(PIN_BTN1);
        }

        /* PIN_BTN2 (aka PIN_BTNREGOUT) was an output, either high or low, which
		 * means it might turn into a pulled-up input
		 */
        if (!(*buttonsLive & BTN_BLUE))
        {
            // To LOW OUTPUT
            buttonPress(PIN_BTN2);
        }
        else
        {
            // To INPUT
            buttonRelease(PIN_BTN2);
        }
        fastDigitalWrite(PIN_BTN2, LOW); /* Disable pull-up, don't do it before
											 * to avoid a spurious 0V state when
											 * going from output high to input
											 */

        // Disable INT1
        suspendClockInterrupt();

        // Set state to ST_JOYSTICK_TEMP
        *state = ST_JOYSTICK_TEMP;

#ifdef ENABLE_INSTRUMENTATION
        fastDigitalToggle(PIN_CD32MODE);
#endif
    }

#ifdef ENABLE_INSTRUMENTATION
    fastDigitalToggle(PIN_INTERRUPT_TIMING);
#endif
}

/** \brief Update the primary output fire button pins when in CD32 mode
 * 
 * This functions updates the status of the 2 main fire buttons inbetween two
 * consecutive CD32-style readings. It might seem counter-intuitive, but many
 * games (including lowlevel.library, I think!) read the state of the two main
 * buttons this way, rather than extracting them from the full 7-button reply,
 * don't ask me why. See \a Banshee, for instance.
 * 
 * So we need a different function that does not perform button mapping as when
 * in 2-button joystick mode, but that rather uses the current CD32 mapping.
 * This means it shall also be coherent with what onPadModeChange() is doing
 * when PIN_PADMODE goes high.
 * 
 * Of course, this function shall only be called when state is ST_CD32.
 */
void handleJoystickButtonsTemp()
{
    // Use the same logic as in handleJoystickButtons()
    noInterrupts();

    if (*state == ST_JOYSTICK_TEMP)
    {
        /* Relying on buttonsLive guarantees we do the right thing even when
		 * useAlternativeCd32Mapping is true
		 */
        if (!(*buttonsLive & BTN_RED))
        {
            buttonPress(PIN_BTN1);
        }
        else
        {
            buttonRelease(PIN_BTN1);
        }

        if (!(*buttonsLive & BTN_BLUE))
        {
            buttonPress(PIN_BTN2);
        }
        else
        {
            buttonRelease(PIN_BTN2);
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
        buttonPress(PIN_UP);
    }
    else
    {
        buttonRelease(PIN_UP);
    }

    if (j.down)
    {
        buttonPress(PIN_DOWN);
    }
    else
    {
        buttonRelease(PIN_DOWN);
    }

    if (j.left)
    {
        buttonPress(PIN_LEFT);
    }
    else
    {
        buttonRelease(PIN_LEFT);
    }

    if (j.right)
    {
        buttonPress(PIN_RIGHT);
    }
    else
    {
        buttonRelease(PIN_RIGHT);
    }

    /* Map buttons, working on a temporary variable to avoid the sampling
	 * interrupt to happen while we are filling in button statuses and catch a
	 * value that has not yet been fully populated.
	 *
	 * Note that 0 means pressed and that MSB must be 1 for the ID
	 * sequence.
	 */
    byte buttonsTmp = 0xFF;

    if (wii_classic.getButtonX() == 1)
        buttonsTmp &= ~BTN_GREEN;

    if (wii_classic.getButtonY() == 1)
        buttonsTmp &= ~BTN_RED;

    if (wii_classic.getButtonB() == 1)
        buttonsTmp &= ~BTN_BLUE;

    if (wii_classic.getButtonA() == 1)
        buttonsTmp &= ~BTN_YELLOW;

    if (wii_classic.getButtonPlus() == 1)
        buttonsTmp &= ~BTN_START;

    if (wii_classic.getButtonZLeft() == 1)
        buttonsTmp &= ~BTN_FRONT_L;

    if (wii_classic.getButtonZRight() == 1)
        buttonsTmp &= ~BTN_FRONT_R;

    // Atomic operation, interrupt either happens before or after this
    *buttonsLive = buttonsTmp;
}

/** \brief ISR servicing rising edges on #PIN_BTNREGCLK
 * 
 * Called on clock pin rising, this function shall shift out next bit.
 */
#ifndef SUPER_OPTIMIZE
void onClockEdge()
{
#else
ISR(INT1_vect)
{
#endif
#ifdef ENABLE_INSTRUMENTATION
    fastDigitalToggle(PIN_INTERRUPT_TIMING);
#endif

    if (!(*isrButtons & 0x01))
    {
        fastDigitalWrite(PIN_BTNREGOUT, LOW);
    }
    else
    {
        fastDigitalWrite(PIN_BTNREGOUT, HIGH);
    }

#ifndef SUPER_OPTIMIZE
    *isrButtons >>= 1U; /* Again, non-existing button 10 will be reported as
						 * pressed for the ID sequence
						 */
#else
    asm volatile(
        "lsr %0\n\t"
        : "=r"(*isrButtons)
        : "0"(*isrButtons));
#endif

#ifdef ENABLE_INSTRUMENTATION
    fastDigitalToggle(PIN_INTERRUPT_TIMING);
#endif
}

/** \brief Enable CD32 controller support
 * 
 * CD32 mode is entered automatically whenever a HIGH level is detected on
 * #PIN_PADMODE, after this function has been called.
 */
inline void enableCD32Trigger()
{
    noInterrupts();

    /* Clear any pending interrupts, see
	 * https://github.com/arduino/ArduinoCore-avr/issues/244
	 */
    EIFR |= (1 << INTF0);

#ifndef SUPER_OPTIMIZE
    // Enable interrupt 0 (i.e.: on pin 2)...
    attachInterrupt(digitalPinToInterrupt(PIN_PADMODE), onPadModeChange, CHANGE);

    // ... and interrupt 1 (pin 3)
    attachInterrupt(digitalPinToInterrupt(PIN_BTNREGCLK), onClockEdge, RISING);

    // ... but keep the latter on hold
    suspendClockInterrupt();
#else
    EIMSK |= (1 << INT0);
#endif

    interrupts();
}

/** \brief Disable CD32 controller support
 * 
 * CD32 mode will no longer be entered automatically after this function has
 * been called.
 */
inline void disableCD32Trigger()
{
    // Disable both interrupts, as this might happen halfway during a shift
#ifndef SUPER_OPTIMIZE
    noInterrupts();
    detachInterrupt(digitalPinToInterrupt(PIN_PADMODE));
    detachInterrupt(digitalPinToInterrupt(PIN_BTNREGCLK));
    interrupts();
#else
    EIMSK &= ~((1 << INT1) | (1 << INT0));
#endif
}

void setup()
{
    dstart(115200);
    debugln(F("Starting up..."));

    wii_classic.begin();

    if (wii_classic.type == Unknown)
    {
        /** If the device isn't auto-detected, set the type explicatly
		 NUNCHUCK,
		 WIICLASSIC,
		 GuitarHeroController,
		 GuitarHeroWorldTourDrums,
		 DrumController,
		 DrawsomeTablet,
		 Turntable
		 */
        wii_classic.type = WIICLASSIC;
    }

    /* This pin tells us when to toggle in/out of CD32 mode, and it will always
	 * be an input
	 */
    fastPinMode(PIN_PADMODE, INPUT_PULLUP);

#ifdef SUPER_OPTIMIZE
    /* Prepare interrupts: INT0 is triggered by pin 2, i.e. PIN_PADMODE, so it
	 * must be triggered on CHANGE.
	 */
    EICRA |= (1 << ISC00);
    EICRA &= ~(1 << ISC01); // Probably redundant

    /* INT1 is triggered by pin 3, i.e. PIN_BTNREGCLK/PIN_BTN1, and we want that
	 * triggered by RISING edges. Actually we should care about falling edges,
	 * but since it will take us some time to react to the interrupt, we start
	 * in advance ;).
	 */
    EICRA |= (1 << ISC11) | (1 << ISC10);

    // Interrupts are not activated here, preparation is enough :)
#endif

#ifdef ENABLE_INSTRUMENTATION
    // Prepare pins for instrumentation
    fastPinMode(PIN_INTERRUPT_TIMING, OUTPUT);
    fastPinMode(PIN_CD32MODE, OUTPUT);
#endif
}

/** \brief Update the output fire button pins when in joystick mode
 * 
 * This functions updates the status of the 2 fire button pins of the DB-9 port.
 * It shall only be called when state is ST_JOYSTICK.
 *
 * \param[in] j Mapped joystick status, as returned by
 *              handleJoystickDirections().
 */
void handleJoystickButtons(const TwoButtonJoystick &j)
{
    /* If the interrupt that switches us to CD32 mode is
	 * triggered while we are here we might end up setting pin states after
	 * we should have relinquished control of the pins, so let's avoid this
	 * disabling interrupts, we will handle them in a few microseconds.
	 */
    noInterrupts();

    /* Ok, this breaks the state machine abstraction a bit, but we *have* to do
	 * this check now, as the interrupt that makes us switch to ST_CD32 might
	 * have occurred after this function was called but before we disabled
	 * interrupts, and we absolutely have to avoid modifying pin
	 * directions/states if the ISR has already been called.
	 */
    if (*state == ST_JOYSTICK)
    {
        if (j.b1)
        {
            buttonPress(PIN_BTN1);
        }
        else
        {
            buttonRelease(PIN_BTN1);
        }

        if (j.b2)
        {
            buttonPress(PIN_BTN2);
        }
        else
        {
            buttonRelease(PIN_BTN2);
        }
    }

    interrupts();
}

void stateMachine()
{
    // static unsigned long lastPoll = 0;
    static unsigned long stateEnteredTime = 0;
    TwoButtonJoystick j = {false, false, false, false, false, false};
    if (*state != ST_NO_CONTROLLER)
    {
        if (wii_classic.readData())
        {
            debugln(F("Controller connection lost"));
            *state = ST_NO_CONTROLLER;
            *buttonsLive = 0x7F; // No ID sequence, all buttons released
        }
        // lastPoll = millis();
    }

    switch (*state)
    {
    case ST_NO_CONTROLLER:
        *state = ST_JOYSTICK;
        break;
    case ST_JOYSTICK:
        handleJoystickDirections(j);
        handleJoystickButtons(j);
        break;
    case ST_CD32:
    {
        handleJoystickDirections(j);
        handleJoystickButtonsTemp();
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
    case ST_JOYSTICK_TEMP:
    {
        handleJoystickDirections(j);
        handleJoystickButtonsTemp();

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

void loop()
{

    stateMachine();
}

```