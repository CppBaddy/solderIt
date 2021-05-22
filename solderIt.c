#include <stdint.h>
#include <stdbool.h>

#ifndef F_CPU
    #define F_CPU   8000000
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <avr/eeprom.h>

#include "pid.h"

// ---------------------    // Vcc, Pin 1 on SSD1306 Board
// ---------------------    // GND, Pin 2 on SSD1306 Board
#define SSD1306_SCL     PB2 // SCL, Pin 3 on SSD1306 Board
#define SSD1306_SDA     PB0 // SDA, Pin 4 on SSD1306 Board

#define SSD1306_SA      0x78    // Slave address

#include "ssd1306xled.h"


#define HEATER_PORT  PB1  // heater control port
#define BEEPER_PORT  PB4  // beep output port


#define ADC_VREF_256V       (_BV(REFS2) | _BV(REFS1))       // 2.56V internal VREF without external capacitor

#define TEMPERATURE_IN      (_BV(MUX1) | _BV(MUX0)) //0x03 input PB3
#define ENCODER_IN          _BV(MUX1)   //0x02 input PB4

#define ADC_LEFT_JUSTIFIED  _BV(ADLAR)  //0x20 - left justified, so we can use ADCH as a 8 bit result


#define ADC_SCAN_FREQ       2000 // 2KHz or each 500 uS

#define TIMER1_PRESCALER    32

#define TIMER1_RELOAD         (F_CPU/TIMER1_PRESCALER/ADC_SCAN_FREQ - 1)

#if (TIMER1_RELOAD > 255)
#error Timer1 reload value is greater then 8 bits. Consider increasing prescaler value.
#elif (TIMER1_RELOAD == 0)
#error Timer1 reload value is zero. Consider decreasing prescaler value.
#endif

//#define USE_QUADPULSE           1

#ifdef USE_QUADPULSE
    #define TEMPERATURE_SET_STEP    2
#else
    #define TEMPERATURE_SET_STEP    5
#endif

#define TEMPERATURE_CELSIUS

#define TEMPERATURE_SCALING     2
#define MIN_TARGET_TEMPERATURE  (200*TEMPERATURE_SCALING)
#define MAX_TARGET_TEMPERATURE  (400*TEMPERATURE_SCALING)
#define INITIAL_TEMPERATURE     (270*TEMPERATURE_SCALING)
#define TEMPERATURE_OFFSET      (80*TEMPERATURE_SCALING)

/* P, I and D parameter values
 *
 * The K_P, K_I and K_D values (P, I and D gains)
 * need to be modified to adapt to the application at hand
 */
#define K_P     10.00
#define K_I     0.10
#define K_D     0.01

enum ePortMode
{
    eAdcInput,
    eDigitalOutput,
    eDigitalInput
};


enum eAdcInput
{
    eAdcEnter   = 0,
    eAdcBit00   = 100,
    eAdcBit01   = 125,
    eAdcBit10   = 160,
    eAdcBit11   = 250
};


enum eBitMask
{
    eBit00   = 0x00,
    eBit01   = 0x01,
    eBit10   = 0x02,
    eBit11   = 0x03,
    eEnter   = 0x04,
    eCodeInvalid = 0xf0
};


enum eUserInput
{
    eIncrement = 1,
    eDecrement,
    ePush,
    eRelease,
    eLongPush
};

enum eAppState
{
    eNormal,
    ePreset,
    eMenu
};

//EEPROM configuration
#define MAX_PRESET_TEMPERATURES     10

uint8_t  EEMEM nvState = ePreset;
uint8_t  EEMEM nvActivePreset = 3;
uint16_t EEMEM nvTemperature = 270*TEMPERATURE_SCALING;
uint16_t EEMEM nvPresets[MAX_PRESET_TEMPERATURES] =
{
    175*TEMPERATURE_SCALING,
    200*TEMPERATURE_SCALING,
    225*TEMPERATURE_SCALING,
    250*TEMPERATURE_SCALING,
    275*TEMPERATURE_SCALING,
    300*TEMPERATURE_SCALING,
    325*TEMPERATURE_SCALING,
    350*TEMPERATURE_SCALING,
    375*TEMPERATURE_SCALING,
    400*TEMPERATURE_SCALING
};

uint16_t EEMEM nvOffset = 10;
uint8_t  EEMEM nvBeepEnable = 1;
uint8_t  EEMEM nvLedContrast = 32;



volatile uint16_t gAdcTemperature = 0;
volatile uint8_t  gAdcEncoder = 0;
volatile uint8_t  gEncoderCode = 0;
volatile uint8_t  gInputReady = 0;

volatile uint16_t gTargetTemperature = 0;

volatile uint8_t gAdcTick = 0;
volatile uint8_t gPidTick = 0;

volatile uint8_t gBeepOn = 0;

// Parameters for regulator
struct PID_DATA pidData;

bool onTarget = false;

enum eAppState gAppState = eNormal;

uint8_t gPresetIdx = 0;


void portMode( uint8_t portId, uint8_t mode );

void beep();
void beepShort();

void    I2C_init();
void    I2C_sendByte(uint8_t b);
uint8_t I2C_getByte();

void setup();
void setupDisplay();

void ADC_start();
void ADC_stop();

void HeaterOn();
void HeaterOff();


void onButtonPress();
void onButtonRelease();

void Temperature_Up();
void Temperature_Down();
void Temperature_Next();
void Temperature_Prev();

void showInput();
void showTemperature();
void showPowerLevel();

void setContrast(uint8_t v); //set OLED display contrast

void setPowerLevel(uint8_t v);

void InitPID();
int16_t getReference();
int16_t getMeasurement();
void    setInput(int16_t inputValue);

void Config_setUint8(uint8_t* nvm, uint8_t v);
uint8_t Config_getUint8(uint8_t* nvm);

void Config_setUint16(uint16_t* nvm, uint16_t v);
uint16_t Config_getUint16(uint16_t* nvm);

void HandleUserInput(enum eUserInput i);


/* Timer0 interrupt handler */
//ISR( TIMER0_OVF_vect )
//{
//}

/* Timer0 interrupt handler */
//ISR( TIMER0_COMPA_vect )
//{
//}

//ISR( TIMER0_COMPB_vect )
//{
//}

/* Timer1 interrupt handler */
//ISR( TIMER1_OVF_vect )
//{
//}

/* Timer1 interrupt handler */
ISR( TIMER1_COMPA_vect ) //happens each 500uS or at 2000Hz
{
    if(gBeepOn)
    {
        if(++gBeepOn) //~200ms
        {
            PINB |= _BV(BEEPER_PORT); //toggle beeper output pin

 //           if(gBeepOn == 196) //two tone buzzer
 //           {
 //               TIMSK &= ~_BV(OCIE1B); //disable interrupt for CompareMatch1B
 //           }
        }
        else
        {
            TIMSK &= ~_BV(OCIE1B); //disable interrupt for CompareMatch1B
            portMode(BEEPER_PORT, eAdcInput); //disable beeper and return to encoder monitoring
        }
    }

    ADC_start();

    if((++gAdcTick % 64) == 0) // at 25Hz
    {
        gAdcTick = 0;
        gPidTick = true;
    }
}

/* Timer1 interrupt handler */
ISR( TIMER1_COMPB_vect )
{
    PINB |= _BV(BEEPER_PORT); //toggle beeper output pin
}

// ADC interrupt service routine
ISR( ADC_vect ) // this gets executed at about 2000 Hz
{
    static uint16_t temperatureSum = 0;

    // Save the AD conversion result
    uint8_t adcl = ADCL;
    uint8_t adch = ADCH;

    switch( ADMUX & TEMPERATURE_IN )
    {
    case TEMPERATURE_IN:
    {
        //filtering by averaging over 32 values
        temperatureSum += (adch << 8) | adcl; //collect sum of 32 values

        if(gAdcTick % 63 == 0)
        {
            gAdcTemperature = (temperatureSum >> 5); //divide by 32 to get average value
            temperatureSum = 0;
        }

        ADMUX = ( ADC_LEFT_JUSTIFIED | ENCODER_IN | ADC_VREF_256V );
    }
        break;

    case ENCODER_IN:
    {
        ADMUX = ( TEMPERATURE_IN | ADC_VREF_256V );

        if(gBeepOn == 0) //we are not beeping
        {
            if( (gAdcEncoder >> 2) == (adch >> 2) ) //input is stable at +- 2 LSB
            {
                uint8_t code = eCodeInvalid;

                if( (eAdcBit11 + eAdcBit10)/2 < adch )
                {
                    code = eBit11;
                }
                else if( (eAdcBit10 + eAdcBit01)/2 < adch )
                {
                    code = eBit10;
                }
                else if( (eAdcBit01 + eAdcBit00)/2 < adch )
                {
                    code = eBit01;
                }
                else if( (eAdcBit00 + eEnter)/2 < adch )
                {
                    code = eBit00;
                }
                else
                {
                    code = eEnter;
                }

                if(code != gEncoderCode) //notify if code changed
                {
                    gEncoderCode = code;
                    gInputReady = true;
                }
            }

            gAdcEncoder = adch; //save last measured value
        }

    }
        break;
    default:
        break;
    }
}

//TODO lookup table for non-linear thermistor reading to temperature
//TODO Configuration menu. Enter during boot time if button held pressed.
//TODO Preset temperatures mode

int main( void )
{
    setup(); //setup ports

    gAppState = Config_getUint8(&nvState);

    gPresetIdx = Config_getUint8(&nvActivePreset);

    if(gAppState == eNormal)
    {
        gTargetTemperature = Config_getUint16(&nvTemperature);
    }
    else
    {
        gTargetTemperature = Config_getUint16(&nvPresets[gPresetIdx]);
    }

    ssd1306_init();

    ssd1306_fillscreen(0x00);

    showInput();
    //ssd1306_char_f8x16(2, 3, "000");
/*  ssd1306_char_f8x16(1, 0, "MZR");
    ssd1306_char_f8x16(30, 2, "SOLDERING");
    ssd1306_char_f8x16(26, 4, "CONTROLLER");
    ssd1306_char_f8x16(95, 6, "2017");
*/
    InitPID();

    uint8_t prevCode = eCodeInvalid;

    uint8_t buttonPressed = false;

    uint8_t displayTimer = 0;

    for(;;)
    {
        // Run PID calculations on PID timer tick
        if(gPidTick)
        {
            gPidTick = false;

            ++displayTimer;

            int16_t targetTemperature = getReference();
            int16_t temperature = getMeasurement();

            if(onTarget == false)
            {
                if(targetTemperature == temperature)
                {
                    onTarget = true;
                    beepShort();
                }
            }

            int16_t powerLevel = pid_Controller(targetTemperature, temperature, &pidData);

            setInput(powerLevel);
        }

        if(displayTimer % 12 == 0) // at 2Hz
        {
            showTemperature();
        }

        if(displayTimer % 4 == 0) // at 6Hz
        {
            showPowerLevel();
        }

        //process command decoder
        if(gInputReady == true)
        {
            gInputReady = false;
            uint8_t code = gEncoderCode; //make local copy

            if(buttonPressed == true)
            {
                buttonPressed = false;
                //action onButtonRelease
                HandleUserInput(eRelease);
            }

            if(code == eEnter)
            {
                buttonPressed = true;
                //action onButtonPress
                HandleUserInput(ePush);
            }
            else
            {
                if((prevCode ^ code) & eBit10) // bit 10 changed
                {
                    if((code & eBit11) == eBit11 /*|| (code & eBit11) == eBit00*/)  //detect CW
                    {
                        //action onDecrement
                        HandleUserInput(eDecrement);
                    }
                    else if((code & eBit11) == eBit10) //detect CCW
                    {
                        //action onIncrement
                        HandleUserInput(eIncrement);
                    }
                }
#ifdef USE_QUADPULSE
                else if((prevCode ^ code) & eBit01) // bit 01 changed
                {
                    if((code & eBit11) == eBit01 || (code & eBit11) == eBit10)  //detect CW
                    {
                        //action onDecrement;
                        HandleUserInput(eDecrement);
                    }
                    else  //detect CCW
                    {
                        //action onIncrement;
                        HandleUserInput(eIncrement);
                    }
                }
#endif

                prevCode = code;
            }

            showInput();
        }

    }

    return 0;
}

void portMode(uint8_t port, uint8_t mode)
{
    switch(mode)
    {
    case eAdcInput:
        DDRB  &= ~_BV(port); //set direction to input
        PORTB &= ~_BV(port); //disable pullup resistor - tristate
        DIDR0 |= _BV(port);  //disable digital input on ADC2 PB4
        break;
    case eDigitalOutput:
        //disable ADC in and enable GPIO
        DDRB  |= _BV(port);  //set direction to output
        PORTB &= ~_BV(port); //has to go through this before setting to 1
        DIDR0 &= ~_BV(port); //enable digital input
        break;
    case eDigitalInput:
        DDRB  &= ~_BV(port); //set direction to input
        PORTB &= ~_BV(port); //disable pullup resistor
        DIDR0 &= ~_BV(port); //enable digital input
        break;
    default:
        break;
    }
}

inline void beepShort()
{
    portMode(BEEPER_PORT, eDigitalOutput);
    TIMSK |= _BV(OCIE1B); //enable interrupt for CompareMatch1B
    gBeepOn = 128;
}

inline void beep()
{
    portMode(BEEPER_PORT, eDigitalOutput);
    TIMSK |= _BV(OCIE1B); //enable interrupt for CompareMatch1B
    gBeepOn = 1;
}

/*
void I2C_init()
{
    PORTB |= _BV(PB0) | _BV(PB2); //set to high state
    DDRB |= _BV(DDB0) | _BV(DDB2); //enable SDA and SCL as outputs

    USIDR = 0xff; //bus in released state

    //two wire mode
    //counter overflow enable
    //timer/counter0 compare match as clock source
    USICR = _BV(USIOIE) | _BV(USIWM1) | _BV(USICS0);

    //clear flags and reset counter
    USISR = _BV(USISIF) | _BV(USIOIF) | _BV(USIPF) | _BV(USIDC);
}

inline void I2C_sendByte(uint8_t b)
{
    //TODO wait for ready state (not busy)
    USIDR = b;
}

uint8_t I2C_getByte()
{
    return USIBR;
}
*/

inline void setup()
{
    CLKPR = _BV(CLKPCE); //enable Clock Prescale Register write
    CLKPR = 0;           // change prescaler to 1, effectively set 8MHz system clock

    //PB0 - SDA
    //PB1 - PWM output
    //PB2 - SCL
    DDRB |= _BV( DDB1 ); /* set PBx to output */

    //PB3 - ADC3 temperature in
    DIDR0 |= _BV( ADC3D ); //disable digital input on ADC3 PB3
    //PB4 - ADC2 encoder input and beeper output
    DIDR0 |= _BV( ADC2D ); //disable digital input on ADC2 PB4

    //PB5 - Reset (can be reused as separate pin later)

    // ADC Voltage Reference: internal 2.56V
    // ADC High Speed Mode: Off
    //ADMUX = ( TEMPERATURE_IN | ADC_VREF_256V );
    ADMUX = ( ADC_LEFT_JUSTIFIED | ENCODER_IN | ADC_VREF_256V );

    // ADC Enabled
    // ADC Clock frequency: prescaler 64, F_OSC/64 = 125KHz, conversion time 104us
    ADCSRA |= ( _BV( ADEN ) | _BV( ADIE ) | _BV( ADPS2 ) | _BV( ADPS1 ) );

    //Timer0 PWM for heater
    TCCR0A = _BV(COM0B1) | _BV(WGM00); //PB1 output, Mode 1: PWM phase correct, counting to 0xff
    TCCR0B = _BV(CS01); //prescaler 8
    OCR0B  = 0; //compare to TCNT0 to set output on PB1

    //Timer1 overflow each 500us
    TCCR1 = _BV(CTC1) | _BV(CS11) | _BV(CS12); //clear Timer1 on CompareMatch1C; prescaler 32
    OCR1A = TIMER1_RELOAD/2;
    OCR1B = 1; //for buzzer output
    OCR1C = TIMER1_RELOAD;

    TIMSK |= _BV(OCIE1A); //enable interrupt for CompareMatch1A

    sei();
}

inline void ADC_start()
{
    // ADC Start conversion
    ADCSRA |= _BV( ADSC );
}

inline void ADC_stop()
{
    // ADC Stop conversion
    ADCSRA &= ~_BV( ADSC );
}

inline void onButtonPress()
{
}

inline void onButtonRelease()
{
    beepShort();
}

void Temperature_Down()
{
    gTargetTemperature -= TEMPERATURE_SET_STEP;

#ifdef TEMPERATURE_CELSIUS
    if(gTargetTemperature < MIN_TARGET_TEMPERATURE)
    {
        gTargetTemperature = MIN_TARGET_TEMPERATURE;
    }
#endif
}

void Temperature_Up()
{
    gTargetTemperature += TEMPERATURE_SET_STEP;

#ifdef TEMPERATURE_CELSIUS
    if(gTargetTemperature > MAX_TARGET_TEMPERATURE)
    {
        gTargetTemperature = MAX_TARGET_TEMPERATURE;
    }
#endif
}

void Temperature_Next()
{
    if(gPresetIdx < MAX_PRESET_TEMPERATURES - 1)
    {
        ++gPresetIdx;

        gTargetTemperature = Config_getUint16(&nvPresets[gPresetIdx]);

        onTarget = false;
    }
}

void Temperature_Prev()
{
    if(gPresetIdx > 0)
    {
        --gPresetIdx;

        gTargetTemperature = Config_getUint16(&nvPresets[gPresetIdx]);

        onTarget = false;
    }
}

#define ASCII_DIGIT_OFFSET 48

const char* uint8toStr(uint8_t v)
{
    static char s[4];

    s[0] = v / 100 + ASCII_DIGIT_OFFSET;
    s[1] = (v % 100) / 10 + ASCII_DIGIT_OFFSET;
    s[2] = v % 10 + ASCII_DIGIT_OFFSET;
    s[3] = 0;

    for(uint8_t i=0; i < 2; ++i)
    {
        if(s[i] == ASCII_DIGIT_OFFSET)
            s[i] = ' ';
        else
            break;
    }

    return s;
}

const char* uint16toStr(uint16_t v)
{
    static char s[6];

    s[0] = v / 10000 + ASCII_DIGIT_OFFSET;
    s[1] = (v % 10000) / 1000 + ASCII_DIGIT_OFFSET;
    s[2] = (v % 1000) / 100 + ASCII_DIGIT_OFFSET;
    s[3] = (v % 100) / 10 + ASCII_DIGIT_OFFSET;
    s[4] = v % 10 + ASCII_DIGIT_OFFSET;
    s[5] = 0;

    for(uint8_t i=0; i < 4; ++i)
    {
        if(s[i] == ASCII_DIGIT_OFFSET)
            s[i] = ' ';
        else
            break;
    }

    return s;
}

void setContrast(uint8_t v)
{
    ssd1306_send_command(0x81);
    ssd1306_send_command(v);
}

inline void setPowerLevel(uint8_t v)
{
    OCR0B = v;
}

inline void showInput()
{
    if(gAppState == eNormal)
    {
        ssd1306_char_f8x16(0, 2, "    ");
    }
    else
    {
        ssd1306_char_f8x16(0, 2, "Pre");

        char s[2];
        s[0] = gPresetIdx + '0';
        s[1] = 0;

        ssd1306_char_f8x16(24, 2, s);
    }

#ifdef TEMPERATURE_CELSIUS
    ssd1306_char_f8x16(32, 2, uint16toStr(gTargetTemperature >> 1));
#else
    ssd1306_char_f8x16(32, 2, uint16toStr(gTargetTemperature));
#endif
}

inline void showTemperature()
{
#ifdef TEMPERATURE_CELSIUS
    uint16_t t = (onTarget ? gTargetTemperature : getMeasurement()) >> 1;

    if(t > 420)
    {
        ssd1306_char_f8x16(23, 4, "    NO     ");
        ssd1306_char_f8x16(23, 6, "CONNECTION ");
    }
    else if(t > 90)
    {
        ssd1306_char_f16x32(12, 4, uint16toStr(t));
        ssd1306_char_f6x8 (91, 4, "o");
        ssd1306_char_f8x16(98, 4, "C");
        ssd1306_char_f8x16(91, 6, "  ");
    }
    else
    {
        ssd1306_char_f8x16(23, 5, "HEATING...");
    }
#else
    uint16_t t = getMeasurement();

    ssd1306_char_f16x32(12, 4, uint16toStr(t));
#endif
}

inline void showPowerLevel()
{
    ssd1306_dbar(0, 1, OCR0B);
}

/* Init of PID controller */
inline void InitPID(void)
{
  pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);
}

/* Read reference value.
 *
 * This function must return the reference value.
 * May be constant or varying
 */
inline int16_t getReference(void)
{
  return gTargetTemperature;
}

/* Read system process value
 *
 * This function must return the measured data
 */
inline int16_t getMeasurement(void)
{
    cli();
    uint16_t t = gAdcTemperature;
    sei();

#ifdef TEMPERATURE_CELSIUS
    //t = (t*10)/12 + TEMPERATURE_OFFSET; // 0.83
    t = (t*10)/13 + TEMPERATURE_OFFSET; // 0.75
#endif

    return t;
}

/* Set control input to system
 *
 * Set the output from the controller as input
 * to system.
 */
void setInput(int16_t v)
{
    if(v < 0)
    {
        v = 0;
    }
    else if(v > 255)
    {
        v = 254;
    }

    setPowerLevel(v);
}

inline void Config_updateUint8(uint8_t* nvm, uint8_t v)
{
    eeprom_busy_wait();
    eeprom_update_byte(nvm, v);
}

inline void Config_setUint8(uint8_t* nvm, uint8_t v)
{
    eeprom_busy_wait();
    eeprom_write_byte(nvm, v);
}

inline uint8_t Config_getUint8(uint8_t* nvm)
{
    eeprom_busy_wait();
    return eeprom_read_byte(nvm);
}

inline void Config_updateUint16(uint16_t* nvm, uint16_t v)
{
    eeprom_busy_wait();
    eeprom_update_word(nvm, v);
}

inline void Config_setUint16(uint16_t* nvm, uint16_t v)
{
    eeprom_busy_wait();
    eeprom_write_word(nvm, v);
}

inline uint16_t Config_getUint16(uint16_t* nvm)
{
    eeprom_busy_wait();
    return eeprom_read_word(nvm);
}

/*void Config_saveTemperature(uint16_t v)
{
    uint8_t* p = (uint8_t*)&v;
    EEPROM_write(CFG_TEMPERATURE, *p);
    EEPROM_write(CFG_TEMPERATURE+1, *(p+1));
}

uint16_t Config_getTemperature()
{
    uint16_t t = EEPROM_read(CFG_TEMPERATURE) + (EEPROM_read(CFG_TEMPERATURE+1) << 8);

    if(t > MAX_TARGET_TEMPERATURE || t < MIN_TARGET_TEMPERATURE)
    {
        t = INITIAL_TEMPERATURE;
    }

    return t;
}
*/

void HandleUserInput(enum eUserInput v)
{
    switch(gAppState)
    {
    case eNormal:
        switch(v)
        {
        case eIncrement:
            Temperature_Up();
            break;

        case eDecrement:
            Temperature_Down();
            break;

        case ePush:
            gAppState = ePreset;
            gTargetTemperature = Config_getUint16(&nvPresets[gPresetIdx]);
            break;

        case eRelease:
            onButtonRelease();
            break;

        case eLongPush:
            gAppState = eMenu;
            break;

        default:
            break;
        }
    break;

    case ePreset:
        switch(v)
        {
        case eIncrement:
            Temperature_Next();
            break;

        case eDecrement:
            Temperature_Prev();
            break;

        case ePush:
            gAppState = eNormal;
            gTargetTemperature = Config_getUint16(&nvTemperature);
            break;

        case eRelease:
            onButtonRelease();
            break;

        case eLongPush:
            gAppState = eMenu;
            break;

        default:
            break;
        }
    break;

    case eMenu:
        switch(v)
        {
        case eIncrement:
            break;

        case eDecrement:
            break;

        case ePush:
            break;

        case eRelease:
            break;

        case eLongPush:
            gAppState = Config_getUint8(&nvState);
            break;

        default:
            break;
        }
    break;

    default:
    break;
    }
}
