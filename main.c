#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#define TRUE 1
#define FALSE 0

#define K1 0                // Electrode 0 is the 1 key,
#define K2 1                // Electrode 1 is the 2 key, etc...
#define K3 2
#define K4 3
#define K5 4
#define K6 5
#define K7 6
#define K8 7
#define K9 8
#define K10 9
#define K11 10
#define K12 11
#define TOUCH_THRESH 0x0F     // touch threshold
#define RELEASE_THRESH 0x09     // release threshold

#define NUM_OF_LEDS 12

#define OFF 0
#define RED 2
#define BLUE 4
#define GREEN 8

#define LED_PINS GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
// External LED pins defined in initLeds

#define SCL_PIN GPIO_PIN_2
#define SDA_PIN GPIO_PIN_3
#define IRQ_PIN GPIO_PIN_7

//TODO(Rebecca): Write Power Button Functionality
#define POWER_SW_PIN GPIO_PIN_2
#define POWER_LED_PIN GPIO_PIN_3
#define POWER_LED 8

#define DEBOUNCE_DELAY 10
#define POWER_BUTTON_NOT_PRESSED 4

struct LED {
    unsigned long aLedState;
    unsigned GPIOPin;
    unsigned GPIO_Port;
};

// Global Variables!
//TODO(Rebecca): Once functionality is working, remove pressedKey references
char lastPressedKey;            // Keypad: store which key was last pressed
_Bool allLedsOn = FALSE;   // For checking LEDs finish flooding
_Bool powerButtonOn = TRUE; // Power Button Flag
uint16_t touchedMap;        // Map of key status
struct LED leds[NUM_OF_LEDS]; // Statically stores the mapping of each port to each external LED
uint8_t ledsOn[NUM_OF_LEDS]; // Stores which LEDs are on (1 for on, 0 for off)

void setup(void);
void I2C_Init(void);
void MPR121_Init(void);
void Timer_Init(void);
void IRQ_Init(void);
void toggleKeylock(void);
void Power_Handler(void);
void PowerPress_Handler(void);
void MPR121_Handler(void);
void KeyPress_Handler(void);
void KeyPressFlood_Handler(void);

struct LED initializeLED(int ledState, int GPIOPin, int GPIOPort) {
    struct LED led;
     led.aLedState = ledState;
     led.GPIOPin = GPIOPin;
     led.GPIO_Port = GPIOPort;
     return led;
}

void initLeds(void) {
     leds[0] = initializeLED(1, GPIO_PIN_0, GPIO_PORTB_BASE);
     leds[1] = initializeLED(2, GPIO_PIN_1, GPIO_PORTB_BASE);
     leds[2] = initializeLED(16, GPIO_PIN_4, GPIO_PORTB_BASE);
     leds[3] = initializeLED(32, GPIO_PIN_5, GPIO_PORTB_BASE);
     leds[4] = initializeLED(64, GPIO_PIN_6, GPIO_PORTB_BASE);
     leds[5] = initializeLED(128, GPIO_PIN_7, GPIO_PORTB_BASE);
     leds[6] = initializeLED(1, GPIO_PIN_0, GPIO_PORTE_BASE);
     leds[7] = initializeLED(2, GPIO_PIN_1, GPIO_PORTE_BASE);
     leds[8] = initializeLED(4, GPIO_PIN_2, GPIO_PORTE_BASE);
     leds[9] = initializeLED(8, GPIO_PIN_3, GPIO_PORTE_BASE);
     leds[10] = initializeLED(16, GPIO_PIN_4, GPIO_PORTE_BASE);
     leds[11] = initializeLED(32, GPIO_PIN_5, GPIO_PORTE_BASE);

     // Setup of which LEDs are 'on' (so don't need to call GPIOPinRead)
     uint8_t i;
     for(i = 0; i < NUM_OF_LEDS; i++) {
         ledsOn[i] = FALSE;
     }
}

void setup(void) {
    // Initial settings
    IntMasterEnable();                                      // Enable processor interrupts

    // Enable device clocking
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable system peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    // Power Button
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    // Pins: I2C0SCL, I2C0SDA, Personal LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);    // Pins: Keypad interrupt (INT2)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);    // Personal LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // LEDs on Microcontroller

    // Setting LED pins to Output
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, POWER_LED_PIN); // Power Button LED
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_PINS); // Microcontroller LEDs
    uint8_t i;
    for(i = 0; i < NUM_OF_LEDS; i++) {
        struct LED led = leds[i];
        GPIOPinTypeGPIOOutput(led.GPIO_Port, led.GPIOPin); // External LEDs
    }

    // Setup Power Button Switch
    GPIODirModeSet(GPIO_PORTA_BASE, POWER_SW_PIN , GPIO_DIR_MODE_IN);

    // Register, configure and enable the Button Interrupt handler
    GPIOIntRegister(GPIO_PORTA_BASE, Power_Handler);
    GPIOIntTypeSet(GPIO_PORTA_BASE, POWER_SW_PIN, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTA_BASE,  POWER_SW_PIN);

    // Enable internal pull up resistors
    GPIOPadConfigSet(GPIO_PORTA_BASE, POWER_SW_PIN , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void I2C_Init(void) {
    // Enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // Reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    // Enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, SCL_PIN);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, SDA_PIN);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If FALSE the data rate is set to 100kbps and if TRUE the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), FALSE);

    // Clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

// Sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, FALSE);

    // Stores list of variable number of arguments
    va_list vargs;

    // Specifies the va_list to "open" and the last fixed argument
    // so vargs knows where to start looking
    va_start(vargs, num_of_args);

    // Put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));

    // If there is only one argument, we only need to use the
    // Single send I2C function
    if(num_of_args == 1)
    {
        // Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        // "close" variable argument list
        va_end(vargs);
    }

    // Otherwise, we start transmission of multiple bytes on the
    // I2C bus
    else
    {
        // Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        // Send num_of_args-2 pieces of data, using the
        // BURST_SEND_CONT command of the I2C module
        uint8_t i;
        for(i = 1; i < (num_of_args - 1); i++)
        {
            // Put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
            // Send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }

        // Put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));
        // Send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        // "close" variable args list
        va_end(vargs);
    }
}

// Read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
    // Specify that we are writing (a register address) to the
    // slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, FALSE);

    // Specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);

    // Send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    // Specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, TRUE);

    // Send control byte and read from the register we
    // specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    // Return data pulled from the specified register
    return I2CMasterDataGet(I2C0_BASE);
}

void MPR121_Init(void) {
        I2CSend(0x5A,2,0x80, 0x63);         // Soft reset

        // Set thresholds and other control values
        I2CSend(0x5A,2,0x2B, 0x01);         // MHD_R
        I2CSend(0x5A,2,0x2C, 0x01);         // NHD_R
        I2CSend(0x5A,2,0x2D, 0x00);         // NCL_R
        I2CSend(0x5A,2,0x2E, 0x00);         // FDL_R
        I2CSend(0x5A,2,0x2F, 0x01);         // MHD_F
        I2CSend(0x5A,2,0x30, 0x01);         // NHD_F
        I2CSend(0x5A,2,0x31, 0x7F);         // NCL_F
        I2CSend(0x5A,2,0x32, 0x09);         // FDL_F
        I2CSend(0x5A,2,0x41, TOUCH_THRESH);   // ELE0_T
        I2CSend(0x5A,2,0X42, RELEASE_THRESH);   // ELE0_R
        I2CSend(0x5A,2,0x43, TOUCH_THRESH);   // ELE1_T
        I2CSend(0x5A,2,0X44, RELEASE_THRESH);   // ELE1_R
        I2CSend(0x5A,2,0x45, TOUCH_THRESH);   // ELE2_T
        I2CSend(0x5A,2,0X46, RELEASE_THRESH);   // ELE2_R
        I2CSend(0x5A,2,0x47, TOUCH_THRESH);   // ELE3_T
        I2CSend(0x5A,2,0X48, RELEASE_THRESH);   // ELE3_R
        I2CSend(0x5A,2,0x49, TOUCH_THRESH);   // ELE4_T
        I2CSend(0x5A,2,0X4A, RELEASE_THRESH);   // ELE4_R
        I2CSend(0x5A,2,0x4B, TOUCH_THRESH);   // ELE5_T
        I2CSend(0x5A,2,0X4C, RELEASE_THRESH);   // ELE5_R
        I2CSend(0x5A,2,0x4D, TOUCH_THRESH);   // ELE6_T
        I2CSend(0x5A,2,0X4E, RELEASE_THRESH);   // ELE6_R
        I2CSend(0x5A,2,0x4F, TOUCH_THRESH);   // ELE7_T
        I2CSend(0x5A,2,0X50, RELEASE_THRESH);   // ELE7_R
        I2CSend(0x5A,2,0x51, TOUCH_THRESH);   // ELE8_T
        I2CSend(0x5A,2,0X52, RELEASE_THRESH);   // ELE8_R
        I2CSend(0x5A,2,0x53, TOUCH_THRESH);   // ELE9_T
        I2CSend(0x5A,2,0X54, RELEASE_THRESH);   // ELE9_R
        I2CSend(0x5A,2,0x55, TOUCH_THRESH);   // ELE10_T
        I2CSend(0x5A,2,0X56, RELEASE_THRESH);   // ELE10_R
        I2CSend(0x5A,2,0x57, TOUCH_THRESH);   // ELE11_T
        I2CSend(0x5A,2,0X58, RELEASE_THRESH);   // ELE11_R
        I2CSend(0x5A,2,0x5D, 0x04);         // FIL_CFG

        // Turn on all 12 electrodes and enter run mode
        I2CSend(0x5A,2,0x5E, 0x0C);         // ELE_CFG
}

void Timer_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);           // Timer for key debouncing
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    // Set up the timers used to lock/unlock the keypad
    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);

    // Setup the interrupts for the timer timeouts
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    IntEnable(INT_TIMER2A);

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

void IRQ_Init(void) {
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, IRQ_PIN);          // Set pin C7 as input
    GPIOPadConfigSet(GPIO_PORTC_BASE, IRQ_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);    // Pullup
    GPIOIntDisable(GPIO_PORTC_BASE, IRQ_PIN);                    // Disable interrupt for PC7
    GPIOIntClear(GPIO_PORTC_BASE, IRQ_PIN);                      // Clear pending interrupts for PC7
    GPIOIntRegister(GPIO_PORTC_BASE, MPR121_Handler);             // Register port C interrupt handler
    GPIOIntTypeSet(GPIO_PORTC_BASE, IRQ_PIN, GPIO_LOW_LEVEL);    // Configure PC for falling edge
    GPIOIntEnable(GPIO_PORTC_BASE, IRQ_PIN);                     // Enable interrupt
}

/* Interrupt GPIO_PORTA when power button pressed */
void Power_Handler(void) {
    // Clear interrupt flag
    GPIOIntClear(GPIO_PORTA_BASE, POWER_SW_PIN);

    // Check that portA isn't in default state ie. button is pushed.
    int buttonValue = GPIOPinRead(GPIO_PORTA_BASE, POWER_SW_PIN);
   _Bool wasAButtonPressed = (buttonValue != POWER_BUTTON_NOT_PRESSED);
   if(wasAButtonPressed) {
       // Reload and enable the debounce timer
       TimerLoadSet(TIMER2_BASE, TIMER_A, 10);
       TimerEnable(TIMER2_BASE, TIMER_A);
   }
}

void setPowerButton() {
    uint8_t ledState = (powerButtonOn) ? OFF: POWER_LED;
    GPIOPinWrite(GPIO_PORTA_BASE, POWER_LED_PIN, ledState);
    powerButtonOn = (powerButtonOn) ? FALSE: TRUE;
}

/* Interrupt for handling power button debounce */
void PowerPress_Handler(void) {
    // Clear Timer Interrupt Flag
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerDisable(TIMER2_BASE, TIMER_A);

    // Check if the button is still pressed
    _Bool wasAButtonPressed = (GPIOPinRead(GPIO_PORTA_BASE, POWER_SW_PIN) != POWER_BUTTON_NOT_PRESSED);
    if(wasAButtonPressed) {
        setPowerButton();
    }
}

// Turn on/off LED
void setLed(uint8_t index, _Bool setLedOn) {
    if(powerButtonOn) {
        struct LED aLed = leds[index];
        uint8_t ledState = setLedOn ? aLed.aLedState: 0;
        GPIOPinWrite(aLed.GPIO_Port, aLed.GPIOPin, ledState);
    }
}

void MPR121_Handler(void){
    uint32_t touchedLSB, touchedMSB;
    uint16_t lastTouchedMap = touchedMap;

    // Get the status of the electrodes
    touchedLSB = I2CReceive(0x5A,0x00);
    touchedMSB = I2CReceive(0x5A,0x01);
    touchedMap = ((touchedMSB << 8) | touchedLSB);

    // If multiple button presses increases, turn on debouncing timer
    if (touchedMap > lastTouchedMap) {
        if (touchedMap & (1<<K1)){ lastPressedKey = '1'; }
        else if (touchedMap & (1<<K2)){ lastPressedKey = '2'; }
        else if (touchedMap & (1<<K3)){ lastPressedKey = '3'; }
        else if (touchedMap & (1<<K4)){ lastPressedKey = '4'; }
        else if (touchedMap & (1<<K5)) { lastPressedKey = '5'; }
        else if (touchedMap & (1<<K6)) { lastPressedKey = '6'; }
        else if (touchedMap & (1<<K7)) { lastPressedKey = '7'; }
        else if (touchedMap & (1<<K8)) { lastPressedKey = '8'; }
        else if (touchedMap & (1<<K9)) { lastPressedKey = '9'; }
        else if (touchedMap & (1<<K10)) { lastPressedKey = 'A'; }
        else if (touchedMap & (1<<K11)) { lastPressedKey = 'B'; }
        else if (touchedMap & (1<<K12)) { lastPressedKey = 'C'; }
        TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 0.5);
        TimerEnable(TIMER0_BASE, TIMER_A);
    }
    // If less buttons are pressed than before, turn off all LEDs and clear timers
    else if(touchedMap < lastTouchedMap) {
        // Turn off all LEDs
        uint8_t i = 0;
        for(i = 0; i < NUM_OF_LEDS; i++) {
            setLed(i, FALSE);
            ledsOn[i] = FALSE;
        }
        // Clear and disable any timers
        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        TimerDisable(TIMER0_BASE, TIMER_A);

        TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
        TimerDisable(TIMER1_BASE, TIMER_A);

        allLedsOn = FALSE;
    }
    // If touchedMap == lastTouchedMap, do nothing
    else {}

    // Clear the asserted interrupts.
    GPIOIntClear(GPIO_PORTB_BASE, IRQ_PIN);
}

// Toggle Key Presses
void toggleKeylock(void)
{
    // For debugging purposes
    uint8_t ledState = GPIOPinRead(GPIO_PORTF_BASE, LED_PINS);
    _Bool isLedNotRed = (ledState != RED);
    if (isLedNotRed) {
        ledState = RED;
    } else {
        ledState = OFF;
    }
    GPIOPinWrite(GPIO_PORTF_BASE, LED_PINS, ledState);

    // Turn on all LEDs pressed on touchMap
    uint8_t i;
    for(i = 0; i < NUM_OF_LEDS; i++) {
        _Bool isButtonPressed = (touchedMap & (1<<i));
        if(isButtonPressed) {
            setLed(i, TRUE);
            ledsOn[i] = TRUE;
        }
    }

    // Turn on LED flooding timer
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 0.5);
    TimerEnable(TIMER1_BASE, TIMER_A);
}

void KeyPress_Handler(void) {
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Disable the timer
    TimerDisable(TIMER0_BASE, TIMER_A);

   // Get the status of the electrodes
   uint32_t touchedLSB = I2CReceive(0x5A,0x00);
   uint32_t touchedMSB = I2CReceive(0x5A,0x01);
   uint16_t currentTouchedMap = ((touchedMSB << 8) | touchedLSB);

   // Check for debouncing
   _Bool isStillPressed = touchedMap == currentTouchedMap;
   if (isStillPressed) {
       toggleKeylock();
   }
}

// Check which LEDs are currently on, and flood the light to adjacent LEDs
void flood(void) {
     // Only flood when there are still LEDs that need to turn on
     if(!allLedsOn) {
         _Bool areAllLedsOn = TRUE;

         // Make a copy of the ledsOn array
         uint8_t presentLedsOn [NUM_OF_LEDS];
         memcpy(&presentLedsOn, &ledsOn, sizeof(presentLedsOn));

         uint8_t i;
         for(i = 0; i < NUM_OF_LEDS; i++) {
             if(ledsOn[i] == 0) { // If at least 1 LED was off before looping
                 areAllLedsOn = FALSE;
             } else { // If an LED is on, turn on adjacent LEDs if not already on
                 // Turn on left LED when applicable
                 _Bool notLeftMostLed = i > 0;
                 _Bool isLeftLedOff = presentLedsOn[i-1] != TRUE;
                 _Bool canTurnLeftLedOn = notLeftMostLed && isLeftLedOff;

                 if(canTurnLeftLedOn) {
                     setLed(i-1, TRUE);
                     presentLedsOn[i-1] = TRUE;
                 }

                 // Turn on right LED when applicable
                 _Bool notRightMostLed = i < (NUM_OF_LEDS - 1);
                 _Bool isRightLedOff = presentLedsOn[i+1] != TRUE;
                 _Bool canTurnRightLedOn = notRightMostLed && isRightLedOff;

                 if(canTurnRightLedOn) {
                     setLed(i+1, TRUE);
                     presentLedsOn[i+1] = TRUE;
                 }
             }
         }
         if (areAllLedsOn) {
             allLedsOn = TRUE;
         }
         // Set ledsOn to presentLedsOn
         memcpy(ledsOn, presentLedsOn, sizeof(ledsOn));
     } else { //If all LEDs are turned on, pulse the lights
         _Bool state = TRUE;
         if(ledsOn[0] == TRUE) {
             state = FALSE;
         }
         uint8_t i;
         for(i = 0; i < NUM_OF_LEDS; i++) {
             setLed(i, state);
             ledsOn[i] = state;
         }
     }
}

// If the same capacitor is being pressed, turn on joint LEDs
void KeyPressFlood_Handler(void) {
    // Clear the timer interrupt.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Disable the timer
    TimerDisable(TIMER1_BASE, TIMER_A);

    // Flood LED light to adjacent LEDs
    flood();

   TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 0.5);
   TimerEnable(TIMER1_BASE, TIMER_A);
}

int main(void) {
    // Setup of the LED struct array
    // Note: This needs to be run first BEFORE setup()
    initLeds();

    // Setup peripherals
    setup();

    // Timer Init
    Timer_Init();

    // Start I2C module
    I2C_Init();

   // Enable the I2C interrupt
    IRQ_Init();

   // Start the MPR121 and set thresholds
   MPR121_Init();

   // Turn on the Micro Controller Blue LED & Power Button LED when finish initializing
   GPIOPinWrite(GPIO_PORTF_BASE, LED_PINS, BLUE);
   GPIOPinWrite(GPIO_PORTA_BASE, POWER_LED_PIN, POWER_LED);

   while(1){}
}
