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

#define K1 0                // Electrode 0 is the 1 key,
/*#define K2 4                // Electrode 4 is the 2 key, etc...
#define K3 3
#define K4 9
#define K5 5
#define K6 2
#define K7 10
#define K8 6
#define K9 1
#define K0 7
#define KS 11               // * (star)
#define KP 0                // # (pound or hash)*/
#define TOUCH_THRESH 0x0F     // touch threshold
#define RELEASE_THRESH 0x09     // release threshold

#define OFF 0
#define RED 2
#define BLUE 4
#define GREEN 8

#define LED_PINS GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
#define SCL_PIN GPIO_PIN_2
#define SDA_PIN GPIO_PIN_3
#define IRQ_PIN GPIO_PIN_7

// Global Variables!
char pressedKey;            // Keypad: store which key was last pressed
_Bool keysUnlocked = true;   // For locking the keypad
uint16_t touchedMap;        // Map of key status

void setup(void);
void I2C_Init(void);
void MPR121_Init(void);
void Timer_Init(void);
void IRQ_Init(void);
void toggleKeylock(void);
void MPR121_Handler(void);
void KeyPress_Handler(void);

void setup(void) {
    // Initial settings
    IntMasterEnable();                                      // Enable processor interrupts

    // Enable device clocking
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Enable system peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);            // Pins: I2C0SCL, I2C0SDA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);            // Pins: Keypad interrupt (INT2)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Setting LED pins to Output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_PINS);
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
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    // Clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

// Sends an I2C command to the specified slave
void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...)
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

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
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    // Specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);

    // Send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    // Specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

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

    // Set up the timers used to lock/unlock the keypad
    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);

    // Setup the interrupts for the timer timeouts
    IntEnable(INT_TIMER0A);

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void IRQ_Init(void) {
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, IRQ_PIN);          // Set pin C7 as input
    GPIOPadConfigSet(GPIO_PORTC_BASE, IRQ_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);    // Pullup
    GPIOIntDisable(GPIO_PORTC_BASE, IRQ_PIN);                    // Disable interrupt for PC7
    GPIOIntClear(GPIO_PORTC_BASE, IRQ_PIN);                      // Clear pending interrupts for PC7
    GPIOIntRegister(GPIO_PORTC_BASE, MPR121_Handler);             // Register port C interrupt handler
    GPIOIntTypeSet(GPIO_PORTC_BASE, IRQ_PIN, GPIO_LOW_LEVEL);    // Configure PC7 for falling edge
    GPIOIntEnable(GPIO_PORTC_BASE, IRQ_PIN);                     // Enable interrupt
}

// Toggle Key Presses
void toggleKeylock(void)
{
    uint8_t led = OFF;
    if (keysUnlocked) {
        keysUnlocked = false;
        led = RED;
    }
    else {
        keysUnlocked = true;
        led = GREEN;
    }
    GPIOPinWrite(GPIO_PORTF_BASE, LED_PINS, led);

}

void MPR121_Handler(void){
    int touchNumber = 0;
    int j;
    uint32_t touchedLSB, touchedMSB;

    // Get the status of the electrodes
    touchedLSB = I2CReceive(0x5A,0x00);
    touchedMSB = I2CReceive(0x5A,0x01);
    touchedMap = ((touchedMSB << 8) | touchedLSB);

    // Check how many electrodes were pressed
    for (j=0; j<12; j++) { if ((touchedMap & (1<<j))) { touchNumber++; } }
    // If one electrode was pressed, register it
    if (touchNumber == 1) {
        if (touchedMap & (1<<K1)){
            pressedKey = '1'; //TODO(Rebecca): Tech never used since not using UART
            toggleKeylock();
        } /*else if (touchedMap & (1<<K2)){
            pressedKey = '2';
        } else if (touchedMap & (1<<K3)){
            pressedKey = '3';
        } else if (touchedMap & (1<<K4)){
            pressedKey = '4';
        }
        else if (touchedMap & (1<<K5)) { pressedKey = '5'; }
        else if (touchedMap & (1<<K6)) { pressedKey = '6'; }
        else if (touchedMap & (1<<K7)) { pressedKey = '7'; }
        else if (touchedMap & (1<<K8)) { pressedKey = '8'; }
        else if (touchedMap & (1<<K9)) { pressedKey = '9'; }
        else if (touchedMap & (1<<K0)) { pressedKey = '0'; }
        else if (touchedMap & (1<<KS)) { pressedKey = '*'; }
        else if (touchedMap & (1<<KP)) {
            pressedKey = '#';
            TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() * 2);
            TimerEnable(TIMER0_BASE, TIMER_A);
        }*/
    }
    // If one electrode was released
    else if (touchNumber == 0) {}
    // Do nothing if more than one button is pressed
    else {}

    // Clear the asserted interrupts.
    GPIOIntClear(GPIO_PORTC_BASE, IRQ_PIN);
}


//TODO(Rebecca): Unsure this is needed as well :?
void KeyPress_Handler(void) {
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Disable the timer
    TimerDisable(TIMER0_BASE, TIMER_A);

    // The timer is up! Toggle keysUnlocked
    if (touchedMap) {
        toggleKeylock();
    }
}

int main(void) {
    setup();

    // Timer Init
    Timer_Init();

    // Start I2C module
    I2C_Init();

   // Enable the I2C interrupt
    IRQ_Init();

   // Start the MPR121 and set thresholds
   MPR121_Init();

   // Ready
   GPIOPinWrite(GPIO_PORTF_BASE, LED_PINS, BLUE);

   while(1){}
}
