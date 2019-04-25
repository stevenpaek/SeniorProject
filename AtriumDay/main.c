/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <file.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <third_party/fatfs/ffcio.h>

#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDFatFS.h>
/****************************************************************************************************
 ************************************** LIGHTS ******************************************************
 ****************************************************************************************************/
const eUSCI_SPI_MasterConfig spiMasterConfigLED=
{
 EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
 12000000,
 5333333,
 EUSCI_B_SPI_MSB_FIRST,
 EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
 EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,
 EUSCI_B_SPI_3PIN
};
void initStrip()//SPI for DAC
{
   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
   SPI_initMaster(EUSCI_B3_BASE, &spiMasterConfigLED);
   SPI_enableModule(EUSCI_B3_BASE);
}
void goBlue(){
    SPI_disableInterrupt(EUSCI_B3_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);  // disable interrupts
    // Send 0x00 for G
    uint8_t lights;
    for(lights = 0; lights<4;lights++)
    {
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0x00;
    }
    for(lights=0;lights<11;lights++)
    {
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0xEF;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0xFF;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0x00;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0x00;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
    }
    for(lights = 0; lights<4;lights++)
    {
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0xFF;
    }
    SPI_enableInterrupt(EUSCI_B3_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);   // enable interrupts
}
void goRed(){
    SPI_disableInterrupt(EUSCI_B3_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);  // disable interrupts
    // Send 0x00 for G
    uint8_t lights;
    for(lights = 0; lights<4;lights++)
    {
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0x00;
    }
    for(lights=0;lights<11;lights++)
    {
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0xEF;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0x00;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0x00;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0xFF;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
    }
    for(lights = 0; lights<4;lights++)
    {
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0xFF;
    }
    SPI_enableInterrupt(EUSCI_B3_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);   // enable interrupts
}
void goGreen(){
    SPI_disableInterrupt(EUSCI_B3_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);  // disable interrupts
    // Send 0x00 for G
    uint8_t lights;
    for(lights = 0; lights<4;lights++)
    {
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0x00;
    }
    for(lights=0;lights<11;lights++)
    {
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0xEF;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0x00;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0xFF;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0x00;
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
    }
    for(lights = 0; lights<4;lights++)
    {
        while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
        EUSCI_B3->TXBUF = 0xFF;
    }
    SPI_enableInterrupt(EUSCI_B3_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);   // enable interrupts
}
/****************************************************************************************************
 ************************************** XBEE ******************************************************
 ****************************************************************************************************/
const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
    3,                                      // BRDIV = 78
    4,                                       // UCxBRF = 2
    0,                                       // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,                  // No Parity
    EUSCI_A_UART_LSB_FIRST,                  // MSB First
    EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
    EUSCI_A_UART_MODE,                       // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

volatile int8_t analogX_data = 0;
volatile int8_t analogY_data = 0;

volatile int8_t analogX_upper = 0;
volatile int8_t analogX_lower = 0;
volatile int8_t analogY_upper = 0;
volatile int8_t analogY_lower = 0;

volatile int8_t analogX_data;
volatile int8_t analogY_data;

volatile uint8_t switches_upper = 0;
volatile uint8_t switches_lower = 0;
volatile uint8_t switches_data = 0;

volatile uint8_t buttons = 0;

volatile uint8_t byte = 0;

int16_t leftMotorEffort = 0;
int16_t rightMotorEffort = 0;
uint16_t soundCounter = 0;
char flag = 0;

void init_clk(void);
volatile uint16_t choice = 0;
volatile uint16_t steve = 0;
volatile int rising_edge;
volatile int meas1 = 0;
volatile int meas2 = 0;
volatile int prev_rising = 0;
volatile int wait = 0;
char obstacle_detected = 0;
volatile uint32_t colorcycle = 0x00000000;
//volatile int gogo0 = (rand()%12000)+12000;
//volatile int gogo1 = (rand()%12000)+12000;

/*******************************************************************************************************************************
 **************************************************** US Sensor Controls ***********************************************************
 *******************************************************************************************************************************/
volatile uint16_t timer;
/* Timer_A Continuous Mode Configuration Parameter */
const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_64,       // SMCLK/1 = 3MHz
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
        TIMER_A_SKIP_CLEAR                   // Skup Clear Counter
};

/* Timer_A Capture Mode Configuration Parameter */
const Timer_A_CaptureModeConfig captureModeConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,        // CC Register 1
        TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,          // Rising Edge and falling
        TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxA Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};


/*********************************************************************************
 ***************************** PWM Configurations ********************************
 *********************************************************************************/
// Pin 7.7
extern Timer_A_PWMConfig pwmConfig_LeftBack =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_1,
 TIMER_A_OUTPUTMODE_RESET_SET,
 6000
};
// Pin 7.6
extern Timer_A_PWMConfig pwmConfig_RightBack =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_2,
 TIMER_A_OUTPUTMODE_RESET_SET,
 6000
};
// Pin 7.5
extern Timer_A_PWMConfig pwmConfig_LeftFront =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_3,
 TIMER_A_OUTPUTMODE_RESET_SET,
 12000
};
// Pin 7.4
extern Timer_A_PWMConfig pwmConfig_RightFront =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_4,
 TIMER_A_OUTPUTMODE_RESET_SET,
 12000
};

Timer_A_PWMConfig* pointer_LeftBack = &pwmConfig_LeftBack;
Timer_A_PWMConfig* pointer_RightBack = &pwmConfig_RightBack;
Timer_A_PWMConfig* pointer_LeftFront = &pwmConfig_LeftFront;
Timer_A_PWMConfig* pointer_RightFront = &pwmConfig_RightFront;

void randomMovement(uint16_t choice, int gogo0, int gogo1)
{
    if(choice%16 == 0) //right
    {
        pointer_LeftBack->dutyCycle = 0;
        pointer_LeftFront->dutyCycle = 12000;
        pointer_RightBack->dutyCycle = 0;
        pointer_RightFront->dutyCycle = 12000;
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
    }
    else if(choice%16 == 1) //right
        {
            pointer_LeftBack->dutyCycle = 0;
            pointer_LeftFront->dutyCycle = 0;
            pointer_RightBack->dutyCycle = 0;
            pointer_RightFront->dutyCycle = 0;
            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
        }
    else if(choice%16 == 2) //right
            {
                pointer_LeftBack->dutyCycle = 0;
                pointer_LeftFront->dutyCycle = 0;
                pointer_RightBack->dutyCycle = 0;
                pointer_RightFront->dutyCycle = 12000;
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
            }
    else if(choice%16 == 3) //right
            {
                pointer_LeftBack->dutyCycle = 0;
                pointer_LeftFront->dutyCycle = 0;
                pointer_RightBack->dutyCycle = 0;
                pointer_RightFront->dutyCycle = 0;
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
            }
    else if(choice%16 == 4) //right
            {
                pointer_LeftBack->dutyCycle = 0;
                pointer_LeftFront->dutyCycle = 0;
                pointer_RightBack->dutyCycle = 0;
                pointer_RightFront->dutyCycle = 12000;
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
            }
    else if(choice%16 == 5) //right
        {
            pointer_LeftBack->dutyCycle = 0;
            pointer_LeftFront->dutyCycle = 0;
            pointer_RightBack->dutyCycle = 0;
            pointer_RightFront->dutyCycle = 0;
            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
        }
    else if(choice%16 == 6) //right
            {
                pointer_LeftBack->dutyCycle = 0;
                pointer_LeftFront->dutyCycle = 12000;
                pointer_RightBack->dutyCycle = 0;
                pointer_RightFront->dutyCycle = 12000;
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
            }
        else if(choice%16 == 7) //right
                {
                    pointer_LeftBack->dutyCycle = 0;
                    pointer_LeftFront->dutyCycle = 0;
                    pointer_RightBack->dutyCycle = 0;
                    pointer_RightFront->dutyCycle = 0;
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                }
        else if(choice%16 == 8) //right
                {
                    pointer_LeftBack->dutyCycle = 0;
                    pointer_LeftFront->dutyCycle = 12000;
                    pointer_RightBack->dutyCycle = 0;
                    pointer_RightFront->dutyCycle = 0;
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                }
        else if(choice%16 == 9) //right
                {
                    pointer_LeftBack->dutyCycle = 0;
                    pointer_LeftFront->dutyCycle = 0;
                    pointer_RightBack->dutyCycle = 0;
                    pointer_RightFront->dutyCycle = 0;
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                }
        else if(choice%16 == 10) //right
            {
                pointer_LeftBack->dutyCycle = 0;
                pointer_LeftFront->dutyCycle = 0;
                pointer_RightBack->dutyCycle = 0;
                pointer_RightFront->dutyCycle = 12000;
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
            }
        else if(choice%16 == 11) //right
                {
                    pointer_LeftBack->dutyCycle = 0;
                    pointer_LeftFront->dutyCycle = 0;
                    pointer_RightBack->dutyCycle = 0;
                    pointer_RightFront->dutyCycle = 0;
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                }
            else if(choice%16 == 12) //right
                    {
                        pointer_LeftBack->dutyCycle = 0;
                        pointer_LeftFront->dutyCycle = 12000;
                        pointer_RightBack->dutyCycle = 0;
                        pointer_RightFront->dutyCycle = 12000;
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                    }
            else if(choice%16 == 13) //right
                    {
                        pointer_LeftBack->dutyCycle = 0;
                        pointer_LeftFront->dutyCycle = 0;
                        pointer_RightBack->dutyCycle = 0;
                        pointer_RightFront->dutyCycle = 0;
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                    }
            else if(choice%16 == 14) //right
                    {
                        pointer_LeftBack->dutyCycle = 0;
                        pointer_LeftFront->dutyCycle = 20000;
                        pointer_RightBack->dutyCycle = 0;
                        pointer_RightFront->dutyCycle = 0;
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                    }
            else if(choice%16 == 15) //right
                {
                    pointer_LeftBack->dutyCycle = 0;
                    pointer_LeftFront->dutyCycle = 0;
                    pointer_RightBack->dutyCycle = 0;
                    pointer_RightFront->dutyCycle = 0;
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                }
    else //left
    {
        pointer_LeftBack->dutyCycle = 0;
        pointer_LeftFront->dutyCycle = 0;
        pointer_RightBack->dutyCycle = 0;
        pointer_RightFront->dutyCycle = 0;
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
    }
}

/*********************************************************************************
 ************************ Peripheral Initializations *****************************
 *********************************************************************************/

void initTimer32()
{
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(TIMER32_0_BASE, 3000000);
    Interrupt_enableInterrupt(TIMER32_0_INTERRUPT);
    Timer32_enableInterrupt(TIMER32_0_BASE);
}

void initTimer32_R2D2()
{
    Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(TIMER32_1_BASE, 1200);
    Interrupt_enableInterrupt(TIMER32_1_INTERRUPT);
   // Timer32_enableInterrupt(TIMER32_1_BASE);
}
const eUSCI_SPI_MasterConfig spiMasterConfig=
{
 EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
 3000000,
 1000000,
 EUSCI_B_SPI_MSB_FIRST,
 EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
 EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,
 EUSCI_B_SPI_3PIN
};
void initSPI()//SPI for DAC
{
   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
   GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN2); //For CS
   SPI_initMaster(EUSCI_B1_BASE, &spiMasterConfig);
   SPI_enableModule(EUSCI_B1_BASE);
}
void writeDAC(uint16_t data)
{
   GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN2);
   uint8_t high = (0x90 | ((data>>6) & 0xff));
   SPI_transmitData(EUSCI_B1_BASE, high);
   uint8_t low = ((data<<2) & 0xff);
   SPI_transmitData(EUSCI_B1_BASE, low);
   GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN2);
}
volatile uint16_t position = 0;
uint16_t r2d2[] = {
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,512,
                   512,513,513,513,513,512,513,512,512,512,512,512,512,512,512,512,512,512,512,512,512,513,
                   513,513,513,513,513,513,513,513,513,513,512,512,512,512,512,513,513,513,513,513,513,513,
                   513,513,513,512,512,512,512,512,512,513,513,513,513,513,512,512,512,512,512,512,512,513,
                   513,513,513,512,512,512,513,513,513,513,513,513,513,513,513,513,513,513,513,512,513,513,
                   513,513,513,513,513,513,513,513,513,513,513,513,513,513,513,513,513,513,513,513,513,513,
                   513,513,513,513,513,513,513,513,513,513,513,514,514,513,513,513,513,514,514,514,514,514,
                   514,513,513,513,513,513,513,513,513,513,514,514,513,513,513,513,513,513,513,513,513,514,
                   514,514,514,514,513,513,513,512,512,513,513,513,513,513,514,514,514,514,514,514,513,513,
                   513,513,513,513,513,514,513,513,513,513,513,513,513,513,514,514,514,514,514,513,513,513,
                   514,513,513,514,514,514,514,514,514,514,514,514,514,514,514,514,514,513,513,514,514,513,
                   513,514,514,514,514,514,515,514,514,514,514,514,514,515,515,515,515,515,515,514,514,514,
                   514,514,513,514,514,514,514,515,515,515,515,516,516,515,515,515,515,514,513,513,513,513,
                   513,514,514,515,516,516,516,516,515,515,515,515,515,515,515,515,515,515,514,515,515,515,
                   514,514,514,515,514,515,515,515,515,515,514,514,514,514,515,515,515,515,515,515,515,515,
                   514,513,513,513,513,514,514,514,514,515,514,514,515,515,515,515,515,515,515,515,515,514,
                   514,515,514,514,514,514,514,514,514,514,514,515,515,515,516,516,515,515,515,515,514,515,
                   516,515,515,515,515,514,514,514,513,513,513,513,514,514,514,515,515,516,516,515,515,515,
                   516,515,515,515,515,514,514,514,514,514,513,513,514,514,514,514,515,515,514,513,513,513,
                   515,516,516,514,512,512,513,515,517,518,516,512,509,509,512,516,519,521,517,512,507,505,
                   510,522,530,533,521,498,481,482,515,545,560,532,487,455,469,531,575,585,523,462,427,470,
                   548,602,591,502,433,414,493,570,615,564,473,410,430,527,589,610,532,459,408,454,541,589,
                   598,521,459,417,456,529,572,596,542,476,442,433,495,554,587,582,505,465,434,443,529,565,
                   591,564,483,469,437,465,545,548,581,542,479,487,439,482,545,544,591,533,477,466,426,516,
                   561,582,577,474,440,418,493,579,610,567,479,416,416,507,583,632,547,468,405,442,539,597,
                   613,509,440,401,484,562,603,572,483,430,412,505,565,602,561,479,438,419,489,557,587,585,
                   502,451,435,449,538,573,592,552,462,454,438,478,556,562,581,529,465,484,445,485,553,536,
                   586,537,480,500,436,487,544,532,592,534,477,480,431,508,554,573,595,490,454,421,465,563,
                   606,599,510,439,406,473,555,625,588,496,432,416,496,560,619,561,486,426,437,519,568,611,
                   544,478,425,447,527,576,605,536,472,436,452,529,569,594,547,477,458,451,496,548,573,578,
                   518,481,466,449,508,552,574,579,505,484,468,455,524,536,556,569,503,501,475,456,519,525,
                   572,576,497,485,449,464,537,564,590,540,468,451,449,515,578,596,562,487,436,432,494,568,
                   617,565,492,432,431,507,571,612,549,471,425,452,529,585,591,529,465,436,474,531,580,577,
                   532,480,452,462,512,560,580,553,492,466,449,493,553,581,573,516,477,467,479,523,561,555,
                   546,505,489,487,475,521,535,546,552,514,495,493,486,509,534,528,543,513,498,503,473,518,
                   537,551,556,495,481,456,491,555,569,563,499,453,450,491,557,585,554,502,459,457,504,549,
                   578,548,501,467,460,509,546,571,553,499,463,459,504,552,569,547,501,468,474,506,546,558,
                   534,507,488,496,503,522,532,529,524,513,510,495,501,509,524,533,523,516,506,514,512,514,
                   511,507,516,525,540,525,504,493,496,520,534,537,521,501,503,510,522,524,513,514,512,518,
                   520,516,515,512,521,524,518,510,501,510,517,524,526,514,512,509,511,514,511,519,526,529,
                   526,506,491,498,515,539,539,521,498,485,510,533,542,530,506,494,497,518,534,532,521,514,
                   506,507,511,513,522,527,528,517,499,496,507,524,539,531,510,494,496,515,527,535,522,506,
                   505,509,520,515,509,510,512,522,524,512,500,499,509,525,527,518,508,499,511,520,520,520,
                   509,510,515,512,516,510,513,525,521,519,504,496,511,524,535,528,506,495,499,518,537,530,
                   519,501,497,515,524,532,518,510,504,505,520,523,526,519,515,511,506,509,516,525,530,523,
                   508,500,501,517,533,534,523,506,499,508,520,530,526,513,509,510,516,519,515,511,511,518,
                   525,523,515,509,508,515,524,525,520,511,513,515,517,518,512,515,519,520,521,510,507,509,
                   516,527,523,513,505,505,515,526,527,522,513,507,514,516,518,515,511,515,517,519,516,509,
                   509,513,522,523,519,514,509,514,520,520,520,514,512,517,518,519,516,513,516,514,514,511,
                   508,515,522,526,521,506,500,505,519,534,532,520,503,495,506,518,530,527,520,512,504,507,
                   511,517,521,526,524,517,508,502,511,521,533,531,518,506,499,508,522,531,530,517,507,503,
                   507,515,524,526,523,516,506,501,503,515,527,531,524,510,500,502,515,528,531,522,508,502,
                   506,515,524,524,519,510,506,509,514,521,524,522,517,509,505,508,514,525,528,521,510,500,
                   501,514,524,530,524,511,506,504,512,520,521,522,519,516,514,510,511,515,522,526,522,512,
                   503,503,512,523,528,522,510,505,506,516,527,528,523,511,505,506,510,520,527,525,519,509,
                   503,506,513,526,529,521,511,502,505,515,524,528,520,509,506,510,520,526,525,519,510,509,
                   512,517,522,521,518,512,507,507,512,520,526,523,515,508,504,511,520,526,523,515,510,508,
                   512,519,521,518,515,513,512,514,515,517,519,520,519,515,512,511,514,519,522,519,512,507,
                   506,511,518,522,521,517,512,511,512,515,520,523,522,518,511,509,508,513,519,520,516,509,
                   506,508,514,518,519,515,511,511,513,518,518,517,514,512,512,513,516,518,518,516,513,510,
                   511,514,518,521,518,513,509,509,514,520,523,521,513,508,509,512,518,520,521,517,512,511,
                   510,514,518,521,521,516,512,510,513,518,521,522,517,511,508,510,515,520,521,520,517,513,
                   512,513,517,521,520,517,513,510,511,514,516,516,519,527,522,505,494,509,536,546,526,490,
                   478,507,548,559,517,472,473,518,566,562,508,463,474,532,573,555,493,455,481,542,580,547,
                   479,449,486,557,587,544,474,447,489,559,585,539,468,448,499,567,582,525,459,455,515,579,
                   577,508,447,457,533,592,571,492,437,465,548,596,564,483,438,477,555,592,548,472,442,490,
                   565,587,535,463,445,503,577,586,523,451,446,517,586,583,512,444,451,528,592,574,497,441,
                   467,548,595,563,484,439,479,559,594,548,472,443,494,570,592,538,463,446,505,576,584,523,
                   455,450,515,583,579,511,451,458,531,585,565,496,448,473,546,584,549,480,445,490,563,585,
                   537,468,450,503,573,582,523,459,453,515,576,568,506,455,469,537,582,554,489,452,485,557,
                   584,539,475,454,503,569,576,522,462,459,521,579,564,500,454,474,544,584,549,481,450,491,
                   563,583,532,467,455,512,578,573,508,453,464,533,584,557,487,448,482,558,586,538,469,451,
                   507,576,578,516,458,464,533,585,561,492,449,481,552,585,541,473,454,501,568,577,521,464,
                   464,524,579,564,500,457,477,545,583,546,481,454,495,562,577,525,465,461,518,576,566,501,
                   456,478,546,583,544,478,456,500,566,575,518,463,467,527,576,558,494,459,489,554,577,529,
                   470,463,519,575,562,498,453,478,547,577,534,469,455,507,572,570,503,451,470,542,581,539,
                   470,452,503,571,571,505,454,470,541,583,544,475,453,501,568,575,515,458,462,527,580,560,
                   492,450,478,549,585,543,476,451,495,568,584,527,461,454,516,581,570,500,449,473,549,587,
                   541,470,453,512,581,571,497,452,485,558,585,529,461,462,528,583,558,484,449,496,569,582,
                   518,454,465,541,590,551,474,447,504,580,581,505,445,467,549,594,544,463,445,509,584,580,
                   500,444,473,555,597,545,465,449,511,582,578,501,445,472,547,588,543,466,451,508,575,575,
                   506,455,477,545,582,542,476,461,508,564,564,506,465,485,540,567,533,483,475,517,557,548,
                   501,478,505,545,551,514,480,495,533,550,526,488,487,523,550,537,498,480,508,544,544,509,
                   479,494,536,553,526,487,482,519,552,542,502,477,499,543,555,523,486,485,523,555,543,503,
                   480,499,539,552,525,490,485,517,549,542,510,487,499,532,546,526,497,490,513,538,535,509,
                   492,502,526,535,520,502,501,518,530,525,511,504,514,523,520,511,507,517,524,519,509,506,
                   517,527,523,509,502,512,529,530,514,500,504,522,532,521,501,498,513,531,531,512,498,504,
                   523,534,521,502,500,515,530,528,511,500,508,523,529,519,506,506,517,525,520,510,506,514,
                   523,521,513,509,514,522,522,513,507,511,520,523,516,509,510,519,525,521,511,506,512,522,
                   525,517,506,506,517,527,525,514,504,508,521,528,521,506,501,511,525,528,517,505,504,517,
                   528,524,511,503,509,521,526,518,508,507,516,523,521,513,508,513,520,520,513,509,513,518,
                   520,515,509,510,516,521,518,511,510,517,523,521,513,508,512,520,524,516,506,505,514,523,
                   521,509,501,505,517,524,519,507,501,510,523,525,515,503,502,514,525,523,509,499,503,518,
                   528,521,507,499,508,524,530,520,505,501,512,525,526,513,500,502,516,527,523,508,500,507,
                   522,529,520,506,502,512,525,526,515,504,505,515,524,522,510,504,510,519,524,520,511,509,
                   514,521,523,517,510,510,516,520,519,513,510,512,517,519,516,512,513,516,518,516,512,512,
                   517,519,517,512,510,514,519,519,513,508,511,519,523,518,510,508,514,522,522,512,505,508,
                   519,525,519,509,505,513,523,524,514,506,508,519,527,522,510,503,509,521,524,516,505,505,
                   515,524,522,513,507,511,521,524,518,509,507,513,520,520,514,509,509,515,520,518,513,511,
                   513,517,519,517,512,510,514,517,518,515,511,511,514,516,516,513,511,513,516,518,516,513,
                   512,514,515,515,513,511,511,515,517,517,515,515,516,517,518,516,514,514,515,516,516,514,
                   513,513,514,515,514,513,513,513,515,516,515,515,515,515,516,516,515,513,513,513,514,514,
                   513,512,512,513,514,514,513,513,514,514,514,514,514,514,515,516,515,514,514,514,514,513,
                   512,513,513,514,514,514,514,514,515,515,515,514,514,514,514,514,514,514,513,513,514,515,
                   515,515,515,514,514,515,514,514,514,514,514,514,514,514,514,515,514,514,513,513,515,516,
                   516,513,512,513,515,515,513,512,512,515,517,516,514,512,514,516,517,515,512,512,515,517,
                   517,514,512,513,516,517,515,513,512,514,517,518,515,512,512,516,517,514,512,516,519,511,
                   503,512,530,529,503,488,512,544,538,496,473,508,553,547,491,463,503,559,556,495,460,498,
                   561,564,501,455,487,558,573,510,452,478,555,578,516,453,468,545,581,526,456,459,534,581,
                   535,459,451,524,579,542,465,448,517,580,551,473,447,509,576,557,480,446,502,574,565,487,
                   444,494,572,572,495,442,484,567,577,502,443,479,563,583,513,445,471,558,587,520,447,465,
                   551,590,530,451,456,540,592,541,456,447,529,594,555,465,442,519,592,565,473,438,507,587,
                   573,482,437,497,582,579,492,438,489,578,585,501,436,479,572,589,508,436,471,566,594,519,
                   440,462,557,599,533,444,451,544,600,544,450,444,534,600,553,457,442,527,598,563,468,437,
                   512,594,572,474,432,503,591,578,482,432,493,584,585,494,431,479,578,595,509,434,469,570,
                   598,518,440,464,555,594,531,450,457,543,592,540,458,454,535,589,543,462,454,530,585,545,
                   466,455,525,584,552,472,451,519,583,555,474,448,512,579,561,482,448,502,572,565,491,449,
                   500,572,567,497,455,500,570,572,502,452,493,568,573,502,451,490,566,576,507,448,480,563,
                   585,516,447,470,556,590,527,450,461,545,589,538,457,452,530,588,548,464,448,522,585,553,
                   471,446,514,583,562,478,444,506,584,570,485,441,495,580,580,496,439,483,572,587,509,441,
                   470,560,591,522,446,460,547,590,533,454,457,537,588,543,463,453,528,586,550,469,448,520,
                   587,557,472,444,513,587,566,479,440,503,585,575,488,438,492,578,581,499,440,481,568,586,
                   511,443,473,559,586,519,448,468,554,585,521,449,467,553,587,526,451,463,548,590,535,455,
                   456,538,589,544,464,453,524,585,556,477,451,511,577,560,486,453,506,570,559,491,457,503,
                   565,560,497,459,496,561,565,504,457,487,557,572,513,456,476,548,578,528,460,463,535,583,
                   543,469,456,521,582,557,480,451,506,574,564,489,447,490,564,570,501,445,477,556,574,510,
                   446,467,550,583,523,450,459,543,589,536,455,449,530,589,548,465,446,515,581,557,478,449,
                   504,572,563,491,450,494,567,569,503,456,489,560,571,509,455,478,546,567,516,463,478,544,
                   573,530,475,477,533,566,533,477,470,519,560,538,486,475,519,560,543,496,481,517,553,540,
                   495,477,507,544,539,500,481,508,547,548,513,486,505,541,545,513,484,494,528,540,515,486,
                   494,528,543,523,497,499,527,543,526,498,494,517,534,522,498,492,513,536,531,507,497,516,
                   538,534,511,497,509,529,529,508,493,503,524,529,515,502,510,529,533,519,504,507,522,526,
                   514,501,503,519,526,517,505,508,522,530,522,509,506,518,526,518,505,502,513,521,516,507,
                   506,518,526,521,513,511,519,523,516,507,506,515,519,513,505,508,519,523,518,510,513,523,
                   528,520,509,509,518,523,515,504,504,516,524,519,510,510,520,527,521,511,507,514,522,518,
                   506,502,511,521,520,510,506,513,524,524,514,507,512,521,522,512,503,505,516,520,513,505,
                   506,517,525,519,509,507,517,525,521,508,503,511,521,519,508,502,512,525,525,514,505,511,
                   523,526,514,502,505,519,524,515,503,504,518,528,521,507,505,518,528,521,506,501,511,522,
                   518,505,500,511,524,524,512,505,514,526,525,512,503,510,523,524,511,501,507,521,526,516,
                   504,507,522,528,518,504,504,517,524,516,502,502,515,525,520,506,503,515,527,525,511,504,
                   514,526,525,511,501,509,522,524,511,501,507,521,527,516,504,507,520,526,517,505,505,517,
                   525,519,507,504,513,524,521,509,503,509,520,521,510,502,508,519,522,514,506,511,523,528,
                   521,511,510,519,524,516,503,499,507,515,513,503,500,509,520,520,511,507,515,524,523,511,
                   503,507,516,517,508,498,501,513,519,513,505,508,520,525,519,509,510,518,522,516,506,505,
                   512,520,517,508,505,513,522,520,512,507,512,519,519,510,504,509,518,521,515,510,512,521,
                   524,518,510,510,516,519,513,505,505,512,518,514,507,507,515,523,522,515,513,518,524,521,
                   512,507,511,517,516,509,505,508,517,521,516,510,511,519,523,517,508,507,515,520,515,508,
                   506,513,520,519,512,508,513,522,522,514,508,509,518,521,513,502,502,521,536,523,496,489,
                   518,548,541,500,471,492,539,558,522,471,468,518,566,556,497,461,489,548,574,532,471,460,
                   509,563,559,500,455,473,535,574,545,482,457,498,560,574,522,463,459,516,570,558,492,448,
                   473,541,580,543,474,450,496,566,578,518,457,462,527,582,560,489,449,479,549,582,539,471,
                   451,501,568,578,521,462,460,518,575,567,504,456,468,531,576,554,490,451,477,543,579,542,
                   476,448,488,556,578,530,466,451,504,571,579,518,454,453,518,580,568,497,443,463,537,586,
                   556,483,441,476,554,589,544,470,445,494,567,585,529,459,446,505,573,577,516,452,451,518,
                   580,571,502,446,460,531,585,565,493,443,466,543,588,555,479,439,477,554,586,542,470,442,
                   490,563,586,535,466,448,499,567,580,528,462,451,505,568,578,525,460,449,506,570,577,519,
                   455,450,510,575,578,515,452,454,518,579,578,512,450,456,524,583,572,503,446,459,526,580,
                   568,499,445,462,529,582,571,507,451,460,526,582,572,504,446,456,522,579,574,506,445,455,
                   524,584,575,505,446,456,525,583,575,505,444,457,523,582,575,505,446,457,522,582,575,507,
                   446,451,517,578,575,508,443,446,510,575,578,512,445,444,508,576,580,516,449,443,502,571,
                   584,524,455,447,501,570,586,531,463,443,490,562,587,539,466,440,481,552,590,553,478,441,
                   475,549,592,559,486,442,465,533,586,568,495,442,455,522,583,577,511,451,449,511,582,586,
                   523,457,443,498,569,590,538,464,442,490,562,590,546,476,446,477,545,584,555,488,448,468,
                   529,578,568,507,460,464,516,573,575,521,468,455,496,556,574,534,476,454,485,543,576,552,
                   496,459,473,531,575,561,506,460,462,512,564,571,522,465,458,503,561,578,536,479,455,486,
                   546,575,545,486,456,474,525,566,558,510,471,472,511,558,566,530,484,466,488,536,563,542,
                   494,466,477,521,561,556,516,479,474,504,549,557,530,492,472,484,523,553,545,512,480,478,
                   509,547,553,531,492,475,493,531,553,541,505,477,485,516,548,550,524,490,484,504,537,551,
                   533,501,481,490,519,545,537,508,486,492,517,542,541,518,496,493,510,529,530,514,500,496,
                   505,518,526,521,514,507,509,515,520,522,519,511,503,504,509,518,520,516,509,506,510,520,
                   526,524,516,505,504,512,522,521,515,504,500,509,522,526,520,510,507,513,521,523,515,508,
                   504,507,513,514,512,511,512,513,516,517,518,519,519,516,514,513,512,514,514,511,510,510,
                   512,516,517,516,517,517,516,515,516,515,514,513,510,509,510,511,512,513,514,517,517,516,
                   515,516,519,519,515,508,505,508,515,518,513,506,505,512,521,525,521,514,509,513,520,524,
                   519,509,504,507,514,519,518,513,508,508,515,522,523,516,508,506,513,519,519,512,505,505,
                   513,521,522,516,510,510,517,522,520,514,507,505,510,515,515,512,508,506,512,518,521,520,
                   513,507,508,515,517,513,507,502,504,511,516,515,512,509,510,516,520,518,515,512,510,509,
                   509,510,512,512,509,507,509,515,520,522,518,512,511,514,518,518,514,507,505,508,513,517,
                   517,513,512,514,518,521,522,518,512,509,510,514,515,513,510,509,511,514,516,517,518,518,
                   515,514,512,514,517,517,512,506,504,510,519,522,517,510,508,514,523,526,518,508,504,508,
                   516,521,517,509,504,507,516,524,524,516,509,509,516,524,524,515,504,502,509,517,520,514,
                   506,505,512,521,525,521,513,510,513,517,519,516,510,508,508,512,516,517,515,513,513,514,
                   516,518,516,513,511,511,513,514,514,514,514,513,514,515,516,518,519,516,512,511,513,515,
                   515,512,508,508,511,515,517,516,515,515,516,517,517,514,512,511,511,511,510,511,512,513,
                   514,515,515,516,516,517,517,516,513,511,511,513,514,513,511,510,512,516,520,519,515,511,
                   513,517,520,517,511,507,509,513,516,515,510,507,509,515,518,518,515,512,511,513,515,515,
                   513,510,508,510,513,515,515,514,511,512,514,516,516,514,512,512,512,513,514,514,513,512,
                   512,513,515,518,518,516,513,512,513,516,515,512,509,508,511,515,517,515,512,511,513,517,
                   519,518,513,510,510,513,515,514,511,509,509,513,517,519,516,512,511,514,517,518,515,511,
                   509,512,517,517,510,507,512,521,525,516,504,499,509,528,539,530,500,480,483,511,544,551,
                   529,490,473,489,524,555,555,524,483,467,488,527,557,554,520,480,466,488,526,557,557,526,
                   486,467,484,518,550,556,530,490,467,479,511,547,562,539,497,467,474,507,547,566,545,499,
                   464,467,501,544,567,548,502,463,464,498,541,564,548,504,466,462,493,534,562,551,511,471,
                   462,488,526,557,556,521,478,461,481,519,554,559,528,485,461,476,514,551,563,535,492,466,
                   476,510,547,563,541,499,468,470,499,535,560,549,511,476,468,491,528,558,555,521,482,466,
                   485,521,555,560,528,487,466,478,513,549,562,538,496,467,473,507,545,565,545,503,470,469,
                   499,538,563,552,511,474,464,490,531,561,558,520,479,463,485,522,555,562,530,489,465,478,
                   515,549,564,540,498,468,472,505,540,562,547,506,472,466,494,531,558,553,519,484,469,489,
                   524,552,559,530,492,468,477,508,541,560,541,504,473,473,503,537,560,548,510,475,469,493,
                   528,557,554,517,479,468,490,524,555,559,526,486,467,486,518,549,558,529,487,465,480,515,
                   548,560,536,497,470,479,511,543,558,538,498,469,473,502,537,559,546,509,476,474,502,536,
                   560,551,514,476,467,492,527,556,554,517,479,466,489,528,559,560,528,487,466,484,517,548,
                   559,529,488,465,476,509,544,561,541,502,470,473,505,538,560,546,506,472,469,496,530,557,
                   550,515,482,471,492,527,554,556,526,487,469,482,512,543,557,532,494,471,478,509,542,560,
                   544,507,475,473,501,531,554,546,509,478,472,495,527,552,551,520,487,471,489,521,549,556,
                   528,488,469,484,516,547,558,534,497,473,480,512,542,556,538,499,471,475,505,535,556,544,
                   508,479,477,503,534,555,548,515,482,474,496,523,546,545,515,485,475,495,523,548,551,525,
                   495,478,493,518,539,546,525,495,478,490,516,538,547,530,504,485,491,515,535,542,527,502,
                   484,488,510,528,538,529,508,493,492,510,527,537,533,513,495,489,502,519,530,530,515,499,
                   492,502,520,532,534,522,506,495,498,511,521,524,515,502,494,497,511,523,529,525,513,504,
                   501,511,520,523,520,509,499,496,503,515,522,524,517,508,504,509,519,524,524,517,506,499,
                   502,512,519,520,515,508,505,509,520,527,526,520,511,506,506,513,517,515,512,507,506,509,
                   518,524,523,520,514,511,513,517,519,516,512,506,504,508,514,519,518,516,513,512,516,520,
                   523,520,515,509,506,508,512,515,515,514,512,511,513,518,522,521,516,512,509,509,511,515,
                   516,512,510,510,513,517,520,522,519,516,514,513,515,514,513,510,509,508,509,512,515,517,
                   517,518,519,518,518,518,517,515,513,511,509,509,510,512,513,513,515,516,517,517,517,517,
                   515,513,512,512,512,511,511,512,513,515,517,518,518,517,515,514,513,512,511,510,509,510,
                   511,514,514,515,516,516,516,517,519,518,516,515,513,513,512,511,511,510,511,513,515,518,
                   519,519,518,517,515,515,514,513,513,512,511,511,513,514,516,517,517,516,515,514,513,512,
                   513,512,512,510,509,510,512,515,517,518,517,516,515,516,517,516,514,512,509,508,509,511,
                   513,514,515,514,514,515,517,517,517,515,514,512,511,511,513,514,515,514,513,513,515,517,
                   519,518,517,514,512,513,514,514,513,511,509,509,511,514,516,518,517,516,515,516,516,517,
                   516,514,512,510,510,511,514,515,515,514,514,515,516,518,519,518,514,511,510,511,513,514,
                   515,514,513,513,515,517,519,518,517,513,511,511,513,515,515,514,511,510,511,514,517,519,
                   518,516,514,514,515,516,516,515,512,509,509,512,515,518,518,516,514,514,516,517,517,515,
                   512,509,508,510,513,515,516,515,515,515,516,517,518,518,516,514,511,509,509,510,512,512,
                   511,510,511,513,516,518,518,516,514,512,512,512,513,512,510,508,509,511,514,516,517,516,
                   515,514,514,514,514,514,513,511,510,510,512,515,516,517,516,515,515,515,515,516,515,514,
                   512,510,511,512,514,515,515,514,514,514,515,516,517,516,515,513,512,513,514,514,514,513,
                   513,514,515,515,516,516,515,514,513,512,513,514,514,514,514,514,515,515,516,515,514,514,
                   519,519,514,505,506,517,529,527,508,492,494,521,546,543,507,479,484,521,555,552,512,472,
                   471,509,553,560,523,473,460,495,548,570,541,485,457,481,539,574,552,493,452,466,523,573,
                   566,509,458,459,508,565,575,528,468,451,491,551,576,538,477,449,477,540,579,554,491,452,
                   471,532,577,560,499,453,461,515,569,567,511,458,455,504,565,577,528,469,451,491,555,579,
                   536,472,449,486,553,584,547,480,448,480,546,584,554,486,446,468,535,583,563,494,445,460,
                   525,581,570,503,447,455,520,581,576,510,449,449,509,574,581,521,453,443,497,566,586,534,
                   462,439,485,559,587,541,470,442,480,550,585,547,475,443,476,545,584,551,481,445,476,544,
                   584,551,480,443,473,543,585,555,482,443,473,544,588,557,482,442,473,544,586,554,479,442,
                   474,543,582,549,478,443,479,548,582,545,477,445,483,552,582,542,473,447,488,555,583,543,
                   472,444,486,556,583,539,469,444,489,557,582,537,469,449,494,560,582,536,469,449,496,562,
                   576,525,461,447,498,562,578,527,462,450,502,568,581,529,462,449,504,570,580,524,456,448,
                   507,572,579,520,454,452,514,577,575,513,452,453,517,577,570,504,447,457,524,580,568,500,
                   447,466,538,584,559,492,448,473,539,577,547,477,446,484,549,572,533,474,456,500,558,563,
                   515,462,458,509,559,556,503,460,473,527,567,551,498,462,481,539,567,536,482,459,491,544,
                   562,526,474,465,509,556,558,514,470,473,523,560,545,496,465,486,536,562,535,487,469,503,
                   553,561,522,477,471,512,556,550,505,468,479,529,564,546,497,466,489,546,568,532,482,463,
                   501,557,564,515,464,466,518,568,558,501,463,481,540,575,543,484,461,493,550,569,526,471,
                   464,510,562,562,514,470,476,528,567,549,496,465,487,537,561,532,483,467,504,553,558,520,
                   479,478,521,557,545,502,471,482,525,552,533,495,479,500,541,555,530,498,489,512,542,539,
                   507,485,491,518,539,528,502,493,508,531,539,523,502,500,515,527,525,509,498,504,519,527,
                   520,509,506,517,527,526,514,506,507,517,522,514,503,501,511,522,523,515,508,513,526,532,
                   524,512,506,512,522,522,512,501,501,514,526,526,514,506,512,526,534,526,509,502,508,519,
                   522,513,501,500,513,524,526,518,509,511,520,525,521,511,505,507,514,517,513,509,509,514,
                   520,520,517,516,517,517,517,514,511,512,512,510,509,507,511,517,520,518,516,516,520,524,
                   521,514,508,506,510,513,512,510,508,510,517,523,523,519,514,514,519,520,516,511,506,508,
                   513,515,514,512,511,515,520,523,521,517,512,512,515,516,514,509,506,509,514,518,518,516,
                   516,518,521,521,518,514,513,514,513,511,510,509,512,515,516,518,517,518,521,523,521,517,
                   514,513,515,515,511,508,508,510,516,518,517,515,515,519,523,522,517,512,510,512,515,514,
                   510,508,510,516,520,519,517,516,518,520,519,515,510,509,511,514,513,510,511,515,520,524,
                   522,518,516,517,520,518,511,504,503,508,513,513,508,505,510,518,523,522,516,512,514,519,
                   518,512,504,502,508,514,516,512,509,511,518,524,523,516,510,510,515,517,515,508,504,508,
                   515,520,519,514,513,516,520,521,518,513,511,512,514,514,512,510,510,514,517,519,518,517,
                   517,519,519,517,515,512,512,512,512,511,511,513,515,517,518,518,519,520,520,519,516,513,
                   512,512,513,513,512,512,513,517,521,522,519,516,516,517,518,516,512,508,508,511,515,516,
                   515,514,516,520,523,522,517,512,513,516,518,514,508,506,509,515,519,517,513,512,517,522,
                   523,519,513,510,512,516,517,514,509,509,515,520,521,517,514,515,519,521,518,512,510,512,
                   515,517,514,510,509,513,519,520,518,514,514,517,520,519,514,509,509,514,516,514,511,510,
                   514,519,520,518,514,514,516,518,517,513,509,509,513,515,514,512,512,515,519,522,520,517,
                   515,516,518,517,513,510,511,515,517,515,512,511,514,518,520,520,516,509,507,519,532,524,
                   499,486,509,542,541,502,474,497,546,560,517,469,475,528,566,539,479,460,504,563,562,501,
                   457,484,549,573,526,465,465,526,572,547,481,453,499,564,566,502,453,479,549,577,530,464,
                   460,523,577,555,483,450,494,563,572,510,452,466,538,580,543,471,450,504,571,571,505,450,
                   473,546,581,537,466,455,515,574,562,494,451,486,556,577,523,460,464,529,575,546,478,453,
                   503,567,568,505,455,477,546,578,533,467,458,517,572,556,489,449,486,555,575,524,460,463,
                   528,577,553,484,451,494,562,575,516,458,469,536,579,547,478,453,503,565,566,505,453,473,
                   539,574,538,473,458,511,567,565,506,459,479,542,574,537,474,458,507,564,564,501,452,473,
                   538,573,536,471,455,506,564,564,501,451,472,537,572,535,469,453,505,565,565,504,454,474,
                   539,575,540,475,454,500,559,564,509,458,471,533,571,540,480,461,503,557,562,510,463,474,
                   526,563,543,488,465,498,552,565,524,477,477,520,559,547,496,467,489,536,558,528,483,477,
                   516,558,554,509,477,489,531,556,532,487,474,503,544,553,516,479,482,524,558,545,500,475,
                   495,539,558,524,479,473,510,553,552,509,474,485,533,563,540,491,470,496,544,559,523,478,
                   472,511,556,557,512,472,479,531,567,545,492,464,490,543,565,529,475,463,506,560,567,518,
                   467,469,525,571,555,497,458,477,537,573,543,482,457,495,557,575,530,474,462,510,566,562,
                   507,460,468,522,568,553,495,460,484,545,576,545,486,459,492,551,568,525,469,459,504,557,
                   564,517,468,470,523,570,558,503,462,476,532,568,546,490,459,484,543,575,547,489,460,492,
                   552,574,535,477,455,495,554,569,524,467,460,509,563,571,524,470,465,513,564,563,513,463,
                   462,510,560,565,516,464,465,520,570,566,512,461,465,521,569,558,498,452,468,526,571,559,
                   503,461,478,534,573,556,502,462,475,523,559,549,500,464,477,523,560,555,511,475,482,523,
                   558,551,509,473,477,514,548,549,511,478,482,518,553,555,519,484,481,510,544,548,516,481,
                   478,506,541,552,525,490,484,511,546,554,524,487,477,502,536,545,518,485,481,509,545,554,
                   528,495,490,513,540,542,517,489,485,505,529,535,519,501,498,511,530,537,526,509,500,504,
                   517,526,524,510,498,499,512,530,535,522,505,500,513,532,534,517,496,491,505,526,531,517,
                   498,496,513,534,541,525,504,498,511,529,531,515,496,491,506,524,528,516,502,503,517,529,
                   528,515,502,501,511,519,516,506,498,504,515,521,520,513,510,517,523,522,516,508,507,511,
                   513,511,508,506,510,516,518,518,518,517,518,518,516,513,512,510,509,508,510,514,517,518,
                   516,515,518,523,525,519,511,507,511,517,517,510,502,503,514,524,525,517,511,515,524,529,
                   524,512,505,507,516,520,515,507,505,512,522,525,520,514,513,517,522,520,514,509,508,512,
                   515,514,514,513,514,517,519,520,520,519,517,516,514,514,515,514,511,510,511,515,520,520,
                   517,514,516,520,523,520,512,507,508,513,517,515,509,507,512,520,525,522,516,512,516,522,
                   523,516,507,505,511,518,519,514,508,509,517,524,524,517,510,510,516,521,519,511,505,507,
                   516,521,520,513,511,515,522,525,520,511,508,511,516,517,512,508,508,513,519,521,517,514,
                   515,519,521,520,515,511,511,514,516,515,513,512,513,517,520,520,518,516,516,518,518,516,
                   513,512,512,513,514,513,513,515,517,518,517,515,515,517,517,514,510,509,512,516,516,513,
                   511,513,518,522,520,514,510,512,517,519,514,507,505,511,518,520,515,510,511,518,524,523,
                   517,511,511,516,519,517,511,507,510,515,517,515,513,512,516,519,520,517,514,513,513,513,
                   514,513,512,512,513,515,518,519,518,517,515,515,516,516,515,512,510,511,514,517,516,514,
                   513,516,520,519,516,513,513,514,516,516,513,511,511,514,517,517,514,513,514,517,519,518,
                   516,514,515,517,516,514,512,512,514,516,516,515,516,517,517,516,516,515,515,515,515,515,
                   513,513,515,516,516,514,512,514,517,517,515,513,512,513,515,516,514,511,511,515,521,522,
                   516,508,507,518,531,531,513,494,491,514,538,539,510,479,479,511,546,550,519,480,473,500,
                   541,558,536,491,468,484,526,559,552,512,471,469,505,552,563,529,479,461,489,538,565,546,
                   496,462,474,520,562,559,519,473,465,499,549,567,538,489,464,481,529,562,553,512,471,468,
                   506,552,563,533,487,465,486,536,565,549,506,470,472,512,555,560,527,485,468,491,538,561,
                   544,503,469,476,519,553,553,524,484,472,498,542,559,539,498,471,482,521,553,552,521,481,
                   470,496,541,565,547,502,467,475,519,559,558,523,478,463,493,541,562,544,501,468,474,518,
                   560,563,526,479,465,496,544,563,541,496,467,479,522,557,554,520,481,470,498,543,560,539,
                   497,473,485,523,553,552,521,481,471,502,546,559,533,490,469,487,528,556,548,511,475,475,
                   510,550,558,529,486,467,489,534,560,544,503,471,475,513,551,556,526,487,470,492,538,565,
                   549,504,467,474,519,558,557,523,483,470,496,540,563,546,499,465,478,524,559,552,516,481,
                   472,499,543,563,542,495,465,481,528,560,550,512,475,471,504,545,558,534,491,466,482,529,
                   562,554,511,471,470,509,553,562,530,484,466,489,534,560,548,507,469,470,512,557,563,525,
                   478,465,496,543,564,545,502,467,473,520,565,563,516,469,465,504,549,562,538,492,461,476,
                   528,568,559,508,464,466,512,559,565,530,482,463,488,538,567,552,503,463,470,517,563,565,
                   523,472,459,498,552,569,539,490,463,480,527,564,560,517,467,459,504,560,572,533,481,461,
                   486,537,569,556,507,460,462,519,571,568,514,462,463,508,556,566,538,495,455,469,537,579,
                   560,500,455,464,517,569,570,522,472,464,502,550,565,539,488,452,474,537,583,562,501,456,
                   466,527,578,564,502,461,463,509,560,567,529,473,450,487,550,573,528,470,456,490,541,562,
                   536,494,465,474,527,575,560,498,455,467,526,570,553,505,473,474,511,562,574,525,466,459,
                   505,555,567,528,475,459,496,550,570,533,476,462,499,552,568,533,486,470,492,534,565,545,
                   490,459,486,546,573,541,489,468,491,532,556,543,504,470,479,528,568,556,500,463,483,532,
                   565,552,506,472,480,518,558,558,515,474,476,517,561,561,513,471,475,519,559,555,514,477,
                   481,515,546,544,514,487,484,507,536,545,525,494,484,507,542,547,517,493,499,520,532,528,
                   515,508,506,509,523,532,516,498,505,526,531,515,500,507,524,528,519,513,508,505,515,528,
                   529,512,496,498,519,539,534,508,489,495,524,545,531,500,492,509,524,532,531,511,497,507,
                   522,529,528,517,499,488,503,531,539,517,496,501,518,530,525,517,516,512,509,514,526,526,
                   510,499,511,529,526,519,520,515,505,511,522,527,522,511,509,515,517,520,520,514,511,511,
                   514,513,513,518,516,509,507,514,520,519,512,505,511,522,523,515,510,513,519,521,515,513,
                   519,523,514,506,509,518,520,515,514,517,515,508,512,523,522,510,505,513,519,515,512,516,
                   518,511,508,517,523,517,508,509,516,521,518,514,512,514,518,522,521,517,514,513,515,517,
                   519,519,516,510,509,512,519,521,514,507,510,520,522,514,508,512,519,518,514,514,516,515,
                   512,515,522,522,513,510,516,521,517,513,514,515,515,516,521,523,516,510,513,521,520,514,
                   511,513,515,515,515,515,514,513,513,516,516,515,513,512,512,515,519,520,514,509,511,518,
                   523,518,509,509,517,522,518,513,513,515,517,515,515,518,516,510,510,516,520,516,512,512,
                   514,515,516,516,513,506,507,514,518,514,509,510,513,515,514,514,513,512,511,513,516,516,
                   513,511,511,514,517,517,514,512,512,514,517,517,513,510,513,517,518,515,514,516,518,516,
                   513,514,516,515,513,514,516,516,514,513,514,515,516,516,516,516,515,516,518,517,512,511,
                   517,520,517,515,515,515,515,516,518,518,513,510,511,515,517,516,514,512,513,517,520,519,
                   515,510,511,516,520,520,516,512,513,516,518,519,519,515,511,511,516,521,519,514,510,511,
                   515,518,519,515,511,510,516,521,521,517,514,512,514,515,518,519,514,508,510,516,521,520,
                   515,513,515,517,519,519,517,513,511,513,517,520,518,513,511,513,515,517,516,514,511,510,
                   514,519,519,514,512,513,515,516,517,516,514,513,513,515,518,518,516,513,513,513,516,517,
                   515,513,513,515,517,516,515,515,516,515,513,516,518,517,514,513,515,516,517,517,515,514,
                   514,515,518,519,517,515,514,515,516,515,514,513,514,514,515,515,515,515,514,514,515,516,
                   514,513,513,513,514,514,514,514,516,516,516,514,515,515,515,514,513,513,514,515,514,514,
                   515,516,514,513,515,516,514,515,517,514,509,514,527,524,503,496,521,542,523,486,490,534,
                   553,516,474,491,544,558,511,468,491,548,560,506,462,488,548,560,507,464,489,550,561,506,
                   464,490,551,559,499,461,496,556,554,494,462,502,560,551,492,465,509,561,547,491,472,513,
                   555,537,486,474,519,556,530,481,478,528,558,521,475,483,534,552,512,475,494,542,548,505,
                   480,509,548,538,495,482,517,549,525,483,485,526,548,518,482,491,533,547,511,480,497,541,
                   545,504,480,508,550,540,494,479,515,550,532,489,478,512,546,527,483,476,517,548,522,480,
                   482,526,551,519,478,485,532,552,515,477,490,537,550,509,474,493,539,544,503,475,497,544,
                   545,501,475,503,547,541,497,475,508,550,535,490,480,519,550,523,482,486,530,549,514,482,
                   496,538,544,506,482,504,539,534,499,489,516,541,525,495,495,526,542,514,487,504,539,536,
                   500,487,516,544,527,491,491,529,545,513,483,501,541,542,500,480,514,551,533,488,482,524,
                   553,526,484,486,529,554,522,478,488,538,552,504,469,501,552,542,489,474,519,557,530,481,
                   482,533,558,519,477,492,540,551,509,474,498,546,547,499,476,510,554,540,490,475,516,555,
                   531,481,480,531,560,520,474,488,543,557,507,468,496,551,551,494,468,510,559,542,485,472,
                   518,561,536,479,472,528,568,528,467,474,541,569,511,458,486,555,559,490,460,512,570,547,
                   478,467,529,574,532,469,475,541,569,514,465,492,556,558,495,466,512,562,538,481,476,531,
                   564,521,470,488,546,556,501,467,505,556,539,482,474,526,557,517,474,491,540,546,505,482,
                   509,544,537,499,486,515,547,530,490,488,525,544,513,480,493,531,539,506,484,505,541,537,
                   503,489,514,541,527,496,494,523,538,516,491,500,529,535,510,492,510,535,531,505,494,514,
                   536,527,498,492,518,540,524,492,494,529,543,510,483,504,539,533,497,490,518,537,520,498,
                   501,522,531,517,501,504,524,530,515,502,509,527,527,511,502,513,527,522,507,503,517,526,
                   519,506,508,522,527,515,505,513,524,522,510,507,516,521,518,512,510,512,515,519,516,508,
                   507,518,523,511,504,515,524,516,504,511,523,520,509,509,520,521,512,508,515,521,515,509,
                   512,521,521,513,508,514,521,518,508,505,514,519,513,504,506,515,516,508,505,512,520,518,
                   509,505,515,525,519,504,503,518,528,514,496,504,527,528,505,497,518,532,516,496,505,525,
                   525,509,503,512,522,521,512,507,511,520,522,511,504,511,523,523,511,504,513,524,521,508,
                   505,517,526,520,506,508,522,527,516,504,511,524,524,510,504,514,525,521,508,505,516,526,
                   520,505,503,519,531,521,503,506,527,532,511,496,511,531,524,501,500,520,529,515,501,506,
                   521,524,513,505,510,522,524,514,507,514,523,521,510,506,514,522,518,509,509,517,524,518,
                   510,511,521,524,515,507,512,522,521,512,508,514,521,518,509,508,514,520,518,510,508,514,
                   523,519,508,507,519,525,515,504,510,523,522,509,504,516,525,519,507,507,519,524,516,507,
                   510,520,521,512,506,513,521,519,509,506,515,521,518,510,512,521,524,518,510,512,519,521,
                   514,509,512,519,520,512,510,517,522,518,511,511,516,519,517,512,511,516,519,516,511,512,
                   518,521,515,510,512,519,519,511,508,514,519,516,510,510,516,519,515,509,511,516,517,512,
                   508,512,517,518,513,510,514,518,517,512,511,516,519,516,511,512,517,519,515,511,514,518,
                   517,514,512,515,518,517,513,513,515,516,514,511,513,517,517,514,512,514,518,518,513,511,
                   514,517,516,512,513,518,519,516,514,514,517,517,514,511,513,517,517,513,512,514,517,516,
                   513,513,515,518,515,512,512,515,517,514,513,515,518,519,516,513,515,518,517,513,512,515,
                   517,516,513,507,509,522,529,510,492,513,545,529,484,490,540,548,499,474,515,554,528,480,
                   488,540,552,503,473,511,556,534,480,482,538,558,506,469,507,560,538,476,476,538,564,508,
                   465,499,557,541,477,468,527,559,511,463,489,551,548,487,467,521,562,522,468,485,548,552,
                   493,469,517,559,523,471,486,547,550,492,471,519,559,521,469,486,549,550,489,466,521,564,
                   521,467,489,554,555,491,465,520,567,526,465,484,556,557,486,462,522,570,523,463,485,556,
                   558,489,462,520,572,529,464,479,554,566,495,458,516,576,534,463,479,556,564,492,457,516,
                   573,533,465,481,558,564,491,462,522,573,526,462,483,556,556,488,465,523,569,521,463,491,
                   559,548,479,470,538,566,507,463,507,567,538,472,479,548,562,497,464,517,569,527,466,484,
                   554,556,488,467,528,569,516,462,498,565,544,473,473,547,564,495,461,519,572,526,465,490,
                   561,554,481,468,535,568,508,461,502,566,539,472,478,548,561,494,463,517,567,524,468,489,
                   554,550,485,470,530,563,509,469,512,567,537,477,488,551,555,492,471,526,564,516,468,498,
                   559,543,479,477,541,562,502,468,518,567,526,471,494,555,546,484,476,534,558,502,468,514,
                   561,523,471,497,557,545,480,479,544,560,495,466,522,567,517,460,497,563,541,468,474,550,
                   564,488,458,526,576,517,456,500,574,546,463,472,559,570,482,453,531,582,513,448,499,582,
                   548,456,469,563,575,478,446,533,591,514,439,495,589,554,453,462,564,583,483,443,528,595,
                   523,441,488,585,556,454,458,563,583,481,441,531,593,512,436,496,588,544,446,469,574,576,
                   472,450,546,591,502,440,513,592,531,443,485,584,560,461,464,560,578,485,448,530,584,513,
                   446,502,583,541,454,478,571,565,470,459,551,580,491,446,527,589,516,442,498,587,545,450,
                   473,574,569,467,459,558,584,488,447,533,587,508,442,508,587,535,446,482,573,552,457,463,
                   559,566,469,451,546,578,490,445,526,585,512,441,501,585,536,445,479,575,558,460,462,559,
                   574,479,451,540,582,500,444,517,584,522,446,495,583,549,457,475,569,567,471,456,549,580,
                   491,446,526,585,513,444,506,586,537,449,486,582,560,458,464,568,578,473,446,548,592,497,
                   438,525,598,520,437,500,597,548,442,474,589,574,457,451,568,595,481,434,540,605,508,427,
                   509,603,536,432,481,593,562,445,459,579,582,465,443,556,595,488,434,532,601,511,432,510,
                   600,533,436,490,593,552,447,473,581,569,460,459,566,578,472,449,551,584,489,447,535,587,
                   505,446,522,589,522,446,504,588,542,452,482,579,563,462,460,560,579,479,445,541,588,499,
                   440,524,592,514,439,508,593,532,442,489,586,552,452,471,573,568,465,458,559,579,479,449,
                   548,587,497,449,532,591,516,450,514,587,533,455,497,578,545,462,487,572,553,467,478,567,
                   566,477,466,556,576,489,456,540,585,506,451,521,585,522,449,504,582,536,453,490,578,547,
                   456,477,570,561,468,468,558,569,481,463,546,570,491,461,533,569,503,463,522,566,512,465,
                   514,562,518,471,510,558,524,476,506,554,526,476,503,553,530,479,499,552,536,480,490,547,
                   542,484,484,543,548,491,479,535,555,500,475,529,560,508,471,520,562,515,466,512,565,520,
                   463,506,568,528,463,499,568,536,464,489,566,548,468,482,563,556,472,473,557,561,477,466,
                   551,567,484,461,543,576,499,456,528,581,517,455,511,580,531,455,494,574,543,459,481,568,
                   554,465,473,563,563,472,466,557,570,478,458,550,576,487,455,542,580,494,452,536,581,502,
                   452,528,581,510,453,518,578,519,455,504,570,525,458,497,566,533,461,488,563,543,465,474,
                   553,556,478,465,540,565,494,458,528,572,504,456,517,571,516,455,505,570,527,457,495,571,
                   540,462,484,566,553,468,473,561,563,476,467,552,569,484,464,545,570,490,461,539,570,497,
                   464,533,567,501,466,532,567,503,466,529,569,508,466,523,569,516,466,513,564,523,470,507,
                   562,528,474,505,561,530,474,503,561,535,474,496,559,540,474,488,556,547,477,482,552,554,
                   482,474,547,560,488,468,539,567,499,467,532,568,505,464,525,567,508,462,519,568,512,459,
                   514,572,521,459,504,572,535,464,490,562,546,473,480,556,555,480,474,549,562,487,467,542,
                   569,495,461,534,575,505,456,520,578,522,456,502,572,534,458,488,569,547,463,477,563,556,
                   469,470,558,567,479,464,551,569,484,460,544,572,488,453,537,577,495,454,536,585,509,453,
                   524,582,517,457,512,575,526,460,503,569,533,466,496,565,542,470,486,560,552,476,476,551,
                   558,484,469,542,565,494,466,535,569,503,462,524,571,512,460,513,567,518,463,506,565,527,
                   466,500,562,533,470,492,559,546,476,480,553,557,487,470,536,564,501,464,520,565,516,465,
                   506,563,528,467,495,560,540,472,484,556,554,480,473,545,565,494,463,531,572,509,459,512,
                   573,527,460,495,569,545,468,477,556,562,484,462,536,572,504,456,515,575,526,459,496,570,
                   542,462,478,562,558,473,461,547,575,493,452,527,586,519,447,497,582,546,454,471,569,571,
                   470,451,548,586,491,440,528,597,515,436,502,595,544,443,472,580,568,458,452,560,587,482,
                   438,535,598,510,433,504,598,543,439,472,581,570,459,449,558,591,488,441,529,594,518,442,
                   499,586,547,453,471,562,562,472,454,537,576,503,451,507,574,533,458,477,554,557,478,460,
                   529,567,509,460,501,562,538,475,480,539,556,502,469,509,559,533,476,483,543,555,497,468,
                   512,564,533,474,484,549,560,498,469,521,568,526,469,491,555,551,486,470,529,567,516,465,
                   496,562,551,484,470,534,571,515,462,496,567,548,474,469,543,571,506,462,507,570,543,474,
                   476,545,567,504,462,509,570,539,472,479,548,563,498,465,516,565,529,474,491,546,549,501,
                   485,517,544,527,498,496,519,534,522,497,492,520,542,523,486,493,541,553,504,472,514,564,
                   537,472,477,548,568,500,456,512,579,543,461,469,554,576,495,448,511,580,542,464,472,549,
                   570,504,462,503,561,546,485,474,525,561,525,475,485,540,556,506,467,500,558,546,482,468,
                   529,569,521,464,486,554,558,494,462,511,566,538,476,479,540,563,512,473,504,552,544,497,
                   482,513,546,536,499,481,511,552,541,491,474,523,567,532,471,481,547,566,505,462,503,561,
                   546,482,468,524,563,526,472,482,539,557,508,471,498,548,548,501,477,504,546,545,503,478,
                   504,549,547,498,470,504,555,546,488,467,514,562,537,479,473,526,563,528,477,481,534,560,
                   520,475,487,538,554,515,479,492,533,549,520,486,490,529,555,527,482,482,533,565,524,466,
                   476,547,571,508,453,490,565,562,488,456,513,571,540,474,472,528,558,519,478,492,532,541,
                   515,498,501,518,537,535,506,485,507,547,542,492,470,513,557,539,486,472,519,561,539,484,
                   473,520,562,540,482,469,519,568,544,478,464,522,574,544,472,460,524,576,540,468,459,525,
                   576,542,473,460,518,569,548,483,454,502,569,567,489,443,496,578,573,487,441,494,571,568,
                   493,445,483,558,574,509,447,467,546,582,524,448,455,539,588,534,450,450,534,590,541,455,
                   447,527,587,550,467,446,512,578,561,482,444,497,570,573,500,443,479,561,584,514,446,470,
                   551,589,532,454,453,532,593,551,459,439,517,591,563,470,438,503,584,577,489,437,488,578,
                   591,505,435,470,563,595,525,444,453,538,598,554,459,431,514,609,585,467,413,497,612,598,
                   471,405,486,605,604,485,410,476,592,606,506,426,459,558,602,542,450,434,519,606,575,463,
                   420,501,599,584,480,427,484,575,588,515,447,461,540,591,552,469,443,506,582,575,493,440,
                   478,562,586,524,453,456,533,590,558,474,440,500,584,585,500,434,470,567,597,524,442,451,
                   540,594,551,467,440,504,584,579,494,435,480,571,587,512,448,469,541,576,540,477,452,499,
                   564,573,507,446,474,558,591,531,456,459,536,587,552,477,452,506,568,566,508,459,474,538,
                   577,546,478,454,505,572,571,505,453,475,544,580,541,470,448,507,579,571,493,440,481,561,
                   580,522,459,461,522,574,561,492,445,483,559,578,519,455,466,531,571,547,490,464,493,544,
                   564,531,478,463,505,560,567,511,454,467,537,585,549,469,442,500,579,585,504,435,464,557,
                   605,541,444,436,524,599,569,472,431,486,571,591,519,441,454,540,596,552,466,446,508,572,
                   565,504,460,474,529,570,551,487,453,491,558,577,520,457,464,531,579,551,479,453,501,560,
                   565,512,465,475,524,559,548,499,467,486,538,567,540,484,463,501,559,568,512,460,474,535,
                   570,540,485,466,494,542,568,537,475,451,502,575,576,496,441,473,551,584,534,467,455,505,
                   568,571,505,452,473,544,578,537,475,458,503,560,568,514,462,467,519,559,544,497,471,488,
                   529,553,529,485,470,499,544,553,514,475,474,510,553,550,505,469,479,527,560,538,492,473,
                   498,541,550,516,481,480,520,554,538,492,468,496,545,558,522,478,471,512,554,551,506,472,
                   486,530,552,526,487,482,514,546,539,502,478,494,530,547,528,500,493,510,530,530,515,504,
                   503,513,524,525,514,503,504,520,532,527,508,497,505,524,537,530,509,493,500,524,539,527,
                   502,493,508,528,530,514,500,503,521,530,521,501,495,514,537,534,509,491,501,523,530,520,
                   507,506,516,522,518,511,507,510,521,529,521,505,498,509,523,524,515,510,514,514,510,511,
                   521,524,516,511,514,516,512,512,519,522,515,507,507,514,517,516,517,518,512,505,507,520,
                   526,517,507,507,516,519,515,512,512,516,517,514,509,507,510,517,520,513,506,508,517,521,
                   515,509,512,522,528,519,505,504,516,529,525,509,501,509,522,522,511,504,511,523,524,512,
                   502,506,522,530,520,504,497,509,525,528,514,500,503,517,524,518,508,506,514,521,518,511,
                   507,510,518,522,518,509,503,508,517,520,515,509,508,515,520,518,511,506,510,519,523,518,
                   508,504,509,519,524,517,506,503,511,520,521,514,509,512,518,520,515,509,509,517,523,519,
                   510,504,508,519,523,518,507,503,511,521,523,515,508,509,518,524,518,507,502,510,522,523,
                   513,504,504,514,522,521,512,507,511,519,519,513,507,509,517,521,517,510,506,509,518,525,
                   522,511,502,507,521,528,518,504,503,514,522,519,510,507,513,520,521,514,507,508,515,523,
                   522,511,503,506,517,523,517,506,503,512,521,520,509,503,510,523,526,515,503,504,516,525,
                   520,509,506,511,518,517,511,507,509,515,517,514,508,508,513,518,518,511,505,506,512,515,
                   515,511,509,510,513,514,513,511,510,513,515,515,512,509,509,512,516,515,511,507,509,514,
                   518,516,513,511,512,515,516,514,511,512,515,518,516,510,507,511,518,521,516,509,507,512,
                   520,520,513,507,510,517,520,514,507,508,516,521,516,509,506,512,519,520,517,511,510,514,
                   518,517,512,509,512,516,516,514,511,512,516,518,518,514,511,511,514,518,518,514,511,512,
                   514,515,513,513,514,515,514,513,512,512,514,515,516,515,514,513,512,513,515,516,515,514,
                   511,511,515,518,517,512,510,513,517,516,513,512,514,518,518,514,509,509,515,519,517,511,
                   508,512,518,518,514,511,512,515,516,513,511,511,513,516,516,514,511,510,513,517,519,516,
                   511,509,513,516,516,512,509,511,515,516,512,509,511,516,519,517,512,510,513,519,520,515,
                   510,509,514,517,515,511,509,512,517,517,514,510,511,517,521,518,511,508,511,517,518,513,
                   509,511,514,516,514,511,511,514,517,516,513,510,511,514,516,515,512,510,512,514,515,514,
                   512,513,514,514,513,512,512,513,514,514,513,512,512,512,513,515,515,513,512,513,514,515,
                   515,513,512,513,515,515,513,510,511,515,517,514,510,510,513,516,516,513,510,512,516,516,
                   516,514,513,515,518,516,511,510,513,516,516,514,512,512,513,516,517,513,510,512,515,516,
                   515,513,512,514,517,517,515,512,512,514,514,513,511,511,512,515,515,512,511,512,513,514,
                   515,513,511,513,516,516,514,512,513,515,516,516,515,513,514,516,516,513,512,514,516,516,
                   514,512,512,514,516,515,513,512,514,516,515,513,512,513,514,514,513,512,513,513,513,512,
                   511,511,511,512,512,512,513,513,512,511,511,512,512,513,514,513,513,514,514,514,513,513,
                   514,514,514,513,513,514,515,514,511,510,512,514,514,513,513,513,513,513,513,512,511,512,
                   513,514,513,512,511,512,513,514,513,512,513,515,515,514,513,514,515,515,514,513,513,514,
                   514,514,514,513,513,513,515,516,516,515,514,514,514,514,514,513,513,514,516,514,513,513,
                   514,515,515,513,512,511,512,515,516,517,516,515,513,511,510,512,515,519,520,517,512,508,
                   506,507,509,512,517,525,531,528,519,506,493,487,493,508,530,549,552,539,515,484,464,467,
                   490,528,561,568,551,523,483,457,462,484,521,558,562,549,533,492,466,472,476,501,551,558,
                   538,539,519,472,465,481,483,514,566,572,538,528,514,463,434,471,502,514,567,608,566,510,
                   507,467,407,426,489,517,552,618,614,541,509,481,408,398,456,500,548,623,631,568,535,485,
                   401,394,440,484,564,631,610,574,535,449,405,419,442,520,590,579,581,563,479,455,453,432,
                   496,544,527,567,572,505,501,496,453,482,530,505,514,556,530,492,497,501,480,498,551,551,
                   503,521,532,467,444,502,511,499,565,597,537,505,517,459,408,450,502,513,562,622,590,528,
                   518,464,394,421,469,498,571,629,592,552,526,446,397,423,455,521,600,604,580,559,478,429,
                   438,433,493,572,563,569,574,504,475,475,445,487,535,514,537,562,513,496,511,478,477,527,
                   536,515,526,538,506,467,486,510,490,521,580,557,510,534,509,432,440,491,498,525,588,599,
                   558,527,504,446,415,448,488,517,572,614,585,544,514,445,400,425,463,514,589,619,588,552,
                   494,429,415,433,484,559,586,583,576,520,466,452,440,467,524,537,552,565,531,512,499,466,
                   479,512,503,511,541,529,514,514,500,488,493,514,530,515,515,534,501,465,493,506,497,543,
                   572,537,521,522,483,451,462,483,500,531,566,582,559,529,504,453,427,448,481,521,578,605,
                   579,544,501,443,419,440,483,542,585,593,574,530,478,448,440,463,514,545,557,565,543,513,
                   493,475,480,497,496,515,540,535,541,539,507,494,498,495,501,516,524,527,520,511,516,514,
                   515,530,520,501,513,517,497,501,516,515,519,532,530,524,522,511,493,480,486,500,515,537,
                   555,553,536,513,486,466,463,483,517,545,563,565,540,503,478,463,468,498,529,544,549,545,
                   522,501,495,491,494,501,508,522,534,533,531,523,503,495,499,500,510,527,527,522,523,515,
                   509,509,511,511,508,507,512,517,516,519,523,521,516,514,507,502,509,513,511,514,517,516,
                   513,515,517,517,518,517,509,502,503,508,522,532,535,533,517,494,489,494,503,520,533,529,
                   525,521,510,506,511,509,509,509,511,518,523,521,520,516,506,502,502,504,514,525,522,519,
                   522,515,508,506,506,507,511,516,517,519,520,521,517,512,510,505,500,501,512,521,528,533,
                   529,515,503,496,494,504,521,528,525,517,512,509,511,516,519,517,510,506,510,515,520,525,
                   522,512,509,509,507,511,520,522,520,515,509,508,510,512,518,523,521,519,513,506,506,510,
                   512,516,522,521,516,512,509,508,513,517,516,517,517,516,515,513,513,510,507,505,511,519,
                   528,533,525,510,500,496,497,511,528,533,530,523,511,503,503,505,509,515,519,521,522,521,
                   519,516,511,507,507,508,511,518,524,523,519,513,505,502,505,511,519,526,527,524,514,504,
                   502,506,511,520,526,524,518,512,506,506,512,515,515,515,516,515,515,514,511,508,506,506,
                   511,518,524,524,515,504,499,500,506,517,528,530,527,519,506,497,498,503,510,520,525,525,
                   522,516,509,506,505,506,510,515,519,524,525,521,517,511,504,502,506,511,519,528,529,524,
                   514,505,501,505,511,519,525,523,519,516,511,510,512,511,511,514,516,518,520,519,516,513,
                   510,510,513,517,521,521,515,510,508,508,512,518,524,526,523,516,508,503,505,510,517,525,
                   528,525,520,513,506,505,506,508,514,520,523,525,524,520,514,509,504,503,509,516,523,529,
                   527,519,510,504,502,506,514,520,524,523,520,515,511,509,511,512,513,517,519,518,519,517,
                   514,512,511,512,515,519,521,521,517,513,509,507,510,514,519,523,521,516,510,505,505,509,
                   515,521,526,525,519,513,507,504,506,510,515,520,523,523,521,515,509,506,504,507,514,521,
                   527,530,525,515,509,504,503,508,516,521,525,524,520,515,511,510,510,513,515,517,518,518,
                   517,516,514,512,512,511,513,515,517,518,517,515,513,512,513,515,517,518,517,515,511,509,
                   509,512,515,519,521,519,515,511,508,507,510,513,517,520,521,519,517,513,509,507,508,510,
                   515,520,523,524,521,515,510,507,506,510,516,521,524,522,517,512,510,509,510,513,516,517,
                   519,519,518,517,516,514,512,512,513,514,516,517,517,516,515,513,513,513,514,516,517,516,
                   516,514,513,514,515,516,517,516,514,513,512,512,514,516,517,518,518,516,515,514,511,511,
                   512,513,516,519,520,520,518,514,511,510,511,514,517,520,521,519,515,512,511,511,512,513,
                   515,517,517,517,516,516,515,514,513,512,512,513,515,517,517,518,516,514,513,512,512,514,
                   516,517,518,517,516,515,514,513,512,513,513,513,513,514,515,515,514,514,513,513,513,514,
                   514,514,514,514,514,515,515,515,514,513,512,512,513,514,516,517,517,516,515,513,512,512,
                   513,513,514,515,515,515,515,515,515,514,514,513,513,514,515,516,517,517,516,514,513,511,
                   511,513,515,516,517,517,515,514,514,514,514,514,515,516,517,517,516,516,514,514,513,514,
                   515,516,516,517,516,515,515,515,515,516,516,516,515,515,515,515,515,516,516,515,515,515,
                   514,515,515,515,516,517,516,515,516,515,514,514,514,514,514,515,515,516,517,517,516,514,
                   513,513,513,515,516,517,518,517,515,513,513,513,514,515,516,517,518,518,517,515,513,513,
                   513,513,515,517,517,517,515,514,514,513,513,515,517,517,516,515,515,515,515,513,512,515,
                   517,516,513,512,518,518,511,511,518,520,516,514,518,519,514,513,512,506,514,524,519,512,
                   515,527,521,506,506,521,522,506,512,523,521,511,506,512,519,524,517,513,516,518,515,507,
                   506,515,520,518,516,514,517,517,515,512,512,516,517,512,508,517,521,514,511,515,518,510,
                   508,516,521,513,509,515,517,513,510,517,520,516,514,517,518,514,515,519,516,511,513,517,
                   517,514,516,519,517,515,517,519,517,513,516,519,515,514,517,517,515,513,513,517,518,515,
                   515,514,513,512,515,519,517,513,516,516,514,515,515,514,515,513,513,514,513,513,514,514,
                   515,516,517,517,518,519,518,516,515,516,516,515,515,517,518,516,514,515,516,515,514,515,
                   516,517,517,516,516,515,514,514,515,515,515,515,516,516,517,517,516,514,514,515,516,516,
                   515,515,515,515,514,515,516,516,514,514,517,517,515,513,513,516,517,516,515,515,515,513,
                   512,512,514,514,513,512,514,515,515,514,514,514,515,516,517,516,515,514,512,512,514,514,
                   514,514,515,515,515,514,514,515,514,514,514,516,516,515,515,515,514,513,513,514,514,515,
                   516,516,516,518,517,516,515,515,515,515,515,515,515,514,515,515,514,514,514,516,516,515,
                   515,515,516,516,516,516,517,517,515,514,515,515,515,515,516,517,517,516,516,516,516,516,
                   515,515,516,517,516,515,515,515,516,516,515,515,516,516,515,515,515,516,515,515,515,515,
                   515,516,516,515,514,514,515,515,514,514,516,517,516,517,516,515,515,516,515,515,516,516,
                   516,515,516,517,516,515,515,515,517,517,515,513,515,516,516,516,515,516,516,515,513,513,
                   514,515,515,514,516,517,516,515,514,514,515,515,515,515,515,515,515,515,515,515,515,515,
                   515,514,515,515,515,515,515,515,515,515,514,514,515
};
//![Simple GPIO Config]
int main(void)
{
    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();
    /* Setting DCO to 12MHz */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);


    /* Initialize TX and RX for UART */
    initStrip();
    goBlue();
    /* Configuring P2.4 as peripheral input for capture */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    /* P2.5 Trigger Pin for Ultra Sonic */
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
    /* Test signal for US */
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    MAP_UART_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);

    // PWM for Motors
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P7, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring Continuous Mode */
    MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &continuousModeConfig);

    MAP_Interrupt_enableInterrupt(INT_TA1_N);
    initSPI();
    initTimer32();
    initTimer32_R2D2();

    /* Configuring Capture Mode */
    Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);
    /* Configuring Continuous Mode */
    Timer_A_configureContinuousMode(TIMER_A0_BASE, &continuousModeConfig);
    Interrupt_enableInterrupt(INT_TA0_N);

    MAP_Interrupt_enableMaster();

    /* Starting the Timer_A0 in continuous mode */
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
    MAP_Timer32_startTimer(TIMER32_0_BASE, false);
    MAP_Timer32_startTimer(TIMER32_1_BASE, false);



    while (1)
    {
        if(soundCounter == 5000)
        {
            Timer32_enableInterrupt(TIMER32_1_BASE);
            soundCounter = 0;
        }
        soundCounter++;
        switches_data = ((switches_upper & 0x0F) << 4) + (switches_lower & 0x0F);
        analogY_data = (((analogY_upper & 0x0F) << 4) + (analogY_lower & 0x0F));
        analogX_data = (((analogX_upper & 0x0F) << 4) + (analogX_lower & 0x0F));

/*******************************************************************************************************************************************
 ********************************************************* MANUAL MODE *********************************************************************
 *******************************************************************************************************************************************/
        if ((switches_data & 0x07) == 0x01)
        {
            goBlue();
        leftMotorEffort = (int16_t) analogY_data + (int16_t) analogX_data;
        rightMotorEffort = (int16_t) analogY_data - (int16_t) analogX_data;
        if ((abs(analogY_data) >= 20 || abs(analogX_data) >= 20) && ((buttons & 0x01) == 0x00))
        {
            if (leftMotorEffort > 5)
            {
                pointer_LeftFront->dutyCycle = 120*leftMotorEffort;
                pointer_LeftBack->dutyCycle = 0;
            }
            else if (leftMotorEffort < -5)
            {
                pointer_LeftBack->dutyCycle = 120*abs(leftMotorEffort);
                pointer_LeftFront->dutyCycle = 0;
            }
            else
            {
                pointer_LeftFront->dutyCycle = 0;
                pointer_LeftBack->dutyCycle = 0;
            }

            if (rightMotorEffort > 5)
            {
                pointer_RightFront->dutyCycle = 120*rightMotorEffort;
                pointer_RightBack->dutyCycle = 0;
            }
            else if (rightMotorEffort < -5)
            {
                pointer_RightBack->dutyCycle = 120*abs(rightMotorEffort);
                pointer_RightFront->dutyCycle = 0;
            }
            else
            {
                pointer_RightFront->dutyCycle = 0;
                pointer_RightBack->dutyCycle = 0;
            }
        }
        else
        {
            pointer_LeftBack->dutyCycle = 0;
            pointer_LeftFront->dutyCycle = 0;
            pointer_RightBack->dutyCycle = 0;
            pointer_RightFront->dutyCycle = 0;
        }
        //goBlue();
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
        volatile int count = 0;
        while (count < 2501)
            count++;
        }//end of manual mode
/*******************************************************************************************************************************************
 ********************************************************* AUTONOMOUS MODE *********************************************************************
 *******************************************************************************************************************************************/
        else if ((switches_data & 0x07) == 0x02)
        {
            Timer32_disableInterrupt(TIMER32_1_BASE);
            goGreen();
            //GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
            volatile int count = 0;
            //while (count < 100000)
            //    count++;
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
            //count = 0;
            while (count < 10)
                count++;
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
            volatile int waiting = 0;
            srand(time(0));

            steve++;
            if (obstacle_detected == 1) // Obstacle
            {

                    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);
                    //move back
                    pointer_LeftBack->dutyCycle = 13000;
                    pointer_LeftFront->dutyCycle = 0;
                    pointer_RightBack->dutyCycle = 13000;
                    pointer_RightFront->dutyCycle = 0;
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                    //choice++;
                    //randomMovement((uint16_t)choice, 0, 1);
                    for(waiting=0;waiting<300000;waiting++);
                    pointer_LeftBack->dutyCycle = 0;
                                        pointer_LeftFront->dutyCycle = 0;
                                        pointer_RightBack->dutyCycle = 0;
                                        pointer_RightFront->dutyCycle = 0;
                                        Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                                                            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                                                            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                                                            Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                                                            //obstacle_detected = 0;



            }
            else // No obstacle
            {
                //obstacle_detected = 0;
                GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
                choice++;
                randomMovement((uint16_t)choice, 0, 1);
                //move random
                //pointer_LeftBack->dutyCycle = 18000;
                //pointer_LeftFront->dutyCycle = 0;
                //pointer_RightBack->dutyCycle = 18000;
                //pointer_RightFront->dutyCycle = 0;
                //Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                //Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                //Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                //Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                for(waiting=0;waiting<300000;waiting++);
                //{
                    //if(obstacle_detected==1)
                    //{
                        //randomMovement(choice, gogo0, gogo1);
                      //  pointer_LeftBack->dutyCycle = 0;
                                    //        pointer_LeftFront->dutyCycle = 0;
                                  //          pointer_RightBack->dutyCycle = 0;
                                //            pointer_RightFront->dutyCycle = 0;
                              //              Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                            //                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                          //                  Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                        //                    Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
                      //                      obstacle_detected = 0;
                    //    break;
                  //  }
                //}
                //while(waiting<100000)
                  //  waiting++;
            }
        }
/*******************************************************************************************************************************************
 ********************************************************* PARTY MODE *********************************************************************
 *******************************************************************************************************************************************/
        else if ((switches_data & 0x07) == 0x04)
        {
            pointer_LeftBack->dutyCycle = 0;
                                pointer_LeftFront->dutyCycle = 0;
                                pointer_RightBack->dutyCycle = 0;
                                pointer_RightFront->dutyCycle = 12000;
                                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftFront);
                                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightBack);
                                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_LeftBack);
                                Timer_A_generatePWM(TIMER_A1_BASE, &pwmConfig_RightFront);
            //goRed();
            colorcycle += 0x0000000F;
            SPI_disableInterrupt(EUSCI_B3_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);  // disable interrupts
                // Send 0x00 for G
                uint8_t lights;
                for(lights = 0; lights<4;lights++)
                {
                    while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
                    EUSCI_B3->TXBUF = 0x00;
                }
                for(lights=0;lights<11;lights++)
                {
                    while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
                    EUSCI_B3->TXBUF = 0xEF;
                    while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
                    EUSCI_B3->TXBUF = (uint8_t)((colorcycle >> 16) & 0xFF);
                    while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
                    EUSCI_B3->TXBUF = (uint8_t)((colorcycle >> 8) & 0xFF);
                    while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
                    EUSCI_B3->TXBUF = (uint8_t)(colorcycle & 0xFF);
                    while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
                    colorcycle += 0x0000000F;
                }
                for(lights = 0; lights<4;lights++)
                {
                    while (!(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG));
                    EUSCI_B3->TXBUF = 0xFF;
                }
                SPI_enableInterrupt(EUSCI_B3_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);   // enable interrupts
                uint32_t foo = 0;
                for(foo = 0; foo<10000;foo++);
        }
        else
        {

        }
    }//end of while
}//end of main



void EUSCIA2_IRQHandler(void)
{

    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
    uint8_t byte = UCA2RXBUF;

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        if ((byte & 0xE0) == 0x00) {
            analogX_upper = (byte & 0x0F);
        }
        else if ((byte & 0xE0) == 0x20) {
            analogX_lower = (byte & 0x0F);
        }
        else if ((byte & 0xE0) == 0x40) {
            analogY_upper = (byte & 0x0F);
        }

        else if ((byte & 0xE0) == 0x60) {
            analogY_lower = (byte & 0x0F);
        }
        else if ((byte & 0xE0) == 0x80) {
            switches_upper = (byte & 0x0F);
        }
        else if ((byte & 0xE0) == 0xA0) {
            switches_lower = (byte & 0x0F);
        }
        else if ((byte & 0xE0) == 0xC0) {
            buttons = (byte & 0x03);
        }

        else {
        }
    }
    EUSCI_A2->IFG &= 0xFFFE;

}

void T32_INT1_IRQHandler(void)
{

    Timer32_clearInterruptFlag(TIMER32_0_BASE);
}
void T32_INT2_IRQHandler(void)
{
    if(position == 13125)
    {
       position = 0;
       Timer32_disableInterrupt(TIMER32_1_BASE);
    }
    writeDAC(r2d2[position++]);
    Timer32_clearInterruptFlag(TIMER32_1_BASE);
}
void TA0_N_IRQHandler(void)
 {
     if ((P2IN & 0x10) != 0)
         rising_edge = 1;
     else
         rising_edge = 0;
     if (rising_edge == 1) // Start
     {
         meas1 = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
     }
     else
     {
         meas2 = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
     }
     if (rising_edge == 0) {
         if (meas2 >= meas1) {
             timer = meas2 - meas1;
         }
         else {
             timer = meas2 + (0xFFFF - meas1);
         }
     }
     if (timer > 300)
     {
         obstacle_detected = 0;
         //randomMovement((uint16_t)choice, 0, 1);
     }
     else
     {
         obstacle_detected = 1;
         //pointer_LeftBack->dutyCycle = 18000;
         //pointer_LeftFront->dutyCycle = 0;
         //pointer_RightBack->dutyCycle = 18000;
         //pointer_RightFront->dutyCycle = 0;

     }
     Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
 }
