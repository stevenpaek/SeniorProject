/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 * MSP432 GPIO - Toggle Output High/Low
 *
 * Description: In this very simple example, the LED on P1.0 is configured as
 * an output using DriverLib's GPIO APIs. An infinite loop is then started
 * which will continuously toggle the GPIO and effectively blink the LED.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P1.0  |---> P1.0 LED
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *
 ******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#include "UF_LCD.h"


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

char flag = 0;

void init_clk(void);

const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,           // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // SMCLK/1 = 3MHz
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
extern Timer_A_PWMConfig pwmConfig_LeftWheel0 =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_1,
 TIMER_A_OUTPUTMODE_RESET_SET,
 6000
};
extern Timer_A_PWMConfig pwmConfig_RightWheel0 =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_2,
 TIMER_A_OUTPUTMODE_RESET_SET,
 6000
};
extern Timer_A_PWMConfig pwmConfig_LeftWheel1 =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_3,
 TIMER_A_OUTPUTMODE_RESET_SET,
 12000
};
extern Timer_A_PWMConfig pwmConfig_RightWheel1 =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_4,
 TIMER_A_OUTPUTMODE_RESET_SET,
 12000
};
extern Timer_A_PWMConfig pwmConfig_LeftWheel0_Stop =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_1,
 TIMER_A_OUTPUTMODE_RESET_SET,
 0
};
extern Timer_A_PWMConfig pwmConfig_RightWheel0_Stop =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_2,
 TIMER_A_OUTPUTMODE_RESET_SET,
 0
};
extern Timer_A_PWMConfig pwmConfig_LeftWheel1_Stop =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_3,
 TIMER_A_OUTPUTMODE_RESET_SET,
 0
};
extern Timer_A_PWMConfig pwmConfig_RightWheel1_Stop =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 24000,
 TIMER_A_CAPTURECOMPARE_REGISTER_4,
 TIMER_A_OUTPUTMODE_RESET_SET,
 0
};
extern Timer_A_PWMConfig pwmConfig_UltraSonic =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_1,
 65535,
 TIMER_A_CAPTURECOMPARE_REGISTER_4,
 TIMER_A_OUTPUTMODE_RESET_SET,
 30
};

/*********************************************************************************
 ************************** Motor Control Functions ******************************
 *********************************************************************************/
void testForward()
{
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel1);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel1);
}
void testRight()
{
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel1);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel1_Stop);
}
void testStop()
{
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel1_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel1_Stop);
}
void moveForward()
{
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel0);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel0);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel1_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel1_Stop);
}
void moveBackward()
{
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel0_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel0_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel1);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel1);
}
void moveRight()
{
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel0);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel0_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel1_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel1_Stop);
}
void moveLeft()
{
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel0_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel0);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel1_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel1_Stop);
}
void moveStop()
{
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel0_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel0_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_LeftWheel1_Stop);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfig_RightWheel1_Stop);
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

void initTimer32_Sound()
{
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
    Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(TIMER32_1_BASE, 10);
    Interrupt_enableInterrupt(TIMER32_1_INTERRUPT);
    Timer32_enableInterrupt(TIMER32_1_BASE);
}

//![Simple GPIO Config]
int main(void)
{
    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();

    /* Setting DCO to 12MHz */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    /* Initialize TX and RX for UART */

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    MAP_UART_enableModule(EUSCI_A2_BASE);

    char analogX[5];
    char analogY[5];

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
    //MAP_Interrupt_enableSleepOnIsrExit();
    //MAP_Interrupt_enableMaster();

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1);

    // PWM for Motors
    //GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4,GPIO_PRIMARY_MODULE_FUNCTION);
    //GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6,GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
    /* Configuring P2.4 as peripheral input for capture */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN5);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
    /* Configuring Capture Mode */
    MAP_Timer_A_initCapture(TIMER_A0_BASE, &captureModeConfig);

    /* Configuring Continuous Mode */
    MAP_Timer_A_configureContinuousMode(TIMER_A0_BASE, &continuousModeConfig);

    MAP_Interrupt_enableInterrupt(INT_TA0_N);
    initTimer32();
    //initTimer32_Sound();
    MAP_Interrupt_enableMaster();


    MAP_GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN1);

    /* Starting the Timer_A0 in continuous mode */
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
    MAP_Timer32_startTimer(TIMER32_0_BASE, false);
    MAP_Timer32_startTimer(TIMER32_1_BASE, false);

    //testForward();

    while (1)
    {
        switches_data = ((switches_upper & 0x0F) << 4) + (switches_lower & 0x0F);
        analogY_data = (((analogY_upper & 0x0F) << 4) + (analogY_lower & 0x0F));
        analogX_data = (((analogX_upper & 0x0F) << 4) + (analogX_lower & 0x0F));

        leftMotorEffort = (int16_t) analogY_data - (int16_t) analogX_data;
        rightMotorEffort = (int16_t) analogY_data + (int16_t) analogX_data;


        if (leftMotorEffort >= 0) {
            testForward();
            volatile int count = 0;
            while (count < 2501)
                count++;
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN1);
        }
        else {
            testStop();
            volatile int count = 0;
            while (count < 2501)
                count++;
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN1);
        }
    }
}



void EUSCIA2_IRQHandler(void)
{

    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
    uint8_t byte = UCA2RXBUF;// MAP_UART_receiveData(EUSCI_A2_BASE);

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
            // analogX_packet1 = byte;
        }
    }

    //MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);
    EUSCI_A2->IFG &= 0xFFFE;

}

void T32_INT1_IRQHandler(void)
{
    Timer32_clearInterruptFlag(TIMER32_0_BASE);
}
void T32_INT2_IRQHandler(void)
{
    Timer32_clearInterruptFlag(TIMER32_1_BASE);
}
