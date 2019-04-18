/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#include "UF_LCD.h"
#include "UART.h"
#include "ADC.h"

volatile  uint16_t resultsBuffer[2];

volatile char button1 = 1;
volatile char button2 = 1;

void clk_init(void);
void GPIO_init(void);

int main(void)
{

    /* Halting the Watchdog  */
    MAP_WDT_A_holdTimer();

    memset(resultsBuffer, 0x00, 2 * sizeof(uint16_t));

    lcd_init();
    ADC_init();
    UART_init();
    clk_init();
    GPIO_init();

    volatile int8_t analogX_upper = 0;
    volatile int8_t analogX_lower = 0;
    volatile int8_t analogY_upper = 0;
    volatile int8_t analogY_lower = 0;

    volatile int8_t analogX_data;
    volatile int8_t analogY_data;

    volatile uint8_t switches_upper = 0;
    volatile uint8_t switches_lower = 0;

    volatile uint8_t buttons = 0;

    // volatile int8_t reconstruct = 0;

    lcdString("Welcome to R2-D2");

    char analogX[5];
    char analogY[5];
    char P7IN_String[5];
    char button1_string[1];
    char button2_string[1];
    char scroll = 0;
    char pos = 0;

    int8_t leftMotorEffort = 0;
    int8_t rightMotorEffort = 0;

    MAP_Interrupt_enableMaster();

    while(1) {
        // convert 123 to string [buf]


        resultsBuffer[0] = MAP_ADC14_getResult(ADC_MEM0); // analogY
        resultsBuffer[1] = MAP_ADC14_getResult(ADC_MEM1); // analogX

        analogY_data = resultsBuffer[0]*200/16383 - 100;
        analogX_data = -resultsBuffer[1]*200/16383 + 100;

        analogX_upper = (int8_t) ((analogX_data >> 4) & 0x0F); // 0b000
        analogX_lower = (int8_t) (analogX_data & 0x0F);
        analogX_lower = (analogX_lower | 0x20); // 0b001

        analogY_upper = (int8_t) ((analogY_data >> 4) & 0x0F);
        analogY_upper = (analogY_upper | 0x40); // 0b010
        analogY_lower = (int8_t) (analogY_data & 0x0F);
        analogY_lower = (analogY_lower | 0x60); // 0b011

        switches_upper = (uint8_t) ((P7IN >> 4) & 0x0F);
        switches_upper = (switches_upper | 0x80); // 0b100
        switches_lower = (uint8_t) (P7IN & 0x0F);
        switches_lower = (switches_lower | 0xA0); // 0b101

        buttons = (uint8_t) ((P10IN >> 4) & 0x03);
        buttons = (buttons | 0xC0); // 0b110

        //analogX_upper = (uint8_t) ((resultsBuffer[1] & 0x3F80) >> 7);
        //analogX_upper = (analogX_upper | 0x80);
        //analogX_lower = (uint8_t) (resultsBuffer[1] & 0x003F);
        //analogX_lower = (analogX_lower & 0x7F);

        //MAP_UART_transmitData(EUSCI_A2_BASE, resultsBuffer[0]*255/16383);
        //MAP_UART_transmitData(EUSCI_A2_BASE, resultsBuffer[1]);

        MAP_UART_transmitData(EUSCI_A2_BASE, analogX_upper);
        MAP_UART_transmitData(EUSCI_A2_BASE, analogX_lower);
        MAP_UART_transmitData(EUSCI_A2_BASE, analogY_upper);
        MAP_UART_transmitData(EUSCI_A2_BASE, analogY_lower);
        MAP_UART_transmitData(EUSCI_A2_BASE, switches_upper);
        MAP_UART_transmitData(EUSCI_A2_BASE, switches_lower);
        MAP_UART_transmitData(EUSCI_A2_BASE, buttons);

        //lcd_command(0x02);

        if (analogX_data <= 20 && analogX_data >= -20 && analogY_data >= 20 && ((buttons & 0x01) == 0x00)) {
            lcd_command(0x02);
            lcdString("Moving Forward  ");
            scroll = 0;
        }
        else if (analogX_data <= 20 && analogX_data >= -20 && analogY_data <= -20 && ((buttons & 0x01) == 0x00)) {
            lcd_command(0x02);
            lcdString("Moving Backward ");
            scroll = 0;
        }
        else if (analogX_data > 20 && analogY_data >= 20 && ((buttons & 0x01) == 0x00)) {
            lcd_command(0x02);
            lcdString("Forward Right   ");
            scroll = 0;
        }
        else if (analogX_data < -20 && analogY_data >= 20 && ((buttons & 0x01) == 0x00)) {
            lcd_command(0x02);
            lcdString("Forward Left    ");
            scroll = 0;
        }
        else if (analogX_data < -20 && analogY_data <= -20 && ((buttons & 0x01) == 0x00)) {
            lcd_command(0x02);
            lcdString("Backward Left   ");
            scroll = 0;
        }
        else if (analogX_data > 20 && analogY_data <= -20 && ((buttons & 0x01) == 0x00)) {
            lcd_command(0x02);
            lcdString("Backward Right  ");
            scroll = 0;
        }
        else {
            if (scroll == 1 && pos <= 16) {
                lcd_command(0x1C);
                pos = pos + 1;
                volatile int delay = 0;
                while (delay < 500000)
                    delay++;
            }
            else {
                lcd_command(0x02);
                lcdString("                 ");
                lcd_command(0x02);
                lcdString("R2-D2");
                scroll = 1;
                pos = 0;
                volatile int delay = 0;
                while (delay < 500000)
                    delay++;
            }
        }

        /*
        lcdString("B: ");

        if ((P10IN & 0x10) == 0x10 && (P10IN & 0x20) == 0x20)
            lcdString("11");
        else if ((P10IN & 0x10) != 0x10 && (P10IN & 0x20) == 0x20)
            lcdString("01");
        else if ((P10IN & 0x10) == 0x10 && (P10IN & 0x20) != 0x20)
            lcdString("10");
        else
            lcdString("00");
        lcdString(" X: ");
        sprintf(analogY, "%d", (int8_t) (((analogY_upper & 0x0F) << 4) + (analogY_lower & 0x0F)));
        sprintf(analogX, "%d", (int8_t) (((analogX_upper & 0x0F) << 4) + (analogX_lower & 0x0F)));
        lcdString(analogX);
        lcdString("         ");
        lcd_command(0xC0);

        lcdString("S: ");
        sprintf(P7IN_String, "%d", (uint16_t) P7IN);
        lcdString(P7IN_String);

        lcdString(" Y: ");
        lcdString(analogY);
        lcdString("        ");
        */


    }

}

void clk_init(void)
{
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    MAP_FPU_enableModule();

    MAP_CS_setDCOFrequency(48000000);

    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
}

void GPIO_init(void)
{
    // GPIO inputs for buttons
    MAP_GPIO_setAsInputPin(GPIO_PORT_P10, GPIO_PIN4);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P10, GPIO_PIN5);

    // GPIO inputs for switches
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN7);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN5);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN4);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0);
}
