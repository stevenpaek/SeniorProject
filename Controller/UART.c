#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/inc/msp432p401r_classic.h>
#include <UART.h>

// assume 48 MHz
const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
    13,//3,                                      // BRDIV = 78
    0,//4,                                       // UCxBRF = 2
    0,                                       // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,                  // No Parity
    EUSCI_A_UART_LSB_FIRST,                  // MSB First
    EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
    EUSCI_A_UART_MODE,                       // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

void UART_init(void)
{
    /* Initialize TX and RX for UART */

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
        GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    MAP_UART_enableModule(EUSCI_A2_BASE);

}
