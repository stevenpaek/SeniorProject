#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/inc/msp432p401r_classic.h>
#include <ADC.h>

void ADC_init(void)
{
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4, 0);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
        GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
        ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A5, false); // Pin 5.0 Vert
    MAP_ADC14_configureConversionMemory(ADC_MEM1,
        ADC_VREFPOS_AVCC_VREFNEG_VSS,
        ADC_INPUT_A4, false); // Pin 5.1 Horz

    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

}
