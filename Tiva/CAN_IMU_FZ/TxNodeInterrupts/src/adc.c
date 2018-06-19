#include "adc.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"



// ADC setup function
void ADCenable(void) {
  // Enable ADC0 on pin PE3
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                           ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 3);
  ADCIntClear(ADC0_BASE, 3);
}

// ADC read function
uint32_t ADCread(void) {
  uint32_t pui32ADC0Value[1]; // FIFO, can be larger
  ADCProcessorTrigger(ADC0_BASE, 3);
  while(!ADCIntStatus(ADC0_BASE, 3, false)) {;}
  ADCIntClear(ADC0_BASE, 3);
  ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
  return pui32ADC0Value[0];
}
