
#include <avr/io.h>

#include "adc_controller.h"

#define ADC_MUX_MASK 0x0F

/***********************************************************/
/***********************************************************/

CADCController CADCController::m_cADCControllerInstance;

/***********************************************************/
/***********************************************************/

CADCController& CADCController::GetInstance() {
   return m_cADCControllerInstance;
}

/***********************************************************/
/***********************************************************/

uint8_t CADCController::GetValue(EChannel e_channel) {
   /* select the channel to do the conversion */
   uint8_t unADMuxSetting = ADMUX;
   unADMuxSetting &= ~ADC_MUX_MASK;
   unADMuxSetting |= static_cast<uint8_t>(e_channel);
   ADMUX = unADMuxSetting;
   /* Start conversion */
   ADCSRA |= (1 << ADSC);
   /* Wait for the conversion to complete */
   while((ADCSRA & (1 << ADSC)) != 0);
   /* Return the result */
   return ADCH;
}

/***********************************************************/
/***********************************************************/

CADCController::CADCController() {
   /* Initialize the analog to digital converter */
   /* Use the internal 1.1V reference, left align result */
   ADMUX |= ((1 << REFS1) | (1 << REFS0) |
             (1 << ADLAR));
   /* Enable the ADC and set the prescaler to 64, (8MHz / 64 = 125kHz) */
   ADCSRA |= ((1 << ADEN) | (1 << ADPS2) | (1 << ADPS1));
}

