#ifndef ADC_DAC_H_
#define ADC_DAC_H_

void adcdac_setup();
uint16_t adcReadRaw(uint8_t ch);
void dacSetCode(uint8_t ch, uint16_t code);

#endif
