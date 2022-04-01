#ifndef RGB_CYCLE_H_
#define RGB_CYCLE_H_
#include <stdint.h>

// Initializes TimerA1 to output a cycle of the RGB color combinations
void Spectrum_Init(uint16_t duty);

// Sets the duty cycle of the RGB LEDs in the Spectrum
void Set_Spectrum_Duty(uint16_t duty);

// ISR for nonzero CCR interrupts
void TA0_N_IRQHandler(void);


#endif /* RGB_CYCLE_H_ */
