/*
 * main.cpp
 *
 *  Created on: Jul 25, 2020
 *      Author: xenir
 */

#include "si5351.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_i2c.h"

Si5351 synth;

#ifdef __cplusplus
 extern "C" {
#endif

void setupSynth( I2C_HandleTypeDef* hi2c )
{
	  synth.init( hi2c, SI5351_CRYSTAL_LOAD_8PF, 25000000, 0 );

	  int currentFrequency = 7200000;
	  int mult = 0;

	  if ( currentFrequency < 8000000 )
		  mult = 100;
	  else if ( currentFrequency < 11000000 )
		  mult = 80;
	  else if ( currentFrequency < 15000000 )
		  mult = 50;

	  uint64_t freq = currentFrequency * 100ULL;
	  uint64_t pllFreq = freq * mult;

	  synth.set_freq_manual(freq, pllFreq, SI5351_CLK0);
	  synth.set_freq_manual(freq, pllFreq, SI5351_CLK2);

	  synth.set_phase(SI5351_CLK0, 0);
	  synth.set_phase(SI5351_CLK2, mult);
	  synth.pll_reset(SI5351_PLLA);

}

#ifdef __cplusplus
}
#endif

