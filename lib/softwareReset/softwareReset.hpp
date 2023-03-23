#pragma once
#include <Arduino.h>


#include <avr/wdt.h>

namespace softwareReset
{
	inline void simple()
	{
		asm volatile (" jmp 0");
	}

	inline void standard()
	{
		while(true)
		{

			wdt_enable(WDTO_15MS);
			for(;;) {};

		}
	}

	// disable software reset after successful reset
	void disable() __attribute__((naked)) __attribute__((section(".init3"))) ;
	void disable()
	{
		MCUSR = 0 ;
		wdt_disable() ;
	}
}