#ifndef config_h
#define config_h

#include <Arduino_FreeRTOS.h>

namespace Pin {
    // const uint8_t regime1 = 30;					// yellow  	- block
    // const uint8_t regime2 = 44; 				// green	- regime 2
    // const uint8_t calibration = 40;				// white	- kidney
    // const uint8_t block = 30;					// black	- calibration
    // const uint8_t kidney = 25;					// blue		- regime 1

    const uint8_t regime1 = 30;					// yellow  	- block
    const uint8_t regime2 = 44; 				// green	- regime 2
    const uint8_t calibration = 25;				// white	- kidney
    const uint8_t block = 40;					// black	- calibration
    const uint8_t kidney = 49;					// blue		- regime 1
    const uint8_t temperature1_pin = 12;
	const uint8_t temperature2_pin = 13;
    const uint8_t emulator_button_pin = 35;
    const uint8_t MOSFET_pin = 36;
};

enum Regime
{
	STOPED,
	REGIME1,
	REGIME2,
    REGIME_REMOVE_BUBBLE,
	BLOCKED
};

enum KidneyState
{
	LEFT_KIDNEY,
	RIGHT_KIDNEY
};

enum AlertType
{
	NONE,
	PRESSURE_LOW,
	PRESSURE_HIGH,
	PRESSURE_UP,
	TEMP1_LOW,
	TEMP1_HIGH,
	TEMP2_LOW,
	TEMP2_HIGH,
	RESISTANCE
};

/** TODO: Нужно вспомнить, какую максимальную скорость мы можем поставить */
const float PUMP_MAX_SPEED = 100;

#endif