#ifndef config_h
#define config_h

#include <Arduino_FreeRTOS.h>

namespace Pin {
    const uint8_t regime1 = 22;
    const uint8_t regime2 = 23;
    const uint8_t calibration = 24;
    const uint8_t block = 25;
    const uint8_t kidney = 26;
};

namespace Pressure {

};

enum Regime
{
	STOPED,
	REGIME1,
	REGIME2
};

enum KidneyState
{
	LEFT_KIDNEY,
	RIGTH_KIDNEY
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

#endif