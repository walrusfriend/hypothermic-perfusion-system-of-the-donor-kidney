#ifndef config_h
#define config_h

#include <Arduino_FreeRTOS.h>

namespace Pin {
    const uint8_t regime1 = 22;
    const uint8_t regime2 = 23;
    const uint8_t calibration = 24;
    const uint8_t block = 25;
    const uint8_t kidney = 26;
    const uint8_t temperature1_pin = 12;
	const uint8_t temperature2_pin = 13;
    const uint8_t emulator_button_pin = 0xff;
    const uint8_t MOSFET_pin = 0xff;
};

namespace Pressure {

};

enum Regime
{
	STOPED,
	REGIME1,
	REGIME2,
    REGIME_REMOVE_KEBAB
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

/** TODO: Нужно вспомнить, какую максимальную скорость мы можем поставить */
const float PUMP_MAX_SPEED = 100;

#endif