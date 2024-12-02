#ifndef PRESSURE_h
#define PRESSURE_h

#include <Arduino.h>

class Pressure {
public:
    Pressure();

    void set_tare(const float& tare);
    const float& get_tare();

    void set_target(const float& target);
    const float& get_target();

    void set_value(const float& value);
    const float& get_value();

    const float& get_low_limit();
    const float& get_optimal_high_limit();
    const float& get_high_limit();

private:
	float target_value;
	float low_limit;
	float optimal_high_limit;
	float high_limit;
	float tare_value;
	float current_value;
};

#endif