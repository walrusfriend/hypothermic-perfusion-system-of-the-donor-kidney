#ifndef PRESSURE_h
#define PRESSURE_h

#include <Arduino.h>

class Pressure {
public:
    Pressure();

    void set_tare(const int& tare);
    inline const int& get_tare();

    void set_target(const int& target);
    inline const int& get_target();

    void set_value(const int& value);
    inline const int& get_value();

private:
	int target_value;
	int low_limit;
	int optimal_high_limit;
	int high_limit;
	int tare_value;
	int current_value;
};

#endif