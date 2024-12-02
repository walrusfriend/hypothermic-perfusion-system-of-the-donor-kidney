#include "BaseParams/Pressure.h"

Pressure::Pressure() 
    : target_value(29)
    , low_limit(target_value - 1)
    , optimal_high_limit(target_value + 1)
    , high_limit(target_value + 10)
    , tare_value(0)
    , current_value(1)
{
}

void Pressure::set_tare(const float &tare)
{
    tare_value = tare;
}

const float &Pressure::get_tare()
{
    return tare_value;
}

void Pressure::set_target(const float &target)
{
    target_value = target;
    low_limit = target - 1;
    optimal_high_limit = target + 1;
	high_limit = target + 10;
}

const float &Pressure::get_target()
{
    return target_value;
}

void Pressure::set_value(const float &value)
{
    current_value = value;
}

const float &Pressure::get_value()
{
    return current_value;
}

const float& Pressure::get_low_limit() {
    return low_limit;
}

const float& Pressure::get_optimal_high_limit() {
    return optimal_high_limit;
}

const float& Pressure::get_high_limit() {
    return high_limit;
}
