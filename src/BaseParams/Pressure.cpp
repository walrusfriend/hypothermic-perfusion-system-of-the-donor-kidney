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

void Pressure::set_tare(const int &tare)
{
    tare_value = tare;
}

inline const int &Pressure::get_tare()
{
    return tare_value;
}

void Pressure::set_target(const int &target)
{
    target_value = target;
}

inline const int &Pressure::get_target()
{
    return target_value;
}

void Pressure::set_value(const int &value)
{
    current_value = value;
}

inline const int &Pressure::get_value()
{
    return current_value;
}
