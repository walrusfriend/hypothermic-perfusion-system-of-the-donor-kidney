#pragma once
#include <Arduino.h>
#include "ModbusRtu.h"

enum PumpStates
{
    ON,
    OFF
};

enum RotateDirections
{
    COUNTERCLOCKWISE,
    CLOCKWISE
};

class Pump
{
public:
    Pump();
    ~Pump();

    void start();
    void stop();
    void set_speed(const float &rmp);
    void set_rotate_direction(const RotateDirections &direction);

    void process();

public:
    bool is_new_modbus_message_ready = false;

private:
    PumpStates pump_state = PumpStates::OFF;
    float pump_rmp = 0;
    RotateDirections pump_rotate_direction = CLOCKWISE;

    // data array for modbus network sharing
    uint16_t au16data[10];
    uint8_t u8state;

    /** TODO: Add a separate modbus telegrams for start/stop, rotate
     * direction and a motor speed */
    /**
     * This is an structe which contains a query to an slave device
     */
    // modbus_t telegram;
    modbus_t state_tg;
    modbus_t rotate_direction_tg;
    modbus_t speed_tg;
};