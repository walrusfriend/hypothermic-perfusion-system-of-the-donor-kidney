#include "pump.h"

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI
 *               or any pin number > 1 for RS-485
 */
Modbus master(0, Serial3, 0); // this is master and RS-232 or USB-FTDI

Pump::Pump()
{
    telegram.u8id = 1;           // slave address
    telegram.u8fct = 6;          // function code (this one is registers read)
    telegram.u16RegAdd = 1000;   // start address in slave
    telegram.u16CoilsNo = 1;     // number of elements (coils or registers) to read
    telegram.au16reg = au16data; // pointer to a memory array in the Arduino

    // state_tg.u8id = 1;           // slave address
    // state_tg.u8fct = 6;          // function code (this one is registers read)
    // state_tg.u16RegAdd = 1000;   // start address in slave
    // state_tg.u16CoilsNo = 1;     // number of elements (coils or registers) to read
    // state_tg.au16reg = au16data; // pointer to a memory array in the Arduino

    // speed_tg.u8id = 1;                               // slave address
    // speed_tg.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS; // function code (this one is registers read)
    // speed_tg.u16RegAdd = 1002;                       // start address in slave
    // speed_tg.u16CoilsNo = 2;                         // number of elements (coils or registers) to read
    // speed_tg.au16reg = au16data; // pointer to a memory array in the Arduino

    // rotate_direction_tg.u8id = 1;                     // slave address
    // rotate_direction_tg.u8fct = MB_FC_WRITE_REGISTER; // function code (this one is registers read)
    // rotate_direction_tg.u16RegAdd = 1001;             // start address in slave
    // rotate_direction_tg.u16CoilsNo = 1;               // number of elements (coils or registers) to read
    // rotate_direction_tg.au16reg = au16data; // pointer to a memory array in the Arduino

    Serial3.begin(9600, SERIAL_8E1);
    master.start();
    master.setTimeOut(2000); // if there is no answer in 2000 ms, roll over
    u8state = 0;
}

Pump::~Pump()
{
}

void Pump::start()
{
    telegram.u8id = 1;                     // slave address
    telegram.u8fct = MB_FC_WRITE_REGISTER; // function code (this one is registers read)
    telegram.u16RegAdd = 1000;             // start address in slave
    telegram.u16CoilsNo = 1;               // number of elements (coils or registers) to read

    au16data[0] = 1;
    pump_state = PumpStates::ON;

    is_new_modbus_message_ready = true;
    // master.query(state_tg);
}

void Pump::stop()
{
    telegram.u8id = 1;                     // slave address
    telegram.u8fct = MB_FC_WRITE_REGISTER; // function code (this one is registers read)
    telegram.u16RegAdd = 1000;             // start address in slave
    telegram.u16CoilsNo = 1;               // number of elements (coils or registers) to read

    au16data[0] = 0;
    pump_state = PumpStates::OFF;

    is_new_modbus_message_ready = true;
    // master.query(state_tg);
}

void Pump::set_speed(const float &rmp)
{
    pump_rmp = rmp;
    
    telegram.u8id = 1;                               // slave address
    telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS; // function code (this one is registers read)
    telegram.u16RegAdd = 1002;                       // start address in slave
    telegram.u16CoilsNo = 2;                         // number of elements (coils or registers) to read

    uint8_t *p_float = (void *)(&pump_rmp);
    au16data[0] = p_float[2] | (p_float[3] << 8);
    au16data[1] = p_float[0] | (p_float[1] << 8);

    is_new_modbus_message_ready = true;
    // master.query(speed_tg);
}

void Pump::set_rotate_direction(const RotateDirections &direction)
{
    telegram.u8id = 1;                     // slave address
    telegram.u8fct = MB_FC_WRITE_REGISTER; // function code (this one is registers read)
    telegram.u16RegAdd = 1001;             // start address in slave
    telegram.u16CoilsNo = 1;               // number of elements (coils or registers) to read
    au16data[0] = direction;

    pump_rotate_direction = direction;

    is_new_modbus_message_ready = true;
    // master.query(rotate_direction_tg);
}

void Pump::process()
{
    switch (u8state)
    {
    case 0:
        if (is_new_modbus_message_ready)
        {
            master.query(telegram);
            is_new_modbus_message_ready = false;
        }
        u8state++;
        break;
    case 1:
        master.poll(); // check incoming messages
        u8state = 0;
        break;
    }
}