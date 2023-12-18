#include <Arduino.h>

#include "UTFT.h"
#include "ModbusRtu.h"

#define dispMISO 8
#define dispSCK 7
#define dispCS 6
#define dispRST 5
#define dispDC 4

/** TODO: Add start/stop state monitoring (add enum and variables) */

// подключаем шрифты
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];

String output_message = "Working...";

void display_task();
void CRCVerify(char *rec, char CRClen, char CRCdata[2]);

void parse_message(const String &str);
void pump_start_handler();
void pump_stop_handler();
void set_pump_rotation_speed_handler(const String &str);
void get_pump_rotation_speed_handler();
void set_pump_rotate_direction(const String &str);
void get_pump_rotate_direction();

void send_wrong_data_to_pump();

// объявляем объект myGLCD класса библиотеки UTFT указывая тип дисплея
UTFT myGLCD(TFT01_24SP, dispMISO,
			dispSCK, dispCS,
			dispRST, dispDC);

enum RotateDirections {
	CLOCKWISE,
	COUNTERCLOCKWISE
};

float pump_rmp = 0;
RotateDirections pump_rotate_direction = CLOCKWISE;

// data array for modbus network sharing
uint16_t au16data[10];
uint8_t u8state;

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI
 *               or any pin number > 1 for RS-485
 */
Modbus master(0, Serial3, 0); // this is master and RS-232 or USB-FTDI

/**
 * This is an structe which contains a query to an slave device
 */
modbus_t telegram;

bool is_new_modbus_message_ready = false;

unsigned long u32wait;

void setup()
{
	myGLCD.InitLCD();
	myGLCD.clrScr();
	myGLCD.setFont(BigFont);
	myGLCD.setColor(VGA_GRAY);

	// PC <-> Arduino Mega communication
	Serial.begin(115200);

	telegram.u8id = 1;			 // slave address
	telegram.u8fct = 6;			 // function code (this one is registers read)
	telegram.u16RegAdd = 1000;	 // start address in slave
	telegram.u16CoilsNo = 1;	 // number of elements (coils or registers) to read
	telegram.au16reg = au16data; // pointer to a memory array in the Arduino

	Serial3.begin(9600, SERIAL_8E1);
	master.start();
	master.setTimeOut(2000); // if there is no answer in 2000 ms, roll over
	u32wait = millis() + 1000;
	u8state = 0;
}

void loop()
{
	// USB input data
	if (Serial.available() >= 1)
	{
		uint8_t buff[512];
		uint16_t size = Serial.readBytesUntil('\n', buff, Serial.available());

		String message;

		for (uint16_t i = 0; i < size; ++i)
		{
			message.concat((char)buff[i]);
		}

		Serial.print("ECHO: ");
		Serial.print(message);

		parse_message(message);
	}

	// Pump reply
	if (Serial3.available() >= 1)
	{
		Serial.print(Serial3.read());
	}

	switch (u8state)
	{
	case 0:
		if (millis() > u32wait)
			u8state++; // wait state
		break;
	case 1:
		if (is_new_modbus_message_ready) {
			master.query(telegram);
			is_new_modbus_message_ready = false;
		}

		u8state++;
		break;
	case 2:
		master.poll(); // check incoming messages
		if (master.getState() == COM_IDLE)
		{
			u8state = 0;
			u32wait = millis() + 100;
		}
		break;
	}

	// display_task();
	delay(10);
}

// Code from MODBUS Communicatio Protocol description file
void CRCVerify(char *rec, char CRClen, char CRCdata[2])
{
	char i1, j;
	unsigned int crc_data = 0xffff;
	for (i1 = 0; i1 < CRClen; i1++)
	{
		crc_data = crc_data ^ rec[i1];
		for (j = 0; j < 8; j++)
		{
			if (crc_data & 0x0001)
			{
				crc_data >>= 1;
				crc_data ^= 0xA001;
			}
			else
			{
				crc_data >>= 1;
			}
		}
	}
	CRCdata[0] = (char)(crc_data);
	CRCdata[1] = (char)(crc_data >> 8);
}

void display_task()
{
	static bool is_text_visible = true;
	if (is_text_visible)
	{
		myGLCD.print("ABEBA", 70, 50);
		is_text_visible = false;
	}
	else
	{
		is_text_visible = true;
		// myGLCD.clrScr();
		myGLCD.print("ABOBA", 70, 50);
	}
}

void parse_message(const String &str)
{
	if (str.startsWith("start"))
	{
		pump_start_handler();
	}
	else if (str.startsWith("stop"))
	{
		pump_stop_handler();
	}
	else if (str.startsWith("set_speed"))
	{
		set_pump_rotation_speed_handler(str);
	}
	else if (str.startsWith("get_speed"))
	{
		get_pump_rotation_speed_handler();
	}
	else if (str.startsWith("wrong"))
	{
		send_wrong_data_to_pump();
	}
	else if (str.startsWith("set_rotate_direction")) {
		set_pump_rotate_direction(str);
	}
	else if (str.startsWith("get_rotate_direction")) {
		get_pump_rotate_direction();
	}
	else
	{
		Serial.println("ERROR: Unknown command!");
	}
}

void pump_start_handler()
{
	Serial.println("INFO: The pump is started!");

	telegram.u8id = 1;			 			// slave address
	telegram.u8fct = MB_FC_WRITE_REGISTER;	// function code (this one is registers read)
	telegram.u16RegAdd = 1000;	 			// start address in slave
	telegram.u16CoilsNo = 1;	 			// number of elements (coils or registers) to read
	
	au16data[0] = 1;

	is_new_modbus_message_ready = true;
}

void pump_stop_handler()
{
	Serial.println("INFO: The pump is stopped!");

	telegram.u8id = 1;			 			// slave address
	telegram.u8fct = MB_FC_WRITE_REGISTER;	// function code (this one is registers read)
	telegram.u16RegAdd = 1000;	 			// start address in slave
	telegram.u16CoilsNo = 1;	 			// number of elements (coils or registers) to read
	
	au16data[0] = 0;

	is_new_modbus_message_ready = true;
}

void set_pump_rotation_speed_handler(const String &str)
{
	char* strtok_index;
	char buff[64];

	strtok_index = strtok(str.c_str(), " ");
	strcpy(buff, strtok_index);
	Serial.print(buff);

	strtok_index = strtok(NULL, " ");
	strcpy(buff, strtok_index);
	Serial.print(buff);

	char float_str[10];

	strtok_index = strtok(buff, "\n");
	strcpy(float_str, strtok_index);
	Serial.print(float_str);

	pump_rmp = atof(float_str);
	Serial.println(pump_rmp);


	telegram.u8id = 1;			 						// slave address
	telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS;	// function code (this one is registers read)
	telegram.u16RegAdd = 1002;							// start address in slave
	telegram.u16CoilsNo = 2;	 						// number of elements (coils or registers) to read

	uint8_t* p_float = (void*)(&pump_rmp);
	au16data[0] = p_float[2] | (p_float[3] << 8);
	au16data[1] = p_float[0] | (p_float[1] << 8);


	is_new_modbus_message_ready = true;
}

void get_pump_rotation_speed_handler()
{
	Serial.println(pump_rmp);
}

void set_pump_rotate_direction(const String& str) {
	char* strtok_index;
	char buff[64];

	strtok_index = strtok(str.c_str(), " ");
	strcpy(buff, strtok_index);
	Serial.print(buff);

	strtok_index = strtok(NULL, " ");
	strcpy(buff, strtok_index);
	Serial.print(buff);

	telegram.u8id = 1;			 			// slave address
	telegram.u8fct = MB_FC_WRITE_REGISTER;	// function code (this one is registers read)
	telegram.u16RegAdd = 1001;				// start address in slave
	telegram.u16CoilsNo = 1;	 			// number of elements (coils or registers) to read	
	
	if (buff[0] == '0') {
		Serial.println("Set the rotate direction to clockwise");
		pump_rotate_direction = RotateDirections::CLOCKWISE;
		au16data[0] = 1;
	}
	else {
		Serial.println("Set the rotate direction to counterclockwise");
		pump_rotate_direction = RotateDirections::COUNTERCLOCKWISE;
		au16data[0] = 0;
	}

	is_new_modbus_message_ready = true;
}

void get_pump_rotate_direction() {
	Serial.println("INFO: The rotate direction is ");
	Serial.print((pump_rotate_direction == RotateDirections::CLOCKWISE) ? "clockwise" : "counterclockwise");
}

void send_wrong_data_to_pump()
{
	// Reply from pump - 1145313145
	uint8_t wrong_sequence[8] = {0x01, 0xFF, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
	Serial3.write(wrong_sequence, sizeof(wrong_sequence));
}