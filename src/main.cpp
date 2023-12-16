#include <Arduino.h>

#include "UTFT.h"
#include "ModbusRtu.h"

#define dispMISO 8
#define dispSCK 7
#define dispCS 6
#define dispRST 5
#define dispDC 4

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

// объявляем объект myGLCD класса библиотеки UTFT указывая тип дисплея
UTFT myGLCD(TFT01_24SP, dispMISO,
			dispSCK, dispCS,
			dispRST, dispDC);

float pump_rmp = 0;

// data array for modbus network sharing
uint16_t au16data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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

void setup()
{
	myGLCD.InitLCD();
	myGLCD.clrScr();
	myGLCD.setFont(BigFont);
	myGLCD.setColor(VGA_GRAY);

	// PC <-> Arduino Mega communication
	Serial.begin(115200);

	// RS-232
	// Serial3.begin(9600);
	telegram.u8id = 1;			 // slave address
	telegram.u8fct = 6;			 // function code (this one is registers read)
	telegram.u16RegAdd = 1000;	 // start address in slave
	telegram.u16CoilsNo = 1;	 // number of elements (coils or registers) to read
	telegram.au16reg = au16data; // pointer to a memory array in the Arduino

	Serial.begin(19200); 
	master.start();
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
	else
	{
		Serial.println("ERROR: Unknown command!");
	}
}

void pump_start_handler()
{
	Serial.println("INFO: The pump is started!");

	// uint8_t start_sequence[8] = {0x01, 0x06, 0x03, 0xE8, 0x00, 0x01, 0xC8, 0x7A};
	// Serial3.write(start_sequence, sizeof(start_sequence));
	au16data[0] = 1;
	master.query(telegram);
}

void pump_stop_handler()
{
	Serial.println("INFO: The pump is stopped!");

	// uint8_t stop_sequence[8] = {0x01, 0x06, 0x03, 0xE8, 0x00, 0x00, 0x09, 0xBA};
	// Serial3.write(stop_sequence, sizeof(stop_sequence));
	au16data[0] = 0;
	master.query(telegram);
}

void set_pump_rotation_speed_handler(const String &str)
{
}

void get_pump_rotation_speed_handler()
{
	Serial.println(pump_rmp);
}