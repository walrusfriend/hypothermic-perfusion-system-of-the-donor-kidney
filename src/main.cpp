#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <Adafruit_ADS1X15.h>
#include "UTFT.h"
#include "ModbusRtu.h"

// подключаем шрифты
extern uint8_t SmallFont[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];

String output_message = "Working...";

void parse_message(const String &str);
void pump_start_handler();
void pump_stop_handler();
void set_pump_rotation_speed_handler(const String &str);
void set_pump_rotate_direction(const String &str);

enum PumpStates
{
	ON,
	OFF
};

PumpStates pump_state = PumpStates::OFF;

enum RotateDirections
{
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

/** TODO: Add a separate modbus telegrams for start/stop, rotate
 * direction and a motor speed */
/**
 * This is an structe which contains a query to an slave device
 */
modbus_t telegram;

bool is_new_modbus_message_ready = false;

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

void task_pressure_sensor_read(void *params);
void task_draw_display(void *params);
void task_pump_control(void *params);
void task_CLI(void *params);

void setup()
{
	Serial.begin(115200);

	// Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
	// because it is sharing a resource, such as the Serial port.
	// Semaphores should only be used whilst the scheduler is running, but we can set it up here.
	if (xSerialSemaphore == NULL) // Check to confirm that the Serial Semaphore has not already been created.
	{
		xSerialSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage the Serial Port
		if ((xSerialSemaphore) != NULL)
			xSemaphoreGive((xSerialSemaphore)); // Make the Serial Port available for use, by "Giving" the Semaphore.
	}

	xTaskCreate(task_pressure_sensor_read, "PressureRead", 128, NULL, 2, NULL);
	xTaskCreate(task_draw_display, "DrawDisplay", 256, NULL, 2, NULL);
	xTaskCreate(task_pump_control, "PumpControl", 256, NULL, 2, NULL);
	xTaskCreate(task_CLI, "CLI", 256, NULL, 2, NULL);
}

void loop() {}

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
	else if (str.startsWith("set_rotate_direction"))
	{
		set_pump_rotate_direction(str);
	}
	else
	{
		Serial.println("ERROR: Unknown command!");
	}
}

void pump_start_handler()
{
	Serial.println("INFO: The pump is started!");

	telegram.u8id = 1;					   // slave address
	telegram.u8fct = MB_FC_WRITE_REGISTER; // function code (this one is registers read)
	telegram.u16RegAdd = 1000;			   // start address in slave
	telegram.u16CoilsNo = 1;			   // number of elements (coils or registers) to read

	au16data[0] = 1;
	pump_state = PumpStates::ON;

	is_new_modbus_message_ready = true;
}

void pump_stop_handler()
{
	Serial.println("INFO: The pump is stopped!");

	telegram.u8id = 1;					   // slave address
	telegram.u8fct = MB_FC_WRITE_REGISTER; // function code (this one is registers read)
	telegram.u16RegAdd = 1000;			   // start address in slave
	telegram.u16CoilsNo = 1;			   // number of elements (coils or registers) to read

	au16data[0] = 0;
	pump_state = PumpStates::OFF;

	is_new_modbus_message_ready = true;
}

void set_pump_rotation_speed_handler(const String &str)
{
	char *strtok_index;
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

	telegram.u8id = 1;								 // slave address
	telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS; // function code (this one is registers read)
	telegram.u16RegAdd = 1002;						 // start address in slave
	telegram.u16CoilsNo = 2;						 // number of elements (coils or registers) to read

	uint8_t *p_float = (void *)(&pump_rmp);
	au16data[0] = p_float[2] | (p_float[3] << 8);
	au16data[1] = p_float[0] | (p_float[1] << 8);

	is_new_modbus_message_ready = true;
}

void set_pump_rotate_direction(const String &str)
{
	char *strtok_index;
	char buff[64];

	strtok_index = strtok(str.c_str(), " ");
	strcpy(buff, strtok_index);
	Serial.print(buff);

	strtok_index = strtok(NULL, " ");
	strcpy(buff, strtok_index);
	Serial.print(buff);

	telegram.u8id = 1;					   // slave address
	telegram.u8fct = MB_FC_WRITE_REGISTER; // function code (this one is registers read)
	telegram.u16RegAdd = 1001;			   // start address in slave
	telegram.u16CoilsNo = 1;			   // number of elements (coils or registers) to read

	if (buff[0] == '0')
	{
		Serial.println("Set the rotate direction to clockwise");
		pump_rotate_direction = RotateDirections::CLOCKWISE;
		au16data[0] = 1;
	}
	else
	{
		Serial.println("Set the rotate direction to counterclockwise");
		pump_rotate_direction = RotateDirections::COUNTERCLOCKWISE;
		au16data[0] = 0;
	}

	is_new_modbus_message_ready = true;
}

void task_pressure_sensor_read(void *params)
{
	Adafruit_ADS1115 ads;

	ads.setGain(GAIN_SIXTEEN);
	if (!ads.begin())
	{
		Serial.println("Failed to initialize ADS.");
		while (1)
			;
	}

	// Start continuous conversions.
	ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);

	for (;;) // A Task shall never return or exit.
	{
		if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
		{
			int16_t results = ads.getLastConversionResults();

			Serial.print("Differential: ");
			Serial.print(results);
			Serial.print("(");
			Serial.print(ads.computeVolts(results));
			Serial.println("mV)");

			xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
		}

		vTaskDelay(60); // one tick delay (16ms) in between reads for stability
	}
}

void task_draw_display(void *params)
{
	const uint16_t dispMISO = 8;
	const uint16_t dispSCK = 7;
	const uint16_t dispCS = 6;
	const uint16_t dispRST = 5;
	const uint16_t dispDC = 4;

	// объявляем объект myGLCD класса библиотеки UTFT указывая тип дисплея
	UTFT myGLCD(TFT01_24SP, dispMISO,
				dispSCK, dispCS,
				dispRST, dispDC);

	myGLCD.InitLCD();
	myGLCD.clrScr();
	myGLCD.setFont(BigFont);
	myGLCD.setColor(VGA_GRAY);

	for (;;)
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

		vTaskDelay(31);
	}
}

void task_pump_control(void *params)
{
	telegram.u8id = 1;			 // slave address
	telegram.u8fct = 6;			 // function code (this one is registers read)
	telegram.u16RegAdd = 1000;	 // start address in slave
	telegram.u16CoilsNo = 1;	 // number of elements (coils or registers) to read
	telegram.au16reg = au16data; // pointer to a memory array in the Arduino

	Serial3.begin(9600, SERIAL_8E1);
	master.start();
	master.setTimeOut(2000); // if there is no answer in 2000 ms, roll over
	u8state = 0;

	for (;;)
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

		vTaskDelay(7);
	}
}

void task_CLI(void *params)
{

	for (;;)
	{
		// USB input data
		if (Serial.available() >= 1)
		{
			if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
			{
				uint8_t buff[128];
				uint16_t size = Serial.readBytesUntil('\n', buff, Serial.available());

				String message;

				for (uint16_t i = 0; i < size; ++i)
				{
					message.concat((char)buff[i]);
				}

				Serial.print("ECHO: ");
				Serial.print(message);

				parse_message(message);
				xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
			}
		}

		vTaskDelay(1);
	}
}