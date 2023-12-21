#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <Adafruit_ADS1X15.h>
#include "UTFT.h"

#include "Pump.h"

#include "GyverPID.h"

void parse_message(const String &str);
void pump_start_handler();
void pump_stop_handler();
void set_pump_rotation_speed_handler(const String &str);
void set_pump_rotate_direction(const String &str);
void set_P(const String &str);
void set_I(const String &str);
void set_D(const String &str);

void PIDor(const float &value);

GyverPID pid(1, 1, 1, 960);

Pump pump;

bool is_data_transmitted = false;

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
	else if (str.startsWith("set_P")) {
		set_P(str);
	}
	else if (str.startsWith("set_I")) {
		set_I(str);
	}
	else if (str.startsWith("set_D")) {
		set_D(str);
	}
	else
	{
		Serial.println("ERROR: Unknown command!");
	}
}

void pump_start_handler()
{
	Serial.println("INFO: The pump is started!");
	pump.start();
}

void pump_stop_handler()
{
	Serial.println("INFO: The pump is stopped!");
	pump.stop();
}

void set_pump_rotation_speed_handler(const String &str)
{
	int space_idx = str.indexOf(' ');

	if (space_idx > str.length())
	{
		Serial.println("ERROR: Space index are more than string size!");
		return;
	}

	int LF_idx = str.indexOf('\n');

	if (LF_idx > str.length())
	{
		// Serial.println("ERROR: LF index are more than string size!");
		LF_idx = str.length() - 1;
		// return;
	}

	if (space_idx >= LF_idx)
	{
		Serial.println("ERROR: space index more than LF index!");
		return;
	}

	String float_str = str.substring(space_idx + 1, LF_idx);

	float pump_rmp = float_str.toFloat();

	Serial.print("DEBUG: Set speed value to ");
	Serial.println(pump_rmp);

	pump.set_speed(pump_rmp);
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

	Serial.println("Set the rotate direction to clockwise");
	pump.set_rotate_direction((buff[0] == '0') ? RotateDirections::COUNTERCLOCKWISE : RotateDirections::CLOCKWISE);
}

void set_P(const String &str) {
	int space_idx = str.indexOf(' ');

	String P = str.substring(space_idx + 1, str.length() - 1);
	pid.Kp = P.toFloat();

	Serial.print("Set kP value to ");
	Serial.println(pid.Kp);
}

void set_I(const String &str) {
	int space_idx = str.indexOf(' ');

	String I = str.substring(space_idx + 1, str.length() - 1);
	pid.Ki = I.toFloat();

	Serial.print("Set ki value to ");
	Serial.println(pid.Ki);
}

void set_D(const String &str) {
	int space_idx = str.indexOf(' ');

	String D = str.substring(space_idx + 1, str.length() - 1);
	pid.Kd = D.toFloat();

	Serial.print("Set kd value to ");
	Serial.println(pid.Kd);
}

void PIDor(const float &value)
{
	pid.input = value;

	pump.set_speed(pid.getResultTimer());
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
	ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);

	float k = 0.1;

	float fill_value = 0;
	char pressure_sensor_output_buffer[32];

	pid.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
	pid.setLimits(1, 100);	// пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
	pid.setpoint = 29;

	for (;;) // A Task shall never return or exit.
	{
		// if (counter > 9) {
		// if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
		// {
		if (is_data_transmitted == false)
		{
			int16_t raw_data = ads.getLastConversionResults();
			float converted_value = raw_data * 7.8125 / 25;

			// if (abs(converted_value) < abs(fill_value) * 0.9) {
			// 	converted_value = fill_value;
			// }

			fill_value += (converted_value - fill_value) * k;

			// sprintf(pressure_sensor_output_buffer, "%d %d\n", converted_value, fill_value);
			// Serial.print(pressure_sensor_output_buffer);
			// Serial.print(converted_value);
			// Serial.print(' ');
			Serial.println(fill_value);

			PIDor(fill_value);
			// Serial.print(converted_value);
			// Serial.print(' ');
			// Serial.println(fill_value);
		}
		// Serial.print("Differential: ");
		// Serial.println(converted_value / counter);
		// Serial.print("(");
		// Serial.print(ads.computeVolts(results));
		// Serial.println("mV)");

		// 	xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
		// }

		vTaskDelay(60); // one tick delay (16ms) in between reads for stability
	}
}

void task_draw_display(void *params)
{
	// подключаем шрифты
	extern uint8_t SmallFont[];
	extern uint8_t BigFont[];
	extern uint8_t SevenSegNumFont[];
	extern uint8_t BigFontRus[];

	const uint16_t dispMISO = 8;
	const uint16_t dispSCK = 7;
	const uint16_t dispCS = 6;
	const uint16_t dispRST = 5;
	const uint16_t dispDC = 4;

	// объявляем объект myGLCD класса библиотеки UTFT указывая тип дисплея
	// UTFT myGLCD(TFT01_24SP, dispMISO,
	// 			dispSCK, dispCS,
	// 			dispRST, dispDC);

	// myGLCD.InitLCD();
	// myGLCD.clrScr();
	// myGLCD.setFont(BigFont);
	// myGLCD.setColor(VGA_GRAY);

	Serial.println("LCD initialized!");

	for (;;)
	{
		// static bool is_text_visible = true;
		// if (is_text_visible)
		// {
		// 	myGLCD.print("abeba", 70, 50);
		// 	is_text_visible = false;
		// }
		// else
		// {
		// 	is_text_visible = true;
		// 	// myGLCD.clrScr();
		// 	myGLCD.print("aboba", 70, 50);
		// }

		vTaskDelay(31);
	}
}

void task_pump_control(void *params)
{
	for (;;)
	{
		pump.process();
		vTaskDelay(1);
	}
}

void task_CLI(void *params)
{

	for (;;)
	{
		// USB input data
		if (Serial.available() >= 1)
		{
			// if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
			// {
			is_data_transmitted = true;

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

			is_data_transmitted = false;
			// 	xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
			// }
		}

		// if (Serial3.available() >= 1) {
		// 	uint8_t buff[128];
		// 	uint16_t size = Serial3.available();
		// 	Serial3.readBytes(buff, size);
		// 	Serial.write(buff, size);
		// }

		vTaskDelay(5);
	}
}