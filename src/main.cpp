#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <Adafruit_ADS1X15.h>
#include "UTFT.h"

#include "Pump.h"

void parse_message(const String &str);
void pump_start_handler();
void pump_stop_handler();
void set_pump_rotation_speed_handler(const String &str);
void set_pump_rotate_direction(const String &str);

Pump pump;

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
	pump.start();
}

void pump_stop_handler()
{
	Serial.println("INFO: The pump is stopped!");
	pump.stop();
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

	float pump_rmp = atof(float_str);
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
	UTFT myGLCD(TFT01_24SP, dispMISO,
				dispSCK, dispCS,
				dispRST, dispDC);

	myGLCD.InitLCD();
	myGLCD.clrScr();
	myGLCD.setFont(BigFont);
	myGLCD.setColor(VGA_GRAY);

	Serial.println("LCD initialized!");

	for (;;)
	{
		static bool is_text_visible = true;
		if (is_text_visible)
		{
			myGLCD.print("АБОБА", 70, 50);
			is_text_visible = false;
		}
		else
		{
			is_text_visible = true;
			// myGLCD.clrScr();
			myGLCD.print("абеба", 70, 50);
		}

		vTaskDelay(31);
	}
}

void task_pump_control(void *params)
{
	for (;;)
	{
		pump.process();
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