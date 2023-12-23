#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <Adafruit_ADS1X15.h>
#include "UTFT.h"

#include "Pump.h"

#include "GyverPID.h"
#include "GyverTimers.h"

void parse_message(const String &str);
void pump_start_handler();
void pump_stop_handler();
void set_pump_rotation_speed_handler(const String &str);
void set_pump_rotate_direction(const String &str);
void set_P(const String &str);
void set_I(const String &str);
void set_D(const String &str);
void set_tv(const String &str);

void PIDor(const float &value);
void check_button(const uint8_t &button_number);

void regime1_handler(const uint8_t &binState);
void regime2_handler(const uint8_t &binState);
void calibration_handler(const uint8_t &binState);
void block_handler(const uint8_t &binState);
void kidney_handler(const uint8_t &binState);

const uint32_t PRESSURE_SENSOR_TICK_RATE = 100;

// GyverPID pid(1, 1, 1, 960);
GyverPID pid(0.2, 0.2, 0.2, PRESSURE_SENSOR_TICK_RATE);

uint16_t target_pressure_value = 29;
float fill_value = 0;

/** TODO: Store to EEPROM */
float pressure_shift = 0;
float resistance = 0;

Pump pump;

bool is_data_transmitted = false;

enum Regime
{
	STOPED,
	REGIME1,
	REGIME2
};

enum KidneyState
{
	LEFT_KIDNEY,
	RIGTH_KIDNEY
};

enum AlertType {
	NONE,
	PRESSURE_LOW,
	PRESSURE_HIGH,
	PRESSURE_UP,
	TEMP_LOW,
	TEMP_HIGH,
	RESISTANCE
};

const uint8_t alert_size = 7;
bool alert[alert_size] = {1, 0, 0, 0, 0, 0, 0};

KidneyState kidney_selector = KidneyState::LEFT_KIDNEY;

bool is_blocked = false;

Regime regime_state = Regime::STOPED;

const uint8_t regime1 = 22;
const uint8_t regime2 = 23;
const uint8_t calibration = 24;
const uint8_t block = 25;
const uint8_t kidney = 26;

bool regime1_flag = false;
bool regime2_flag = false;
bool calibration_flag = false;
bool block_flag = false;
bool kidney_flag = false;

int16_t temperature1 = 0;
int16_t temperature2 = 0;

uint8_t hours = 0;
uint8_t mins = 0;
uint8_t secs = 0;

void task_pressure_sensor_read(void *params);
void task_draw_display(void *params);
void task_pump_control(void *params);
void task_CLI(void *params);
void task_process_buttons(void *params);
void task_handle_error(void *params);
void task_temperature_sensor(void *params);

void setup()
{
	Serial.begin(115200);

	// Configure and stop a timer (start by default)
	Timer5.setFrequency(1);
	Timer5.enableISR();
	Timer5.pause();

	xTaskCreate(task_pressure_sensor_read, "PressureRead", 128, NULL, 2, NULL);
	xTaskCreate(task_draw_display, "DrawDisplay", 256, NULL, 2, NULL);
	xTaskCreate(task_pump_control, "PumpControl", 256, NULL, 2, NULL);
	xTaskCreate(task_CLI, "CLI", 256, NULL, 2, NULL);
	xTaskCreate(task_process_buttons, "Buttons", 128, NULL, 2, NULL);
	xTaskCreate(task_handle_error, "Errors", 128, NULL, 2, NULL);
	xTaskCreate(task_temperature_sensor, "Temperature", 64, NULL, 2, NULL);
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
	else if (str.startsWith("set_P"))
	{
		set_P(str);
	}
	else if (str.startsWith("set_I"))
	{
		set_I(str);
	}
	else if (str.startsWith("set_D"))
	{
		set_D(str);
	}
	else if (str.startsWith("set_tv"))
	{
		set_tv(str);
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

void set_P(const String &str)
{
	int space_idx = str.indexOf(' ');

	String P = str.substring(space_idx + 1, str.length() - 1);
	pid.Kp = P.toFloat();

	// Serial.print("Set kP value to ");
	// Serial.print(P);
}

void set_I(const String &str)
{
	int space_idx = str.indexOf(' ');

	String I = str.substring(space_idx + 1, str.length() - 1);
	pid.Ki = I.toFloat();

	// Serial.print("Set ki value to ");
	// Serial.println(I);
}

void set_D(const String &str)
{
	int space_idx = str.indexOf(' ');

	String D = str.substring(space_idx + 1, str.length() - 1);
	pid.Kd = D.toFloat();

	// Serial.print("Set kd value to ");
	// Serial.println(D);
}

void set_tv(const String &str)
{
	int space_idx = str.indexOf(' ');

	String target_value = str.substring(space_idx + 1, str.length() - 1);
	target_pressure_value = target_value.toInt();
	pid.setpoint = target_pressure_value;

	// Serial.print("Set kd value to ");
	// Serial.println(target_value);
}

void PIDor(const float &value)
{
	pid.input = value;
	pump.set_speed(pid.getResultTimer());
}

void check_button(const uint8_t &button_number)
{
	bool btnState = digitalRead(button_number);
	switch (button_number)
	{
	case regime1:
		regime1_handler(btnState);
		break;
	case regime2:
		regime2_handler(btnState);
		break;
	case calibration:
		calibration_handler(btnState);
		break;
	case block:
		block_handler(btnState);
		break;
	case kidney:
		kidney_handler(btnState);
		break;
	}
}

void regime1_handler(const uint8_t &btnState)
{
	if (btnState && !regime1_flag)
	{
		regime1_flag = true;

		if (regime_state == Regime::STOPED)
		{
			regime_state = Regime::REGIME1;
			Timer5.restart();
			hours = 0;
			mins = 0;
			secs = 0;
		}
		else if (regime_state == Regime::REGIME1)
		{
			regime_state = Regime::STOPED;
			Timer5.stop();
		}

		Serial.print("INFO: Current regime after first button clicked is ");
		Serial.println(regime_state);
	}
	if (!btnState && regime1_flag)
	{
		regime1_flag = false;
	}
}

void regime2_handler(const uint8_t &btnState)
{
	if (btnState && !regime2_flag)
	{
		regime2_flag = true;

		if (regime_state == Regime::STOPED)
		{
			regime_state = Regime::REGIME2;
		}
		else if (regime_state == Regime::REGIME2)
		{
			regime_state = Regime::STOPED;
		}

		Serial.print("INFO: Current regime after first button clicked is ");
		Serial.println(regime_state);
	}
	if (!btnState && regime2_flag)
	{
		regime2_flag = false;
	}
}

void calibration_handler(const uint8_t &btnState)
{
	if (btnState && !calibration_flag)
	{
		calibration_flag = true;
		pressure_shift = fill_value;

		Serial.print("INFO: Calibrated value is ");
		Serial.println(pressure_shift);
	}
	if (!btnState && calibration_flag)
	{
		calibration_flag = false;
	}
}

void block_handler(const uint8_t &btnState)
{
	if (btnState && !block_flag)
	{
		block_flag = true;

		is_blocked = !is_blocked;
	}
	if (!btnState && block_flag)
	{
		block_flag = false;
	}
}

void kidney_handler(const uint8_t &btnState)
{
	if (btnState && !kidney_flag)
	{
		kidney_flag = true;

		if (kidney_selector == KidneyState::LEFT_KIDNEY)
		{
			kidney_selector = KidneyState::RIGTH_KIDNEY;
		}
		else if (kidney_selector == KidneyState::RIGTH_KIDNEY)
		{
			kidney_selector = KidneyState::LEFT_KIDNEY;
		}
	}
	if (!btnState && kidney_flag)
	{
		kidney_flag = false;
	}
}

ISR(TIMER5_A) {
	++secs;

	if (secs > 59) {
		secs = 0;
		++mins;
	}

	if (mins > 59) {
		mins = 0;
		++hours;
	}
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

	pid.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
	pid.setLimits(1, 100);	  // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
	pid.setpoint = 29;

	uint8_t counter = 0;
	float sum = 0;

	bool is_first_regime2_start = true;

	for (;;) // A Task shall never return or exit.
	{
		if (counter > 9)
		{
			if (is_data_transmitted == false)
			{
				fill_value += ((sum / counter) - fill_value) * k;
				// Serial.println(fill_value);

				counter = 0;
				sum = 0;

				if (regime_state == Regime::REGIME1)
				{
					if (pump.get_state() == PumpStates::OFF)
					{
						pump.set_speed(10);
						vTaskDelay(1000 / 16);
						pump.start();
					}

					PIDor(fill_value);
				}
				else if (regime_state == Regime::REGIME2)
				{
					if (is_first_regime2_start)
					{
						// pump.set_speed(200 / 0.8);
						pump.set_speed(100);

						_delay_ms(20);

						if (pump.get_state() == PumpStates::OFF)
						{
							pump.start();
						}

						is_first_regime2_start = false;
					}
				}
				else if (regime_state == Regime::STOPED)
				{
					if (pump.get_state() == PumpStates::ON)
					{
						pump.stop();
						pump.set_speed(10);
						vTaskDelay(1000 / 16);
					}

					is_first_regime2_start = true;
				}
			}
		}
		else
		{
			++counter;
			int16_t raw_data = ads.getLastConversionResults();
			float converted_value = raw_data * 7.8125 / 25 - pressure_shift;

			sum += converted_value;
		}
		vTaskDelay(PRESSURE_SENSOR_TICK_RATE / 16); // one tick delay (16ms) in between reads for stability
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

	float flow = 10.0;			// perfusion speed
	float temperature_1 = 10.0; // value from temperature sensor 1
	float temperature_2 = 10.0; // value from temperature sensor 2

	// объявляем объект myGLCD класса библиотеки UTFT указывая тип дисплея
	UTFT myGLCD(TFT01_24SP, dispMISO,
				dispSCK, dispCS,
				dispRST, dispDC);

	myGLCD.InitLCD();
	myGLCD.fillScr(VGA_WHITE);		// fill screen with white colour
	myGLCD.setColor(VGA_BLACK);		// set black colour for fonts
	myGLCD.setFont(SmallFont);		// set SmallFont
	myGLCD.setBackColor(VGA_WHITE); // set white colour for background colour

	// Pressure value output
	myGLCD.drawRect(70, 0, 110, 20);  // pressure value rectangle
	myGLCD.drawRect(115, 0, 150, 20); // pressure value units rectangle
	// Resistance value output
	myGLCD.drawRect(85, 35, 125, 55);  // resistance value rectangle
	myGLCD.drawRect(130, 35, 230, 55); // resistance value units rectangle
	// Flow rate
	myGLCD.drawRect(35, 70, 75, 90);  // flow value rectangle
	myGLCD.drawRect(80, 70, 135, 90); // flow value units rectangle
	// Temperature sensor 1
	myGLCD.drawRect(45, 105, 85, 125);	// Temperature_1 value rectangle
	myGLCD.drawRect(90, 105, 105, 125); // Temperature_1 value units rectangle
	// Temperature sensor 2
	myGLCD.drawRect(45, 140, 85, 160);	// Temperature_2 value rectangle
	myGLCD.drawRect(90, 140, 105, 160); // Temperature_2 value units rectangle
	// Procedure time
	myGLCD.drawRect(183, 135, 218, 155); // timer_hour rectangle
	myGLCD.drawRect(228, 135, 263, 155); // timer_min rectangle
	myGLCD.drawRect(273, 135, 308, 155); // timer_sec rectangle
	// Regime output
	myGLCD.drawRect(183, 3, 315, 23); // Regime state rectangle
	// Kidney
	myGLCD.drawRect(220, 63, 315, 83); // Regime state rectangle
	// Alarm
	myGLCD.drawRect(10, 210, 100, 230); // Alarm state rectangle
	// Alarm type
	myGLCD.drawRect(183, 210, 315, 230); // Alarm type rectangle
	// Block state
	myGLCD.drawRect(183, 170, 305, 190); // block state rectangle

	Serial.println("LCD initialized!");

	for (;;)
	{
		flow = pump.get_speed() * 0.8;
		resistance = fill_value / flow;
		temperature_1 = random(0, 50);
		temperature_2 = random(0, 50);

		myGLCD.setFont(SmallFont);	// set SmallFont
		myGLCD.setColor(VGA_BLACK); // set black colour for fonts
		// Pressure value output
		myGLCD.print("Pressure", 0, 0);			// print "Pressure"
		myGLCD.printNumF(fill_value, 1, 77, 3); // print pressure value
		myGLCD.print("inHg", 118, 3);			// print "Pressure unit"

		// Resistance value output
		if (alert[AlertType::RESISTANCE]) {
			myGLCD.setColor(VGA_RED);
		}
		else {
			myGLCD.setColor(VGA_BLACK);
		}

		myGLCD.print("Resistance", 0, 35);
		myGLCD.setColor(VGA_BLACK);
		myGLCD.printNumF(resistance, 1, 90, 38); // print resistance value
		myGLCD.print("inHg/ml/min", 133, 38);	 // print "resistance unit"


		// Flow rate
		myGLCD.print("Flow", 0, 70);	   // print "flow"
		myGLCD.printNumF(flow, 1, 40, 73); // print flow value
		myGLCD.print("ml/min", 85, 73);	   // print "flow unit"

		// Temperature sensor 1
		myGLCD.print("Temp1", 0, 105);				 // print "Temp_1"
		myGLCD.printNumF(temperature_1, 1, 50, 108); // print Temperature_1 value
		myGLCD.print("C", 95, 108);					 // print "Temperature_1 unit"

		// Temperature sensor 2
		myGLCD.print("Temp2", 0, 138);				 // print "Temp_2"
		myGLCD.printNumF(temperature_2, 1, 50, 143); // print Temperature_2 value
		myGLCD.print("C", 95, 143);					 // print "Temperature_2 unit"

		// Procedure time
		myGLCD.setFont(SmallFont);				  // set SmallFont
		myGLCD.print("Procedure time", 186, 120); // print "Procedure time"

		myGLCD.printNumF(hours, 1, 186, 140); // print hours value
		myGLCD.printNumF(mins, 1, 231, 140);  // print minutes value
		myGLCD.printNumF(secs, 1, 276, 140);  // print seconds value

		// Regime_output
		myGLCD.setFont(BigFont); // set BigFont
		// print Regime
		if (regime_state == Regime::STOPED)
		{
			myGLCD.print("Stopped ", 186, 6);
		}
		else if (regime_state == Regime::REGIME1)
		{
			myGLCD.print("Regime 1", 186, 6); // print "Regime 1"
		}
		else if (regime_state == Regime::REGIME2)
		{
			myGLCD.print("Regime 2", 186, 6); // print "Regime 2"
		}

		// myGLCD.drawRect(183, 170, 308, 190); // block state rectangle
		if (is_blocked == true)
		{
			myGLCD.print("Blocked", 184, 173); // print "Blocked"
		}
		else if (is_blocked == false)
		{
			myGLCD.print("       ", 184, 173); // print "Right"
		}

		// Kidney
		// print Kidney
		if (kidney_selector == KidneyState::RIGTH_KIDNEY)
		{
			myGLCD.print("RIGHT", 225, 66); // print "Right"
		}
		else if (kidney_selector == KidneyState::LEFT_KIDNEY)
		{
			myGLCD.print("LEFT ", 225, 66); // print "Right"
		}

		// Alarm
		// print alarm state
		if (alert[AlertType::NONE] == false)
		{
			myGLCD.setColor(VGA_RED);		// set black colour for fonts
			myGLCD.print("ALARM", 13, 213); // print "ALARM"
		}
		else
		{
			myGLCD.print("     ", 13, 213); // print "ALARM"
		}

		// Alarm type
		// print Alarm type
		if (alert[AlertType::NONE])
		{
			myGLCD.print("        ", 186, 213); // print "ALARM TYPE"
		}
		else if (alert[AlertType::PRESSURE_HIGH])
		{
			myGLCD.print("PR-HIGH", 186, 213);	// print "ALARM TYPE"
		}
		else if (alert[AlertType::PRESSURE_LOW])
		{
			myGLCD.print("PR-LOW ", 186, 213); // print "ALARM TYPE"
		}
		else if (alert[AlertType::PRESSURE_UP])
		{
			myGLCD.print("PR-UP  ", 186, 213); // print "ALARM TYPE"
		}
		else
		{
			myGLCD.print("        ", 186, 213); // print "ALARM TYPE"
		}

		vTaskDelay(6);
	}
}

void task_pump_control(void *params)
{
	for (;;)
	{
		pump.process();
		vTaskDelay(3);
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

			// Serial.print("ECHO: ");
			// Serial.print(message);

			parse_message(message);

			is_data_transmitted = false;
			// 	xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
			// }
		}

		/** TODO: Parse pump output and check that pump is available */
		if (Serial3.available() >= 1)
		{
			Serial3.readBytes(pump.reply, Serial3.available());
			// uint8_t buff[128];
			// uint16_t size = Serial3.available();
			// Serial3.readBytes(buff, size);
			// Serial.write(buff, size);
		}

		vTaskDelay(5);
	}
}

void task_process_buttons(void *params)
{
	pinMode(regime1, INPUT);
	pinMode(regime2, INPUT);
	pinMode(calibration, INPUT);
	pinMode(block, INPUT);
	pinMode(kidney, INPUT);

	for (;;)
	{
		if (!is_blocked)
		{
			check_button(regime1);
			check_button(regime2);
			check_button(calibration);
			check_button(kidney);
		}
		check_button(block);

		vTaskDelay(100 / 16);
	}
}

void task_handle_error(void *params) {
	
	// vTaskDelay(15000);

	// Optimal borders 28 - 30
	// Working borders 28 - 40

	for (;;)
	{
		/** TODO: Fix bug - fill_value may have pressure_up and
		 * pressure_high errors */
		// if (fill_value > 30) {
		// 	alert[AlertType::PRESSURE_UP] = true;
		// }
		// else {
		// 	alert[AlertType::PRESSURE_UP] = false;
		// }

		// if (fill_value < 28) {
		// 	// pump.stop();
		// 	// regime_state = Regime::STOPED;
		// 	alert[AlertType::PRESSURE_LOW] = true;
		// }
		// else {
		// 	alert[AlertType::PRESSURE_LOW] = false;
		// }

		// if (fill_value > 40) {
		// 	// pump.stop();
		// 	// regime_state = Regime::STOPED;
		// 	alert[AlertType::PRESSURE_HIGH] = true;
		// }
		// else {
		// 	alert[AlertType::PRESSURE_HIGH] = false;
		// }

		// if (resistance > 1.1) {
		// 	alert[AlertType::RESISTANCE] = true;
		// }
		// else {
		// 	alert[AlertType::RESISTANCE] = false;
		// }

		// /** TODO: Add other alert handlers */
		
		// bool is_error_occured = false;
		// for (uint8_t i = 1; i < alert_size; ++i) {
		// 	if (alert[i] == true) {
		// 		is_error_occured = true;
		// 		break;
		// 	}
		// }

		// alert[AlertType::NONE] = !is_error_occured;
		

		vTaskDelay(1000 / 16);
	}
}

void task_temperature_sensor(void *params) {
	/** TODO: Read the temperature */

	for (;;) {

		vTaskDelay(1000 / 16);
	}
}