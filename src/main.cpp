#include "config.h"
#include <semphr.h>

#include <Adafruit_ADS1X15.h>

#include "Pump.h"
#include "custom_time.h"
#include "bubble_remover.h"

#include "GyverPID.h"
#include "GyverTimers.h"

#include <microDS18B20.h>

/**
 * TODO: Проверить, что треды не владеют общими ресурсами,
 * иначе обвесить всё семафорами
 * 
 * TODO: Сейчас режим продувки, по идее, можно прервать с кнопки
 */

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

GyverPID pid(0.2, 0.2, 0.2, PRESSURE_SENSOR_TICK_RATE);
Pump pump;

uint16_t target_pressure_value = 29;
float fill_value = 1;

float pressure_shift = 0;
float resistance = 0;

bool is_data_transmitted = false;

const uint8_t alert_size = 9;
bool alert[alert_size] = {1, 0, 0, 0, 0, 0, 0, 0, 0};

KidneyState kidney_selector = KidneyState::LEFT_KIDNEY;
Regime regime_state = Regime::STOPED;

BubbleRemover bubble_remover;

bool is_blocked = false;

bool regime1_flag = false;
bool regime2_flag = false;
bool calibration_flag = false;
bool block_flag = false;
bool kidney_flag = false;

float temperature1 = 0;
float temperature2 = 0;

bool is_pressure_stabilized = false;

Time time {0, 0, 0};

uint8_t error_timer_mins = 0;
uint8_t error_timer_secs = 0;

uint8_t remove_bubble_secs = 0;

bool is_system_blocked = false;
bool is_error_timer_start = false;

void task_pressure_sensor_read(void *params);
void task_create_report(void *params);
void task_pump_control(void *params);
void task_CLI(void *params);
void task_process_buttons(void *params);
void task_handle_error(void *params);
void task_temperature_sensor(void *params);
void task_bubble_remover(void* params);

void setup()
{
	Serial.begin(115200);

	// Configure and stop a timer (start by default)
	Timer5.setFrequency(1);
	Timer5.enableISR();
	Timer5.stop();

	/* Настроим таймер для продувки пузырьков */
	Timer3.setFrequency(1);		// Частота 1 Гц
	Timer3.enableISR();
	Timer3.stop();

	xTaskCreate(task_pressure_sensor_read, "PressureRead", 128, NULL, 2, NULL);
	xTaskCreate(task_create_report, "CreateReport", 256, NULL, 2, NULL);
	xTaskCreate(task_pump_control, "PumpControl", 512, NULL, 2, NULL);
	xTaskCreate(task_CLI, "CLI", 256, NULL, 2, NULL);
	xTaskCreate(task_process_buttons, "Buttons", 128, NULL, 2, NULL);
	xTaskCreate(task_handle_error, "Errors", 128, NULL, 2, NULL);
	xTaskCreate(task_temperature_sensor, "Temperature", 256, NULL, 2, NULL);
	xTaskCreate(task_bubble_remover, "BubbleRemover", 128, NULL, 2, NULL);
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
	case Pin::regime1:
		regime1_handler(btnState);
		break;
	case Pin::regime2:
		regime2_handler(btnState);
		break;
	case Pin::calibration:
		calibration_handler(btnState);
		break;
	case Pin::block:
		block_handler(btnState);
		break;
	case Pin::kidney:
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
			time.reset();

			is_pressure_stabilized = false;
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
			Timer5.stop();
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

ISR(TIMER5_A)
{
	++time;
}

ISR(TIMER4_A)
{
	++error_timer_secs;

	if (error_timer_secs > 59)
	{
		error_timer_secs = 0;
		++error_timer_mins;
	}

	if (error_timer_mins == 10)
	{
		// Block the system
		is_system_blocked = true;
		pump.stop();
	}
}

ISR(TIMER3_A)
{
	++remove_bubble_secs;

	/** 
	 * По прошествии минуты обнуляем таймер и 
	 * возвращаемся к нормальному режиму работы 
	 */
	if (remove_bubble_secs >= 60) {
		bubble_remover.stop(regime_state);
		remove_bubble_secs = 0;
	}
}

void task_pressure_sensor_read(void *params)
{
	Adafruit_ADS1115 ads;

	ads.setGain(GAIN_SIXTEEN);
	if (!ads.begin())
	{
		Serial.println("Failed to initialize ADS.");
		while (1);
	}

	// Start continuous conversions.
	ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_2_3, /*continuous=*/true);

	float k = 0.2;

	pid.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
	pid.setLimits(1, 100);	  // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
	pid.setpoint = 29;

	uint8_t counter = 0;
	float pressure_sum = 0;

	bool is_first_regime2_start = true;

	float average_sistal[10];

	for (;;) // A Task shall never return or exit.
	{
		/* Если система упала в блокировку, то тупо ничего не делаем */
		if (is_system_blocked)
		{
			vTaskDelay(1000);
			continue;
		}

		/**
		 * Производим усреднение по 10-ти значениям
		 * 
		 * TODO: Загнать количество точек усреднения в константу или переменную,
		 * если хотим управлять ей через СОМ порт
		 * 
		 * TODO: Можно сделать плавующую среднюю по последним N значениям
		 */

		if (counter < 10) {
			/* Читаем данные с АЦП */
			int16_t raw_data = ads.getLastConversionResults();

			/* Преобразуем значение давления по формуле */
			float converted_value = raw_data * 7.8125 / 25 - pressure_shift;

			// Serial.print(fill_value);
			// Serial.print(' ');
			// Serial.println(converted_value);

			// pressure_sum += converted_value;

			/* Сохраняем средние значения */
			average_sistal[counter] = converted_value;
			++counter;
		}
		else {
			/** TODO: А что мы тут проверяем?
			 * Зачем нам блокировать вычисление средней, если идёт парсинг с СОМ порта?
			 * Я так понимаю, тут мы производим усреднение\
			 * Тут мы можем улететь в бесконечный цикл
			 */
			if (is_data_transmitted == false)
			{
				// fill_value += ((pressure_sum / counter) - fill_value) * k;
				// Serial.println(fill_value);

				/**
				 * Сортировка позырьком, почему так? 
				 * TODO: Применить нормальную сортировку
				 */
				int i, j;
				bool swapped;
				for (i = 0; i < 10 - 1; i++)
				{
					swapped = false;
					for (j = 0; j < 10 - i - 1; j++)
					{
						if (average_sistal[j] > average_sistal[j + 1])
						{
							/* Тут вообще swap отвалился */
							// swap(float, average_sistal[j], average_sistal[j + 1]);
							float tmp = average_sistal[j];
							average_sistal[j] = average_sistal[j + 1];
							average_sistal[j + 1] = tmp;
							swapped = true;
						}
					}

					// If no two elements were swapped
					// by inner loop, then break
					if (swapped == false)
						break;
				}

				/* Выбираем элементы с 4-го по 8-й */
				for (uint8_t i = 0; i < 5; ++i)
				{
					pressure_sum += average_sistal[4 + i];
				}

				/* Вычисляем среднее */
				float average_value = pressure_sum / 5;

				/* Душим скачки давления */
				fill_value += (average_value - fill_value) * k;

				pressure_sum = 0;

				/** TODO: Это нужно перенести в отдельный поток управления насосом!!! */
				/* В первом режиме включаем минимальную скорость и запускаем ПИД */
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
				/* Во втором режиме просто шарашим на полную */
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
				/* ANTIBALLBUSTING */
				else if (regime_state == Regime::REGIME_REMOVE_KEBAB) {
					pump.set_speed(PUMP_MAX_SPEED);

					if (pump.get_state() == PumpStates::OFF) {
						pump.start();
					}
				}
				/* Ну тут всё понятно */
				else if (regime_state == Regime::STOPED)
				{
					if (pump.get_state() == PumpStates::ON)
					{
						pump.stop();
						pump.set_speed(10);
						// vTaskDelay(1000 / 16);
					}

					is_first_regime2_start = true;
				}
			}

			/**
			 * Обнуляем переменную, даже если не произвели вычисления,
			 * иначе есть солидный шанс попасть в бесконечный цикл
			 */
			counter = 0;
		}

		vTaskDelay(PRESSURE_SENSOR_TICK_RATE / 16); // one tick delay (16ms) in between reads for stability
	}
}

/**
 * Тут мы будем формировать отчёт с текущими параметрами системы
 * Отправлять этот отчёт будет поток CLI
 */
void task_create_report(void *params)
{
	float flow;

	char output[1024];

	for (;;)
	{
		/** TODO: Use coefficients as a named constant */
		// flow = pump.get_speed() * 0.8;
		flow = pump.get_speed() * 0.6;
		// flow = pump.get_speed() * 0.35;

		resistance = fill_value / flow;
		
		sprintf(output,
				"\nReport:\n"
				"\tPressure: %f inHg\n"
				"\tFlow: %f ml/min\n"
				"\tResistance: %f inHg/ml/min\n"
				"\tTemp1: %f C\n"
				"\tTemp2: %f C\n"
				"\tRegime: %d\n"
				"\tKidney: %s\n"
				"\ttime: %d:%d:%d\n"
				"\terrors:\n"
				"\t\tNONE: %d\n"
				"\t\tPRESSURE_LOW: %d\n"
				"\t\tPRESSURE_HIGH: %d\n"
				"\t\tPRESSURE_UP: %d\n"
				"\t\tTEMP1_LOW: %d\n"
				"\t\tTEMP1_HIGH: %d\n"
				"\t\tTEMP2_LOW: %d\n"
				"\t\tTEMP2_HIGH: %d\n"
				"\t\tRESISTANCE: %d\n"
				"\tblocked: %d\n",
				fill_value,
				flow,
				resistance,
				temperature1,
				temperature2,
				regime_state,
				(kidney_selector == KidneyState::LEFT_KIDNEY) ? "left" : "right",
				time.get_hours(), time.get_mins(), time.get_secs(),
				alert[NONE],
				alert[PRESSURE_LOW],
				alert[PRESSURE_HIGH],
				alert[PRESSURE_UP],
				alert[TEMP1_LOW],
				alert[TEMP1_HIGH],
				alert[TEMP2_LOW],
				alert[TEMP2_HIGH],
				alert[RESISTANCE],
				is_blocked);

		Serial.print(output);

		vTaskDelay(6);
	}
}

void task_pump_control(void *params)
{
	for (;;)
	{
		if (is_system_blocked)
		{
			vTaskDelay(1000);
			continue;
		}

		pump.process();
		vTaskDelay(3);
	}
}

void task_CLI(void *params)
{

	for (;;)
	{
		if (is_system_blocked)
		{
			vTaskDelay(1000);
			continue;
		}

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

			parse_message(message);

			is_data_transmitted = false;
			// 	xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
			// }
		}

		if (Serial3.available() >= 1)
		{
			Serial3.readBytes(pump.reply, Serial3.available());
		}

		vTaskDelay(5);
	}
}

void task_process_buttons(void *params)
{
	pinMode(Pin::regime1, INPUT);
	pinMode(Pin::regime2, INPUT);
	pinMode(Pin::calibration, INPUT);
	pinMode(Pin::block, INPUT);
	pinMode(Pin::kidney, INPUT);

	for (;;)
	{
		if (!is_blocked or is_system_blocked)
		{
			check_button(Pin::regime1);
			check_button(Pin::regime2);
			check_button(Pin::calibration);
			check_button(Pin::kidney);
		}
		check_button(Pin::block);

		vTaskDelay(100 / 16);
	}
}

void task_handle_error(void *params)
{
	const uint8_t PRESSURE_LOW_LIMIT = 28;
	const uint8_t PRESSURE_OPTIMAL_HIGH_LIMIT = 30;
	const uint8_t PRESSURE_HIGH_LIMIT = 40;
	const uint8_t PRESSURE_STABLE_VALUE = 29;

	const uint8_t TEMP_LOW_LIMIT = 4;
	const uint8_t TEMP_HIGH_LIMIT = 10;

	bool is_pressure_high_beat = false;

	// Add 10 mins timer - if after 10 mins pressure doesn't fall, stop
	// the system, if pressure fall below HIGH, stop the timer
	Timer4.setFrequency(1);
	Timer4.enableISR();

	for (;;)
	{
		if (is_system_blocked)
		{
			vTaskDelay(1000);
			continue;
		}

		if (regime_state == Regime::REGIME1)
		{
			if (!is_pressure_stabilized)
			{
				if (fill_value >= PRESSURE_STABLE_VALUE)
				{
					is_pressure_stabilized = true;
				}
			}
		}

		if (is_pressure_stabilized and regime_state == Regime::REGIME1)
		{
			if (fill_value > PRESSURE_OPTIMAL_HIGH_LIMIT)
			{
				alert[AlertType::PRESSURE_UP] = true;
				alert[AlertType::PRESSURE_HIGH] = false;
				alert[AlertType::PRESSURE_LOW] = false;
			}
			else
			{
				alert[AlertType::PRESSURE_UP] = false;
			}

			if (fill_value < PRESSURE_LOW_LIMIT)
			{
				// pump.stop();
				// regime_state = Regime::STOPED;
				alert[AlertType::PRESSURE_LOW] = true;
				alert[AlertType::PRESSURE_UP] = false;
				alert[AlertType::PRESSURE_HIGH] = false;

				if (!is_error_timer_start)
				{
					Timer4.restart();
					is_error_timer_start = true;
				}
			}
			else
			{
				alert[AlertType::PRESSURE_LOW] = false;
			}

			if (fill_value > PRESSURE_HIGH_LIMIT)
			{
				pump.stop();
				// regime_state = Regime::STOPED;
				alert[AlertType::PRESSURE_HIGH] = true;
				alert[AlertType::PRESSURE_LOW] = false;
				alert[AlertType::PRESSURE_UP] = false;

				is_pressure_high_beat = true;

				if (!is_error_timer_start)
				{
					Timer4.restart();
					is_error_timer_start = true;
				}
			}
			else
			{
				alert[AlertType::PRESSURE_HIGH] = false;
			}

			if (fill_value > 28 and fill_value < 40)
			{
				alert[AlertType::PRESSURE_HIGH] = false;
				alert[AlertType::PRESSURE_LOW] = false;
				alert[AlertType::PRESSURE_UP] = false;

				Timer4.stop();
				is_error_timer_start = false;

				if (is_pressure_high_beat) {
					pump.start();
					regime_state = Regime::REGIME1;
					is_pressure_high_beat = false;
				}
			}

			if (resistance > 1.1)
			{
				alert[AlertType::RESISTANCE] = true;
			}
			else
			{
				alert[AlertType::RESISTANCE] = false;
			}

			if (temperature1 < TEMP_LOW_LIMIT)
			{
				alert[AlertType::TEMP1_LOW] = true;
				alert[AlertType::TEMP1_HIGH] = false;
			}
			else if (temperature1 > TEMP_HIGH_LIMIT)
			{
				alert[AlertType::TEMP1_LOW] = false;
				alert[AlertType::TEMP1_HIGH] = true;
			}
			else
			{
				alert[AlertType::TEMP1_LOW] = false;
				alert[AlertType::TEMP1_HIGH] = false;
			}

			if (temperature2 < TEMP_LOW_LIMIT)
			{
				alert[AlertType::TEMP2_LOW] = true;
				alert[AlertType::TEMP2_HIGH] = false;
			}
			else if (temperature2 > TEMP_HIGH_LIMIT)
			{
				alert[AlertType::TEMP2_LOW] = false;
				alert[AlertType::TEMP2_HIGH] = true;
			}
			else
			{
				alert[AlertType::TEMP2_LOW] = false;
				alert[AlertType::TEMP2_HIGH] = false;
			}

			bool is_error_occured = false;
			for (uint8_t i = 1; i < alert_size; ++i)
			{
				if (alert[i] == true)
				{
					is_error_occured = true;
					break;
				}
			}

			alert[AlertType::NONE] = !is_error_occured;
		}

		vTaskDelay(1000 / 16);
	}
}

void task_temperature_sensor(void *params)
{
	MicroDS18B20<Pin::temperature1_pin> sensor1;
	MicroDS18B20<Pin::temperature2_pin> sensor2;

	for (;;)
	{
		if (is_system_blocked)
		{
			vTaskDelay(1000);
			continue;
		}

		sensor1.requestTemp();
		sensor2.requestTemp();

		if (sensor1.readTemp())
			temperature1 = sensor1.getTemp();
		else
			Serial.println("error");

		if (sensor2.readTemp())
			temperature2 = sensor2.getTemp();
		else
			Serial.println("error");

		vTaskDelay(1000 / 16);
	}
}

void task_bubble_remover(void* params) {

	for (;;)
	{
		if (is_system_blocked)
		{
			vTaskDelay(1000);
			continue;
		}

		/** 
		 * TODO: Что делать, если система заметила второй пузырь
		 * во время промывки первого?
		 * Сейчас мы будем начинать всё сначала
		 */

		/* Проверяем состояние кнопки (датчика пузырьков) */
		if (bubble_remover.is_bubble()) {
			bubble_remover.start(regime_state);
		}

		/* Ставим минимальное ожидание в треде, чтобы не пропустить пузырёк */
		/* Но пока поставим не сильно много, чтобы не было дребезка кнопки при эмуляции */
		// vTaskDelay(1);
		vTaskDelay(100 / 16);
	}
}