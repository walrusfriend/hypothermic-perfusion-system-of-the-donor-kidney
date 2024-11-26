#include "config.h"
#include <semphr.h>

#include <Adafruit_ADS1X15.h>

#include "Pump.h"
#include "custom_time.h"
#include "bubble_remover.h"
#include "CLI.h"

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
void tare_pressure_handler(const String& str);
void set_perfussion_speed_ratio_handler(const String& str);
void set_pump_rotate_direction(const String &str);
void set_tv(const String &str);
void start_handler(const String& str);
void pause_handler(const String& str);
void stop_handler(const String& str);
void regime_handler(const String& str);

void set_PID(const float &value);
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
float pressure = 1;

float pressure_shift = 0;
float resistance = 0;
float perfussion_ratio = 0.6;

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

/**
 * float -> uin32_t -> 4 bytes * 4 -> 16 bytes for all float values
 * uint8_t -> 1 byte * 3 -> 3 bytes for all uint8_t values
 * Time -> 3 byte
 * alert array -> 9 bytes
 * 
 * Pack regime_state, kidney_selector and block state to 1 byte
 * Regime_state takes 2 bits now, give 3 bits for future
 * kidney_selector takes only 1 bit
 * block_state takes only 1 bit too
 * 
 * Delete first alert byte (which is NONE) and pack it to 1 byte
 * 
 * floats + time + packed byte 1 + alert byte =
 * 20 + 3 + 1 + 1 = 25 + \n = 26
 * 
 * UPDATE:
 * We can delete resistance from transmission
 * 16 + 3 + 1 + 1 = 21 + \n = 22
 */
const uint8_t TO_SEND_ARRAY_SIZE = 22;
uint8_t to_send[TO_SEND_ARRAY_SIZE];

static const Command command_list[] = {
	Command("start", start_handler),
	Command("pause", pause_handler),
	Command("stop", stop_handler),
	Command("regime", regime_handler),
	Command("set_speed", set_pump_rotation_speed_handler),
	Command("tare_pressure", tare_pressure_handler),
	Command("set_perfussion_speed_ratio", set_perfussion_speed_ratio_handler),
	Command("set_tv", set_tv)
};

void task_pressure_sensor_read(void *params);
void task_pump_control(void *params);
void task_CLI(void *params);
void task_process_buttons(void *params);
void task_handle_error(void *params);
void task_temperature_sensor(void *params);
void task_bubble_remover(void* params);

void setup()
{
	to_send[TO_SEND_ARRAY_SIZE - 1] = '\n';

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
	xTaskCreate(task_pump_control, "PumpControl", 512, NULL, 2, NULL);
	xTaskCreate(task_CLI, "CLI", 256, NULL, 2, NULL);
	xTaskCreate(task_process_buttons, "Buttons", 128, NULL, 2, NULL);
	xTaskCreate(task_handle_error, "Errors", 128, NULL, 2, NULL);
	xTaskCreate(task_temperature_sensor, "Temperature", 256, NULL, 2, NULL);
	xTaskCreate(task_bubble_remover, "BubbleRemover", 128, NULL, 2, NULL);
}

void loop() {}

void parse_message(const String &message)
{
	for (uint8_t i = 0; i < sizeof(command_list); ++i)
	{
		if (message.startsWith(command_list[i].name))
		{
			command_list[i].handler(message);
			return;
		}
	}

	Serial.println("ERROR: Unknown command!");
}

void pump_start_handler()
{
	pump.start();
}

void pump_stop_handler()
{
	pump.stop();
}

void set_pump_rotation_speed_handler(const String &str)
{
	int space_idx = str.indexOf(' ');

	if (space_idx > str.length())
	{
		// Serial.println("ERROR: Space index are more than string size!");
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
		// Serial.println("ERROR: space index more than LF index!");
		return;
	}

	String float_str = str.substring(space_idx + 1, LF_idx);

	float pump_rmp = float_str.toFloat();

	/** TODO: Add command reply to Qt program */

	// Serial.print("DEBUG: Set speed value to ");
	// Serial.println(pump_rmp);

	pump.set_speed(pump_rmp);
}

void tare_pressure_handler(const String& str) {
	pressure_shift = pressure;
}

void set_perfussion_speed_ratio_handler(const String& str) {
	int space_idx = str.indexOf(' ');

	if (space_idx > str.length())
	{
		// Serial.println("ERROR: Space index are more than string size!");
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
		// Serial.println("ERROR: space index more than LF index!");
		return;
	}

	String float_str = str.substring(space_idx + 1, LF_idx);
	perfussion_ratio = float_str.toFloat();
}

void set_pump_rotate_direction(const String &str)
{
	char *strtok_index;
	char buff[64];

	strtok_index = strtok(str.c_str(), " ");
	strcpy(buff, strtok_index);
	// Serial.print(buff);

	strtok_index = strtok(NULL, " ");
	strcpy(buff, strtok_index);
	// Serial.print(buff);

	// Serial.println("Set the rotate direction to clockwise");
	pump.set_rotate_direction((buff[0] == '0') ? RotateDirections::COUNTERCLOCKWISE : RotateDirections::CLOCKWISE);
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


void start_handler(const String& message) {
	Timer5.resume();
	regime_state = Regime::REGIME1;
}

void pause_handler(const String& message) {
	Timer5.pause();
	regime_state = Regime::STOPED;
}

void stop_handler(const String& message) {
	Timer5.stop();
	regime_state = Regime::STOPED;

	time.set_hours(0);
	time.set_mins(0);
	time.set_secs(0);
}

void regime_handler(const String& message) {
	/* Handle invalid input */
	if (message.length() == 0)
		return;

	char input_regime = message[message.length() - 1];

	if (isDigit(input_regime)) {
		regime_state = static_cast<Regime>(input_regime - '0');
	}
}

void set_PID(const float &value)
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
	if (!btnState && !regime1_flag)
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

		// Serial.print("INFO: Current regime after first button clicked is ");
		// Serial.println(regime_state);
	}
	if (btnState && regime1_flag)
	{
		regime1_flag = false;
	}
}

void regime2_handler(const uint8_t &btnState)
{
	if (!btnState && !regime2_flag)
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

		// Serial.print("INFO: Current regime after second button clicked is ");
		// Serial.println(regime_state);
	}
	if (btnState && regime2_flag)
	{
		regime2_flag = false;
	}
}

void calibration_handler(const uint8_t &btnState)
{
	if (!btnState && !calibration_flag)
	{
		calibration_flag = true;
		pressure_shift = pressure;

		// Serial.print("INFO: Calibrated value is ");
		// Serial.println(pressure_shift);
	}
	if (btnState && calibration_flag)
	{
		calibration_flag = false;
	}
}

void block_handler(const uint8_t &btnState)
{
	if (!btnState && !block_flag)
	{
		block_flag = true;

		is_blocked = !is_blocked;

		// if (is_blocked)
			// Serial.println("Block is activated");
		// else
			// Serial.println("Block is disabled");
	}
	if (btnState && block_flag)
	{
		block_flag = false;
	}
}

void kidney_handler(const uint8_t &btnState)
{
	if (!btnState && !kidney_flag)
	{
		kidney_flag = true;

		if (kidney_selector == KidneyState::LEFT_KIDNEY)
		{
			kidney_selector = KidneyState::RIGTH_KIDNEY;
			// Serial.println("Right kidney selected");
		}
		else if (kidney_selector == KidneyState::RIGTH_KIDNEY)
		{
			kidney_selector = KidneyState::LEFT_KIDNEY;
			// Serial.println("Left kidney selected");
		}
	}
	if (btnState && kidney_flag)
	{
		kidney_flag = false;
	}
}

ISR(TIMER5_A)
{
	/* Write flow */
	float flow = pump.get_speed() * perfussion_ratio;
	uint8_t* magic = ((uint8_t*)(&flow));
	uint8_t* p_writer = to_send;

	for(uint8_t i = 0; i < 4; i++) {
		*(p_writer++) = magic[i];
	}

	/* Write pressure */
	magic = ((uint8_t*)(&pressure));

	for(uint8_t i = 0; i < 4; i++) {
		*(p_writer++) = magic[i];
	}

	/* Write temp1 */
	magic = ((uint8_t*)(&temperature1));

	for(uint8_t i = 0; i < 4; i++) {
		*(p_writer++) = magic[i];
	}

	/* Write temp2 */
	magic = ((uint8_t*)(&temperature2));

	for(uint8_t i = 0; i < 4; i++) {
		*(p_writer++) = magic[i];
	}

	/* Write time */
	*(p_writer++) = time.get_hours();
	*(p_writer++) = time.get_mins();
	*(p_writer++) = time.get_secs();

	/* Write packed regime + kidney_selector + is_blocked */
	uint8_t packed_byte = (regime_state) | 
						  (kidney_selector << 3) |
						  (is_blocked << 4);
	*(p_writer++) = packed_byte;

	/* Write alert */
	/* Skip first code - NONE */
	uint8_t alert_byte = 0;
	for (uint8_t i = 1; i < alert_size; i++) {
		alert_byte |= (alert[i] & 0b1) << (i - 1);
	}
	*(p_writer++) = alert_byte;

	++time;

	/** TODO: Add messages to queue and send it to COM port outside of the interrupt */
	Serial.write(to_send, TO_SEND_ARRAY_SIZE);
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
	// if (remove_bubble_secs >= 60) {
	if (remove_bubble_secs >= 5) {
		bubble_remover.stop(regime_state);
		remove_bubble_secs = 0;
		regime_state = Regime::REGIME1;
		// Serial.println("Remove kebab complete");
	}

}

void task_pressure_sensor_read(void *params)
{
	Adafruit_ADS1115 ads;

	ads.setGain(GAIN_SIXTEEN);
	if (!ads.begin())
	{
		// Serial.println("Failed to initialize ADS.");
		while (1);
	}

	// Serial.println("ADC initialized successfully");

	// Start continuous conversions.
	ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);

	float k = 0.2;

	pid.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
	pid.setLimits(1, 100);	  // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
	pid.setpoint = 29;

	bool is_first_regime2_start = true;

	uint8_t counter = 0;
	float pressure_sum = 0;
	float average_sistal[10];

	for (;;)
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
				// pressure += ((pressure_sum / counter) - pressure) * k;
				// Serial.println(pressure);

				/**
				 * Сортировка позырьком, почему так? 
				 * TODO: Применить нормальную сортировку
				 */

				/** TODO: Check new sort function */
				// qsort(average_sistal, counter, sizeof(average_sistal[0]), 
				// 	[](const void* a, const void* b) -> int 
				// 	{
				// 		return *((int*)a) - *((int*)b);
				// 	}
				// );

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
				pressure += (average_value - pressure) * k;

				pressure_sum = 0;

						/* В первом режиме включаем минимальную скорость и запускаем ПИД */
				if (regime_state == Regime::REGIME1)
				{
					if (pump.get_state() == PumpStates::OFF)
					{
						pump.set_speed(10);
						vTaskDelay(1000 / 16);
						pump.start();
					}

					set_PID(pressure);
				}
				/* Во втором режиме просто шарашим на полную */
				else if (regime_state == Regime::REGIME2)
				{
					if (is_first_regime2_start)
					{
						pump.set_speed(100);

						_delay_ms(20);

						if (pump.get_state() == PumpStates::OFF)
						{
							pump.start();
						}

						is_first_regime2_start = false;
					}
				}
				else if (regime_state == Regime::REGIME_REMOVE_BUBBLE) {
					pump.set_speed(PUMP_MAX_SPEED);

					if (pump.get_state() == PumpStates::OFF) {
						pump.start();
					}
				}
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

		if (Serial.available() >= 1)
		{
			is_data_transmitted = true;

			char sym[64];
			int size = Serial.readBytesUntil('\n', sym, 100);
			sym[size] = '\0';

			parse_message(String(sym));

			is_data_transmitted = false;
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
	pinMode(Pin::regime1, INPUT_PULLUP);
	pinMode(Pin::regime2, INPUT_PULLUP);
	pinMode(Pin::calibration, INPUT_PULLUP);
	pinMode(Pin::block, INPUT_PULLUP);
	pinMode(Pin::kidney, INPUT_PULLUP);

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
				if (pressure >= PRESSURE_STABLE_VALUE)
				{
					is_pressure_stabilized = true;
				}
			}
		}

		if (is_pressure_stabilized and regime_state == Regime::REGIME1)
		{
			if (pressure > PRESSURE_OPTIMAL_HIGH_LIMIT)
			{
				alert[AlertType::PRESSURE_UP] = true;
				alert[AlertType::PRESSURE_HIGH] = false;
				alert[AlertType::PRESSURE_LOW] = false;
			}
			else
			{
				alert[AlertType::PRESSURE_UP] = false;
			}

			if (pressure < PRESSURE_LOW_LIMIT)
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

			if (pressure > PRESSURE_HIGH_LIMIT)
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

			if (pressure > 28 and pressure < 40)
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
			// Serial.println("error");

		if (sensor2.readTemp())
			temperature2 = sensor2.getTemp();
		else
			// Serial.println("error");

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
			// Serial.println("Bubble emulated");
			bubble_remover.start(regime_state);
		}

		/* Ставим минимальное ожидание в треде, чтобы не пропустить пузырёк */
		/* Но пока поставим не сильно много, чтобы не было дребезка кнопки при эмуляции */
		// vTaskDelay(1);
		vTaskDelay(200 / 16);
	}
}