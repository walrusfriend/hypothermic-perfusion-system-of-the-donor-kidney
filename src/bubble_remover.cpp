#include "bubble_remover.h"
#include "GyverTimers.h"

BubbleRemover::BubbleRemover() {
    pinMode(Pin::emulator_button_pin, INPUT_PULLUP);
	pinMode(Pin::MOSFET_pin, OUTPUT);

	/** TODO: Уточнить начальное состояние */
	digitalWrite(Pin::MOSFET_pin, LOW);
}

bool BubbleRemover::is_bubble() {
    /**
     *  TODO: Заменить на рабочий код, а не испоьзовать кнопку
     *  Инвертируем состояние, потому что кнопка будет нажиматься
     *  и замыкать на землю, значит GND -> true
     */
    return !digitalRead(Pin::emulator_button_pin);
}

void BubbleRemover::start(Regime& regime_state) {
    /* С помощью MOSFET'а закрываем клапан */
    digitalWrite(Pin::MOSFET_pin, HIGH);

    /* Меняем режим на продувку на минуту */
    regime_state = Regime::REGIME_REMOVE_BUBBLE;

    /* Запускаем таймер на минуту */
    Timer3.restart();
}

void BubbleRemover::stop(Regime& regime_state) {
    /* С помощью MOSFET'а открываем клапан */
    digitalWrite(Pin::MOSFET_pin, LOW);

    /* Останавливаем таймер */
    Timer3.stop();

    /* Возвращаем рабочий режим удержания давления */
    regime_state = Regime::REGIME1;
}