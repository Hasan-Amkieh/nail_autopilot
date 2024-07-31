#include <Arduino.h>
#include <TeensyTimerTool.h>

class SignalGen {
private: 
    SignalGen();

    const int pin;
    float dutyCycle;
    uint32_t period;
    uint32_t highTime;
    uint32_t lowTime;
    volatile bool state;
    TeensyTimerTool::PeriodicTimer timer;

public:
    SignalGen(const int pin) : dutyCycle(0), period(20000), highTime(0), lowTime(0), state(LOW), pin(pin) {}

    void begin(float frequency) {
        pinMode(pin, OUTPUT);
        digitalWriteFast(pin, LOW);

        period = 1'000'000 / frequency; // Period in microseconds
        highTime = dutyCycle * 1000 + 1000; // between 1ms and 2ms
        lowTime = period - highTime;
        timer.begin([this] { this->onTimer(); }, highTime); // Start with the highTime interval
    }

    // Duty range is 0.0 - 1.0
    void setDutyCycle(float dutyCycle) {
        this->dutyCycle = dutyCycle;
        highTime = dutyCycle * 1000 + 1000; // between 1ms and 2ms
        lowTime = period - highTime;
        if (state == HIGH) {
            timer.setPeriod(highTime); // Update timer interval based on new highTime
        } else {
            timer.setPeriod(lowTime); // Update timer interval based on new lowTime
        }
    }

private:

    void onTimer() {
        state = !state;
        digitalWriteFast(pin, state);
        timer.setPeriod(state ? highTime : lowTime); // Update timer interval for the next state
    }
};