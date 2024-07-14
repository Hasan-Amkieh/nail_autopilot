#include <Arduino.h>
#include <display_funcs.h>

#define RADIO_TIMEOUT 4000
#define LORA_TIMEOUT 10000
#define GPS_TIMEOUT 3000
#define BATTERY_VOLTAGE_MINIMUM 20.0 // which is equivalent to 3.3 volts per cell, around 15% remaining, as the stepper motor driver requires 20V minimum

enum UAV_MODES {
    idle, vtol, fixed_wing, failsafe
};

const int failsafe_values[6] = {1009, 1002, 1001, 1016, 1000, 1000};
UAV_MODES uav_mode = UAV_MODES::idle;
uint32_t lastRadioPacket = 0, lastLoraPacket = 0, lastGPSPacket = 0;
UAV_MODES prevMode = UAV_MODES::idle;
bool isRadioOn = true, isLoraOn = true, isGPSOn = true, isBattVoltLow = false;
bool isManual = true;

float batt_voltage = 0;

void switchToFailSafe() {

    if (uav_mode != UAV_MODES::failsafe) {
        if (!isRadioOn) {
            Serial.println("WARNING: Switching to failsafe mode\n\tRadio communication is lost!");
            displayError("Switching to failsafe mode, disconnected radio!");
        } else if (!isLoraOn) {
            Serial.println("WARNING: Switching to failsafe mode\n\tLORA communication is lost!");
            displayError("Switching to failsafe mode, disconnected LORA!");
        } else if (!isGPSOn) {
            Serial.println("WARNING: Switching to failsafe mode\n\tNo GPS signal!");
            displayError("Switching to failsafe mode, no GPS signal!");
        } else if (isBattVoltLow) {
            Serial.println("WARNING: Switching to failsafe mode\n\tBattery voltage is LOW!");
            displayError("Switching to failsafe mode, Battery voltage is LOW!");
        }
        prevMode = uav_mode;
        uav_mode = UAV_MODES::failsafe;
    }

}

bool isGPSRunning() {

    isGPSOn = (millis() - lastGPSPacket) < GPS_TIMEOUT;
    return isGPSOn;

}

void updateFailSafeMessage() {

    if (uav_mode == UAV_MODES::failsafe) {
        if (!isRadioOn) {
            displayError("Switching to failsafe mode, disconnected RADIO!");
        } else if (!isLoraOn) {
            displayError("Switching to failsafe mode, disconnected LORA!");
        } else if (!isGPSOn) {
            displayError("Switching to failsafe mode, no GPS signal!");
        } else if (isBattVoltLow) {
            displayError("Switching to failsafe mode, Battery voltage is LOW!");
        }
    }

}

void exitFailSafeMode() {

    if (uav_mode == UAV_MODES::failsafe) {
        uav_mode = prevMode;
    }

}

bool isLoraRunning() {

    isLoraOn = (millis() - lastLoraPacket) < LORA_TIMEOUT;
    return isLoraOn;

}

bool isBatteryVoltageLow() {
    
    isBattVoltLow = batt_voltage < BATTERY_VOLTAGE_MINIMUM;
    return isBattVoltLow;

}

bool isRadioRunning(uint16_t* channels) {

    if ((millis() - lastRadioPacket) >= RADIO_TIMEOUT) {
        isRadioOn = false;
        return false;
    }

    for (int i = 0 ; i < 6 ; i++) {
        if (failsafe_values[i] != channels[i]) {
            isRadioOn = true;
            return true;
        }
    }

    isRadioOn = false;
    return false;

}

bool isFlying() {

    return uav_mode == UAV_MODES::vtol || uav_mode == UAV_MODES::fixed_wing;

}
