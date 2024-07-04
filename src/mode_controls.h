#include <Arduino.h>
#include <display_funcs.h>

#define RADIO_TIMEOUT 2000
#define LORA_TIMEOUT 10000 // 10 seconds

enum UAV_MODES {
    idle, vtol, fixed_wing, failsafe
};

const int failsafe_values[6] = {1009, 1002, 1001, 1016, 1000, 1000};
UAV_MODES uav_mode = UAV_MODES::idle;
uint32_t lastRadioPacket = 0, lastLoraPacket = 0;
UAV_MODES prevMode = UAV_MODES::idle;
bool isRadioOn = true, isLoraOn = true;

void switchToFailSafe() {

    if (uav_mode != UAV_MODES::failsafe) {
        if (!isRadioOn) {
            Serial.println("WARNING: SWITCHING TO FAILSAFE MODE\n\tCAUSE:RADIO COMMUNICATON IS OUT!");
            displayError("SWITCHING TO FAILSAFE MODE, DISCONNECTED RADIO!");
        } else if (!isLoraOn) {
            Serial.println("WARNING: SWITCHING TO FAILSAFE MODE\n\tCAUSE:LORA COMMUNICATON IS OUT!");
            displayError("SWITCHING TO FAILSAFE MODE, DISCONNECTED LORA!");
        }
        prevMode = uav_mode;
        uav_mode = UAV_MODES::failsafe;
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
