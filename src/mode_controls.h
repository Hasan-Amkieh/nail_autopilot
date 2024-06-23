#include <Arduino.h>
#include <display_funcs.h>

enum UAV_MODES {
    idle, vtol, fixed_wing, failsafe
};

const int failsafe_values[6] = {1009, 1002, 1001, 1016, 1000, 1000};
UAV_MODES uav_mode = UAV_MODES::idle;
uint32_t lastRadioPacket = 0;
UAV_MODES prevMode = UAV_MODES::idle;

void switchToFailSafe() {

    if (uav_mode != UAV_MODES::failsafe) {
        Serial.println("WARNING: SWITCHING TO FAILSAFE MODE\n\tCAUSE:RADIO COMMUNICATON IS OUT!");
        displayError("SWITCHING TO FAILSAFE MODE, DISCONNECTED RADIO COMMUNICATON!");
        prevMode = uav_mode;
        uav_mode = UAV_MODES::failsafe;
    }

}

void exitFailSafeMode() {

    if (uav_mode == UAV_MODES::failsafe) {
        uav_mode = prevMode;
    }

}

bool isRadioRunning(uint16_t* channels) {

    if ((millis() - lastRadioPacket) >= 2000) {
        return false;
    }

    for (int i = 0 ; i < 6 ; i++) {
        if (failsafe_values[i] != channels[i]) {
            return true;
        }
    }

    return false;

}

bool isFlying() {

    return uav_mode == UAV_MODES::vtol || uav_mode == UAV_MODES::fixed_wing;

}
