#include <Arduino.h>
#include "LoRa_E32.h"
#include <display_funcs.h>


#define LORA_TRANSMITTER_TX_PIN 35
#define LORA_TRANSMITTER_RX_PIN 34
#define LORA_TRANSMITTER_SERIAL Serial8
#define LORA_TRANSMITTER_SERIAL_BPS 115200
#define TRANSMITTER_ADDH 1
#define TRANSMITTER_ADDL 11
#define TRANSMITTER_CHANNEL 6

#define LORA_RECEIVER_TX_PIN 20
#define LORA_RECEIVER_RX_PIN 21
#define LORA_RECEIVER_SERIAL Serial5
#define LORA_RECEIVER_SERIAL_BPS 9600
#define RECEIVER_ADDH 1
#define RECEIVER_ADDL 12
#define RECEIVER_CHANNEL 8

#define CONTROL_STATION_ADDH 1
#define CONTROL_STATION_ADDL 10
#define CONTROL_STATION_CHANNEL 6

LoRa_E32 E32_transmitter(&LORA_TRANSMITTER_SERIAL, UART_BPS_RATE_115200);
LoRa_E32 E32_receiver(&LORA_RECEIVER_SERIAL, UART_BPS_RATE_115200);

void print_E32_parameters(struct Configuration configuration);

void setup_lora() {

  if (!E32_transmitter.begin()) {
    Serial.println("Unable to initialize E32 transmitter!");
    displayError("Unable to initialize E32 transmitter!");
    while (1) {}
  }

  if (!E32_receiver.begin()) {
    Serial.println("Unable to initialize E32 receiver!");
    displayError("Unable to initialize E32 receiver!");
    while (1) {}
  }

  // IF YOU WANT TO SET CONFIGURATION, SET THE UART BAUDRATE TO 9600 AND M0, M1 TO HIGH
  /*ResponseStructContainer c;
  c = E32_receiver.getConfiguration();
  Configuration configuration = *(Configuration*) c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  print_E32_parameters(configuration);
  configuration.CHAN = RECEIVER_CHANNEL;
  configuration.ADDH = RECEIVER_ADDH;
  configuration.ADDL = RECEIVER_ADDL;

  configuration.OPTION.fec = FEC_1_ON;
  configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
  configuration.OPTION.transmissionPower = POWER_20;
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;

  configuration.SPED.airDataRate = AIR_DATA_RATE_101_192;
  configuration.SPED.uartBaudRate = UART_BPS_115200;
  configuration.SPED.uartParity = MODE_00_8N1;

  ResponseStatus rs = E32_receiver.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);
  c.close();
  print_E32_parameters(configuration);
  */

}

void print_E32_parameters(struct Configuration configuration) {
	Serial.println("----------------------------------------");

	Serial.print(F("HEAD : "));  Serial.print(configuration.HEAD, BIN);Serial.print(" ");Serial.print(configuration.HEAD, DEC);Serial.print(" ");Serial.println(configuration.HEAD, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, BIN);
	Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, BIN);
	Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDatte  : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRate());
	Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRate());

	Serial.print(F("OptionTrans        : "));  Serial.print(configuration.OPTION.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFixedTransmissionDescription());
	Serial.print(F("OptionPullup       : "));  Serial.print(configuration.OPTION.ioDriveMode, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getIODroveModeDescription());
	Serial.print(F("OptionWakeup       : "));  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
	Serial.print(F("OptionFEC          : "));  Serial.print(configuration.OPTION.fec, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFECDescription());
	Serial.print(F("OptionPower        : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());

	Serial.println("----------------------------------------");

}