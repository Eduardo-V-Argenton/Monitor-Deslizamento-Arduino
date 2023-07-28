#include <HardwareSerial.h>
#include <LoRa_E220.h> 
#include "extras.h"

// LoRa
#define UART 2
#define AUX_PIN 18
#define M0_PIN 19
#define M1_PIN 23
#define UART_BPS_RATE UART_BPS_RATE_9600
#define UART_BPS UART_BPS_9600
#define COMMUNICATION_CHANNEL 64
#define HIGH_ADDRESS_BYTE 0
#define LOW_ADDRESS_BYTE 2

HardwareSerial hs(UART);
LoRa_E220 lora(&hs, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE);

void loraConfig(){
  lora.begin();
  ResponseStructContainer c;
	c = lora.getConfiguration();
	Configuration configuration = *(Configuration*) c.data;

  configuration.CHAN = COMMUNICATION_CHANNEL; // Communication channel
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.SPED.uartBaudRate = UART_BPS; // Serial baud rate
  configuration.ADDH = HIGH_ADDRESS_BYTE;
  configuration.ADDL = LOW_ADDRESS_BYTE;
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;

  ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
  printParameters(configuration);
  c.close();
}

void setup() {
  Serial.begin(9600);
  loraConfig();
}
 
void loop() {
  if (lora.available()  > 1){
    ResponseContainer rs = lora.receiveMessageRSSI();
    String message = rs.data; // First ever get the data
    Serial.println(rs.status.getResponseDescription());
    Serial.println(rs.rssi);
    Serial.println(message);
  }
}