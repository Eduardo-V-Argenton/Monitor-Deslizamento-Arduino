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
        ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(packet<sensors_read>));
        struct packet<sensors_read> pck = *(packet<sensors_read>*) rsc.data;
        printSensorReadings(&pck.data);
    }
    // else{
    //     ResponseStatus lora_response = lora.sendFixedMessage(0,1,64,"Ola mundo");
    //     Serial.println(lora_response.getResponseDescription());
    //     delay(2000);
    // }
}