#include <HardwareSerial.h>
#include "LoRa_E220.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL345_U.h"
#include "DHT.h"
#include "extras.h"

// LoRa
#define UART 2
#define AUX_PIN 18
#define M0_PIN 32
#define M1_PIN 19
#define UART_BPS_RATE UART_BPS_RATE_9600
#define UART_BPS UART_BPS_9600
#define COMMUNICATION_CHANNEL 64
#define HIGH_ADDRESS_BYTE 0
#define LOW_ADDRESS_BYTE 1
#define RECEIVER_HIGH_ADDRESS_BYTE 0
#define RECEIVER_LOW_ADDRESS_BYTE 2
#define SEND_DELAY 5000

// Sensors
#define CAP_SOIL_PIN 34
#define RAIN_SENSOR_PIN 35
#define DHT_PIN 4 
#define DHT_TYPE DHT11

DHT dht(DHT_PIN, DHT_TYPE);
HardwareSerial hs(UART);
LoRa_E220 lora(&hs, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


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
  configuration.TRANSMISSION_MODE.enableLBT = LBT_ENABLED;

  ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
  printParameters(configuration);
  c.close();
}

void setup() {
  //LoRa
  Serial.begin(9600);
  loraConfig();

  //DHT
  dht.begin();

  //ADXL
  if(!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }   
}

void build_message(){
  int css = analogRead(CAP_SOIL_PIN);
  int rs = analogRead(RAIN_SENSOR_PIN);
  int humidity = dht.readHumidity();
  int temperature = dht.readTemperature();
  sensors_event_t event; 
  accel.getEvent(&event);
 
  String message = 
  String(event.acceleration.x) + ";" +
  String(event.acceleration.y) + ";" +
  String(event.acceleration.z) + ";" +
  String(humidity) + ";" +
  String(temperature) + ";" +
  String(css) + ";" +
  String(rs);

  return message;
}
 
void loop() {

  String message = build_message();
  Serial.println(message);
  ResponseStatus lora_response = lora.sendFixedMessage(
    RECEIVER_HIGH_ADDRESS_BYTE,RECEIVER_LOW_ADDRESS_BYTE,COMMUNICATION_CHANNEL,message);
	Serial.println(lora_response.getResponseDescription());
  delay(SEND_DELAY);
}