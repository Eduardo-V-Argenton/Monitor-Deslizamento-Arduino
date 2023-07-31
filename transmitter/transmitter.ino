#include <HardwareSerial.h>
#include <LoRa_E220.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <DHT.h>

#include "extras.h"

// LoRa
#define UART 2
#define AUX_PIN 18
#define M0_PIN 32
#define M1_PIN 19
#define UART_BPS_RATE UART_BPS_RATE_9600
#define UART_BPS UART_BPS_9600
HardwareSerial hs(UART);
LoRa_E220 lora(&hs, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE);
int communication_channel = 64;
int addr[] = {0,1};
int receptor_addr[] = {0,2};
int send_delay = 5000;

// Sensors
#define CAP_SOIL_PIN 34
#define RAIN_SENSOR_PIN 35
#define DHT_PIN 4 
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


void createAndSendSensorReadPacket();
void loraConfig();
void startDHT();
void startAccel();
void loadSensorRead(struct packet<sensors_read>* pck);
void loadPacketOptions(struct packet<sensors_read>* pck, byte OP, byte ACK = 0, bool USEACK = false, bool URG = false, bool SYN = false, bool BRC = false);
void sendPacket(struct packet<sensors_read>* pck);

void setup() {
    Serial.begin(9600);
    loadLoraConfig();
    startDHT();
    startAccel();
}
 
void loop() {
    createAndSendSensorReadPacket();
}


void createAndSendSensorReadPacket(){
    struct packet<sensors_read> pck;
    loadSensorRead(&pck);
    loadPacketOptions(&pck, 1);
    sendPacket(&pck);
    delay(2000);    
}

void sendPacket(struct packet<sensors_read>* pck){
    ResponseStatus lora_response = lora.sendFixedMessage(receptor_addr[0],receptor_addr[1],communication_channel,pck,sizeof(packet<sensors_read>));
    Serial.println(lora_response.getResponseDescription());
}

// int waitACK(){
//     unsigned long startTime = millis();

//     while (millis() - startTime < 10000){
//         if (lora.available()  > 1){
//             ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(packet));
//             struct packet pck =  *(struct packet*) rsc.data;
//             if(pck.ACK == 1){
//                 return 1;
//             }
//             rsc.close();
//         }
//     }
//     return 0;
// }

void loadLoraConfig(){
    lora.begin();
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;

    configuration.CHAN = communication_channel; // Communication channel
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    configuration.SPED.uartBaudRate = UART_BPS; // Serial baud rate
    configuration.ADDH = addr[0];
    configuration.ADDL = addr[1];
    // configuration.TRANSMISSION_MODE.enableLBT = LBT_ENABLED;
    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;

    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
    printParameters(configuration);
    c.close();
}

void startDHT(){
    dht.begin();
}

void startAccel(){
    if(!accel.begin())
    {
        Serial.println("ADXL345 Error");
        while(1);
    }   
}

void loadSensorRead(struct packet<sensors_read>* pck){
    sensors_event_t event; 
    accel.getEvent(&event);

    pck->data.accelerometer[0] = event.acceleration.x;
    pck->data.accelerometer[1] = event.acceleration.y;
    pck->data.accelerometer[2] = event.acceleration.z;
    pck->data.air_humidity = dht.readHumidity();
    pck->data.air_temperature = dht.readTemperature();
    pck->data.soil_humidity = analogRead(CAP_SOIL_PIN);
    pck->data.rain_sensor_value = analogRead(RAIN_SENSOR_PIN);
}

void loadPacketOptions(struct packet<sensors_read>* pck, byte OP, byte ACK, bool USEACK, bool URG, bool SYN , bool BRC){
    pck->OP = OP;
    pck->ACK = ACK;
    pck->USEACK= USEACK;
    pck->SYN = SYN;
    pck->URG = URG;
    pck->BRC = BRC;
}
