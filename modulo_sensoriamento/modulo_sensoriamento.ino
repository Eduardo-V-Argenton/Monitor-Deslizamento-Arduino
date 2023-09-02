#include <HardwareSerial.h>
#include <LoRa_E220.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <DHT.h>
#include "extras.h"
#include "reliable_lora.h"


// LoRa
#define UART 2
#define AUX_PIN 18
#define M0_PIN 32
#define M1_PIN 19
#define UART_BPS_RATE UART_BPS_RATE_9600
#define UART_BPS UART_BPS_9600
HardwareSerial hs(UART);
LoRa_E220 lora(&hs, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE);
byte communication_channel = 64;
byte addr[] = {0,1};
byte receptor_addr[] = {0,2};

int send_delay = 5000;
int reliable = 1;

Configuration configuration_backup;

// Sensors
#define CAP_SOIL_PIN 34
#define RAIN_SENSOR_PIN 35
#define DHT_PIN 4 
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
int cap_soil_air = 2600;
int cap_soil_water = 700; 

void createAndSendSensorReadPacket();
void loraConfig();
void startDHT();
void startAccel();
void loadSensorRead(struct Packet<SensorsRead>* pck);
void sendPacket(struct Packet<SensorsRead>* pck);
byte handshake();
byte recieveLoraConfig(struct Packet<LoRaConfig>* pck);
byte waitLoRaPacket(struct Packet<LoRaConfig>* pck);
byte loadLoRaConfigFromPacket(struct LoRaConfig* lc);
byte restoreLoRaConfigFromPacket();

void setup() {
    Serial.begin(9600);
    loadLoraConfig();
    startDHT();
    startAccel();
    lora.setMode(MODE_0_NORMAL);
}
 
void loop() {
    byte OP = 0;
    if(reliable == 1){
        if(handshake(&OP) == 0){
            Serial.println("Erro no handshake");
        }else{
            Serial.println("Handshake feito");
        }
    }else{
        SendSensorsRead();
    }
    if(OP == 1){
        SendSensorsRead();
    }
    else if(OP == 2){
        struct Packet<LoRaConfig> pck;
        recieveLoraConfig(&pck);
    }
}

byte SendSensorsRead(){

    struct Packet<SensorsRead> pck;
    struct Packet<byte> pck2;
    unsigned long startTime = millis();
    loadSensorRead(&pck);
    pck.OP = 1;
    sendSensorsRead(&pck);
    while(waitACK(lora, time_out_ACK, &pck2) == 0){
        if(millis() - startTime >= timeout_packet)
            return 0;
        else
            sendSensorsRead(&pck);
    }
    return 1;
    delay(5000);
}

byte handshake(byte* OP){
    struct Packet<byte> pck;
    unsigned long startTime = millis();
    while(waitSYN(lora, &pck, OP) == 0){
        if(millis() - startTime >= time_out_handshake){return 0;}
    }
    sendSYNACK(lora, receptor_addr, communication_channel, 1);
    while(waitACK(lora, time_out_ACK, &pck) == 0){
        if(millis() - startTime >= time_out_handshake)
            return 0;
        else
            sendSYNACK(lora, receptor_addr, communication_channel, 1);
    }
    return 1;
}


void createAndSendSensorReadPacket(){
    struct Packet<SensorsRead> pck;
    loadSensorRead(&pck);
    pck.OP = 1;
    sendPacket(&pck);
    delay(2000);    
}

byte sendSensorsRead(struct Packet<SensorsRead>* pck){
    pck->Checksum = serializeData(&pck->data);
    ResponseStatus lora_response = lora.sendFixedMessage(receptor_addr[0],receptor_addr[1],communication_channel,pck,sizeof(Packet<SensorsRead>));

    return lora_response.code;
}

void loadLoraConfig(){
    lora.begin();
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;

    configuration.CHAN = communication_channel; // Communication channel
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    configuration.SPED.uartBaudRate = UART_BPS; // Serial baud rate
    configuration.ADDH = addr[0];
    configuration.ADDL = addr[2];
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

void loadSensorRead(struct Packet<SensorsRead>* pck){
    sensors_event_t event; 
    accel.getEvent(&event);

    pck->data.accelerometer[0] = event.acceleration.x;
    pck->data.accelerometer[1] = event.acceleration.y;
    pck->data.accelerometer[2] = event.acceleration.z;
    pck->data.air_humidity = dht.readHumidity();
    pck->data.air_temperature = dht.readTemperature();
    pck->data.soil_moisture = analogRead(CAP_SOIL_PIN);
    pck->data.rain_sensor_value = analogRead(RAIN_SENSOR_PIN);
}

byte loadLoRaConfigFromPacket(struct LoRaConfig* lc){
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    configuration_backup = configuration;

    configuration.ADDH = lc->ADDH;
    configuration.ADDL = lc->ADDL;
    configuration.CHAN = lc->CHAN;
    configuration.SPED.airDataRate = lc->air_data_rate;
    configuration.OPTION.transmissionPower = lc->transmission_power;
    configuration.OPTION.RSSIAmbientNoise = lc->enable_RSSI_ambient_noise;
    configuration.TRANSMISSION_MODE.WORPeriod = lc->wor_period;
    configuration.TRANSMISSION_MODE.enableLBT = lc->enable_lbt;
    configuration.TRANSMISSION_MODE.enableRSSI = lc->enable_rssi;

    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
    printParameters(configuration);
    c.close();
    return rs.code;
}

byte restoreLoRaConfigFromPacket(){
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = configuration_backup;
    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
    printParameters(configuration);
    c.close();
    return rs.code;    
}

byte waitLoRaPacket(struct Packet<LoRaConfig>* pck){
    unsigned long startTime = millis();
    while (millis() - startTime < timeout_packet){
        if (lora.available()  > 1){
            ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(Packet<LoRaConfig>));
            *pck = *(Packet<LoRaConfig>*) rsc.data;
            rsc.close();
            if(serializeData(&pck->data) != pck->Checksum){
                Serial.println(pck->Checksum);
                Serial.println(serializeData(&pck->data));
                Serial.println("Falha no checksum");
                continue;
            }
            sendACK(lora, receptor_addr, communication_channel, 2);
            return 1;
        }
    }
    return 0;
}

byte recieveLoraConfig(struct Packet<LoRaConfig>* pck){
    if(waitLoRaPacket(pck) == 0){
        return 0;
    }
    else if(loadLoRaConfigFromPacket(&pck->data) == 1){
        sendACK(lora, receptor_addr, communication_channel, 3);
    }
    byte OP = 0;
    if(handshake(&OP) == 0){
        Serial.println("Erro no handshake");
        Serial.println("Restaurando configuração do LoRa");
        restoreLoRaConfigFromPacket();
        return 0;
    }else{
        Serial.println("configuração do LoRa concluida");
    }
    return 1;
}