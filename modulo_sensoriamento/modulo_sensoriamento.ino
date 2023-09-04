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
void loadDefaultLoraConfig();
void startDHT();
void startAccel();
void loadSensorRead(struct Packet<SensorsRead>* pck);
void sendPacket(struct Packet<SensorsRead>* pck);
byte recieveLoraConfig(struct Packet<LoRaConfig>* pck);
byte waitLoRaPacket(struct Packet<LoRaConfig>* pck);
byte loadLoRaConfigFromPacket(struct LoRaConfig* lc);
byte restoreLoRaConfigFromPacket();

void setup() {
    Serial.begin(9600);
    loadDefaultLoraConfig();
    startDHT();
    startAccel();
    lora.setMode(MODE_0_NORMAL);  
}
 
void loop() {
    byte OP = 0;
    int handshake_return = receptorHandshake(lora, &OP);
    if(handshake_return == -1){
        Serial.println("Erro no handshake");
        return;
    }else if(handshake_return == 1){
        Serial.println("Handshake feito");
    }
    else{return;}

    if(OP == 0){
        struct Packet<LoRaConfig> pck;
        recieveLoraConfig(&pck);
    }
    else if(OP == 2){
        SendSensorsRead();
    }
}

byte SendSensorsRead(){

    struct Packet<SensorsRead> pck;
    struct Packet<byte> pck_ack;
    unsigned long startTime = millis();
    loadSensorRead(&pck);
    pck.OP = 1;
    sendSensorsRead(&pck);
    Serial.println("Pacote com leitura dos sensores foi enviado");
    while(waitACK(lora, &pck_ack) == 0){
        if(millis() - startTime >= timeOutSensorsReadPacket)
            return 0;
        else
            sendSensorsRead(&pck);
    }
    Serial.println("Pacote foi recebido pelo destinatario");
    return 1;
}

void createAndSendSensorReadPacket(){
    struct Packet<SensorsRead> pck;
    loadSensorRead(&pck);
    pck.OP = 1;
    sendPacket(&pck);
}

byte sendSensorsRead(struct Packet<SensorsRead>* pck){
    String string_lora_config = stringifySensorsRead(&pck->data);
    pck->Checksum = crc16_ccitt(reinterpret_cast<const uint8_t *>(string_lora_config.c_str()), string_lora_config.length());
    ResponseStatus lora_response = lora.sendFixedMessage(receptorAddr[0],receptorAddr[1],channel,pck,sizeof(Packet<SensorsRead>));

    return lora_response.code;
}

void loadDefaultLoraConfig(){
    senderAddr[0] = 10;
    senderAddr[1] = 10;
    channel = 64;

    lora.begin();
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    configuration.CHAN = channel;
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;
    configuration.SPED.uartBaudRate = UART_BPS_9600;
    configuration.SPED.uartParity = MODE_00_8N1; 
    configuration.ADDH = senderAddr[0];
    configuration.ADDL = senderAddr[1];
    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;

    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
    printParameters(configuration);
    c.close();
}

void startDHT(){
    dht.begin();
    Serial.println("DHT inicializado");
}

void startAccel(){
    if(!accel.begin())
    {
        Serial.println("ADXL345 Error");
        while(1);
    }
    Serial.println("ADXL345 inicializado");   
}

void loadSensorRead(struct Packet<SensorsRead>* pck){
    sensors_event_t event; 
    accel.getEvent(&event);

    pck->data.accelerometer[0] = event.acceleration.x;
    pck->data.accelerometer[1] = event.acceleration.y;
    pck->data.accelerometer[2] = event.acceleration.z;
    pck->data.airHumidity = dht.readHumidity();
    pck->data.airTemperature = dht.readTemperature();
    pck->data.soilMoisture = analogRead(CAP_SOIL_PIN);
    pck->data.rainSensorValue = analogRead(RAIN_SENSOR_PIN);
}

byte loadLoRaConfigFromPacket(struct LoRaConfig* lc){
    timeOutConfigPacket = lc->timeOutConfigPacket*1000;
    timeOutSensorsReadPacket = lc->timeOutSensorsReadPacket*1000;
    timeOutHandshake = lc->timeOutHandshake*1000;
    timeOutSYNACK = lc->timeOutSYNACK*1000;
    timeOutACK = lc->timeOutACK*1000;

    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    configuration_backup = configuration;

    configuration.CHAN = lc->CHAN;
    configuration.SPED.airDataRate = lc->airDataRate;
    configuration.OPTION.transmissionPower = lc->transmissionPower;
    configuration.TRANSMISSION_MODE.WORPeriod = lc->WORPeriod;
    configuration.TRANSMISSION_MODE.enableLBT = lc->enableLBT;

    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
    Serial.println("    Nova Configuração do LoRa   ");
    printParameters(configuration);
    c.close();
    Serial.println("Time out pacote de configuração: " + String(timeOutConfigPacket));
    Serial.println("Time out pacote leitura de sensores: " + String(timeOutSensorsReadPacket));
    Serial.println("Time out handshake: " + String(timeOutHandshake));
    Serial.println("Time out SYNACK: " + String(timeOutSYNACK));
    Serial.println("Time out ACK: " + String(timeOutACK));
    Serial.println("----------------------------------------");
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
    while (millis() - startTime < timeOutConfigPacket){
        if (lora.available()  > 1){
            ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(Packet<LoRaConfig>));
            *pck = *(Packet<LoRaConfig>*) rsc.data;
            rsc.close();
            String string_lora_config = stringifyLoraConfig(&pck->data);
            uint16_t genCRC = crc16_ccitt(reinterpret_cast<const uint8_t *>(string_lora_config.c_str()), string_lora_config.length());
            Serial.println("CRC Dado = " + String(pck->Checksum));
            Serial.println("CRC Gerado = " + String(genCRC));
            if(genCRC != pck->Checksum){
                Serial.println("Falha no checksum");
                continue;
            }
            sendACK(lora,channel, 2);
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
        sendACK(lora, channel, 3);
    }
    byte OP = 0;
    if(receptorHandshake(lora, &OP) == 0){
        Serial.println("Erro no handshake");
        Serial.println("Restaurando configuração do LoRa");
        restoreLoRaConfigFromPacket();
        return 0;
    }else{
        Serial.println("configuração do LoRa concluida");
    }
    return 1;
}