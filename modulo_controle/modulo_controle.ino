#include <HardwareSerial.h>
#include <LoRa_E220.h> 
#include "extras.h"
#include "reliable_lora.h"

// LoRa
#define UART 2
#define AUX_PIN 18
#define M0_PIN 19
#define M1_PIN 23
#define UART_BPS_RATE UART_BPS_RATE_9600
#define UART_BPS UART_BPS_9600

HardwareSerial hs(UART);
LoRa_E220 lora(&hs, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE);
byte communication_channel = 64;
byte addr[] = {0,2};
byte receptor_addr[] = {0,1};
int send_delay = 5000;

int reliable = 1;


void loadLoraConfig();
byte handshake();
byte recieveSensorsRead(struct Packet<SensorsRead>* pck);
byte waitSensorsRead(struct Packet<SensorsRead>* pck);
void printSensorReads(struct SensorsRead* data);
byte sendLoraConfigPacket(struct Packet<LoRaConfig>* pck);


void menu(){
    Serial.println("rec dados --> 1");
    Serial.println("env loraconfig --> 2");
    Serial.println("--->");
}
void setup() {
    Serial.begin(9600);
    loadLoraConfig();
    lora.setMode(MODE_0_NORMAL);
    menu();
}
 
void loop() {
   
    if(Serial.available() > 0){
        int input = Serial.parseInt();

        if(input == 1){
            struct Packet<SensorsRead> pck;
            recieveSensorsRead(&pck);
        }
        else if(input == 2){
            struct Packet<LoRaConfig> pck;
            sendLoraConfigPacket(&pck);
        }
        Serial.println("");
        Serial.println("============================");
        Serial.println("");
    }
}

byte recieveSensorsRead(struct Packet<SensorsRead>* pck){
    if(reliable == 1){
        if(handshake(1) == 0){
            Serial.println("Erro no handshake");
            return 0;
        }
        Serial.println("Handshake feito");
        Serial.println("");
    }
    if(waitSensorsRead(pck) == 1){
        printSensorReads(&pck->data);
        sendACK(lora, receptor_addr, communication_channel, 1);
    }
    delay(5000);
    return 1;
}

byte handshake(byte OP){
    struct Packet<byte> pck;
    unsigned long startTime = millis();
    sendSYN(lora,receptor_addr, communication_channel, OP);
    while(waitSYNACK(lora, time_out_SYNACK, &pck) == 0){
        if(millis() - startTime >= time_out_handshake){
            return 0;
        }
        else
            sendSYN(lora,receptor_addr, communication_channel, OP);
    }
    sendACK(lora,receptor_addr, communication_channel, OP);
    return 1;
}

byte waitSensorsRead(struct Packet<SensorsRead>* pck){
    unsigned long startTime = millis();
    while (millis() - startTime < timeout_packet){
        if (lora.available()  > 1){
            ResponseStructContainer rsc = lora.receiveMessage(sizeof(Packet<SensorsRead>));
            *pck = *(Packet<SensorsRead>*) rsc.data;
            rsc.close();
            if(serializeData(&pck->data) != pck->Checksum){
                Serial.println("Falha no checksum");
                continue;
            }
            return 1;
        }
    }
    return 0;
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
    configuration.ADDL = addr[1];
    // configuration.TRANSMISSION_MODE.enableLBT = LBT_ENABLED;
    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;

    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
    printParameters(configuration);
    c.close();
}

void printSensorReads(struct SensorsRead* data) {
    Serial.print("Accelerometer: ");
    Serial.print(data->accelerometer[0]);
    Serial.print(", ");
    Serial.print(data->accelerometer[1]);
    Serial.print(", ");
    Serial.println(data->accelerometer[2]);

    Serial.print("Soil Moisture: ");
    Serial.println(data->soil_moisture);

    Serial.print("Air Temperature: ");
    Serial.println(data->air_temperature);

    Serial.print("Air Humidity: ");
    Serial.println(data->air_humidity);

    Serial.print("Rain Sensor Value: ");
    Serial.println(data->rain_sensor_value);
}

void loadLoraConfigPacket(struct Packet<LoRaConfig>* pck){
    pck->data.ADDH = 0;
    pck->data.ADDL = 1;
    pck->data.CHAN = 64;
    pck->data.uart_parity = MODE_00_8N1;
    pck->data.uart_baud_rate = UART_BPS_9600;
    pck->data.air_data_rate = AIR_DATA_RATE_010_24;
    pck->data.sub_packet_option = SPS_200_00;
    pck->data.transmission_power = POWER_22;
    pck->data.enable_RSSI_ambient_noise = 0;
    pck->data.wor_period = WOR_2000_011;
    pck->data.enable_lbt = 0;
    pck->data.enable_rssi = 1;
    pck->data.enable_fixed_transmission = 1;
}

byte sendLoraConfigPacket(struct Packet<LoRaConfig>* pck){
    if(reliable == 1){
        if(handshake(2) == 0){
            Serial.println("Erro no handshake");
            return 0;
        }
        Serial.println("Handshake feito");
        Serial.println("");
    }
    pck->OP = 2;
    loadLoraConfigPacket(pck);
    pck->Checksum = serializeData(&pck->data);
    lora.sendFixedMessage(receptor_addr[0],receptor_addr[1],communication_channel,pck,sizeof(Packet<LoRaConfig>));

    struct Packet<byte> pck_ack;
    unsigned long startTime = millis();
    while(waitACK(lora, time_out_ACK, &pck_ack) == 0){
        if(millis() - startTime >= timeout_packet){
            return 0;
        }
        else{
            lora.sendFixedMessage(receptor_addr[0],receptor_addr[1],communication_channel,pck,sizeof(Packet<LoRaConfig>));
        }
    }
    if(handshake(3) == 0){
        Serial.println("Erro no handshake");
        Serial.println("Restaurando configuração do LoRa");
        return 0;
    }
    Serial.println("Configuração concluida");
    return 1;
}
