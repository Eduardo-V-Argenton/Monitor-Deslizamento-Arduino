#include <HardwareSerial.h>
#include <LoRa_E220.h> 
#include "extras.h"
#include "reliable_lora.h"
#include <WiFi.h>
#include <HTTPClient.h>

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
byte receptor_addr[2];
int read_commands_period = 3000;

const char* ssid = "Eduardo Net";
const char* password = "edu19126";
const char* commands_url = "http://192.168.1.108:8081/communication/commands/";
const char* response_url = "http://192.168.1.108:8081/communication/response/";
const char* data_url = "http://192.168.1.108:8081/communication/data/";



String key = "6dc8a0fb";

void loadLoraConfig();
byte handshake();
byte recieveSensorsRead(struct Packet<SensorsRead>* pck);
byte waitSensorsRead();
void printSensorReads(struct SensorsRead* data);
byte sendLoraConfigPacket(int* config);
void connectWiFi();
String getCommands();
byte loadConfig(int* config);
byte checkCRCFromWeb(const char* input, String crc);
void sendResponse(int result);
byte verifyCommand(String* command);
void printLoRaConfig();


void setup() {
    Serial.begin(9600);
    connectWiFi();
    printLoRaConfig();
    lora.setMode(MODE_1_WOR_TRANSMITTER);

}
 
void loop() {
    String command = getCommands();
    if(command != "0"){
        byte op = verifyCommand(&command);
        if(op == -1){
            sendResponse(0);
        }

        int configSize = (op == 1) ? 6 : 20;
        int* config = new int[configSize];
        parseCommandToConfig(command.substring(2).c_str(), config);
        if(checkCRCFromWeb(command, String(config[configSize - 1])) == 0){
            //ToDO Error
            Serial.println("checksum Inválido");
        }else{
            if(op == 0){
                int wasCorrectlyLoadedForCommunication = loadConfigForCommunication(config, op);
                if(wasCorrectlyLoadedForCommunication == 1){
                    sendResponse(sendLoraConfigPacket(config));
                }else{
                    //ToDO Error
                    sendResponse(wasCorrectlyLoadedForCommunication);
                }
            }else if(op == 1){
                sendResponse(loadConfig(config));

            }else{
                sendResponse(waitSensorsRead());
            }
        }
        delete[] config;
    }
    delay(read_commands_period);
}

byte verifyCommand(String* command){
    if (command->length() <= 0) {
        return -1;
    }

    char firstChar = command->charAt(0);
    if(!isDigit(firstChar)){
        //ToDO Error
        return -1;
    }

    int op = firstChar - '0';
    if (op < 0 || op > 2) {
        //ToDO Error
        return -1;
    }
    return op;
}

byte recieveSensorsRead(struct Packet<SensorsRead>* pck){
    if(handshake(1) == 0){
        Serial.println("Erro no handshake");
        return 0;
    }
    Serial.println("Handshake feito");
    Serial.println("");
    
    if(waitSensorsRead() == 1){
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

byte waitSensorsRead(){
    struct Packet<SensorsRead> pck;
    unsigned long startTime = millis();
    while (millis() - startTime < timeout_packet){
        if (lora.available()  > 1){
            ResponseStructContainer rsc = lora.receiveMessage(sizeof(Packet<SensorsRead>));
            pck = *(Packet<SensorsRead>*) rsc.data;
            rsc.close();
            String string_sr = stringifySensorsRead(&pck.data); 
            const uint8_t *data = reinterpret_cast<const uint8_t *>(string_sr.c_str());
            uint16_t genCRC = crc16_ccitt(data, string_sr.length());
            if(genCRC != pck.Checksum){
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

    configuration.CHAN = 64;
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    configuration.SPED.uartBaudRate = UART_BPS;
    configuration.ADDH = 0x0;
    configuration.ADDL = 0x1;
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

void loadLoraConfigPacket(struct Packet<LoRaConfig>* pck, int* config){
    pck->data.ADDH = config[0];
    pck->data.ADDL = config[1];
    pck->data.CHAN = config[2];
    pck->data.air_data_rate = config[3];
    pck->data.transmission_power = config[4];
    pck->data.enable_RSSI_ambient_noise = config[5];
    pck->data.wor_period = config[6];
    pck->data.enable_lbt = config[7];
    pck->data.enable_rssi = config[8];
}

byte sendLoraConfigPacket(int* config){
    struct Packet<LoRaConfig> pck;
    if(handshake(2) == 0){
        Serial.println("Erro no handshake");
        return 0;
        Serial.println("Handshake feito");
        Serial.println("");
    }
    pck.OP = 0;
    loadLoraConfigPacket(&pck, config);
    String string_lora_config = stringifyLoraConfig(&pck.data);
    pck.Checksum = crc16_ccitt(reinterpret_cast<const uint8_t *>(string_lora_config.c_str()), string_lora_config.length());
    lora.sendFixedMessage(receptor_addr[0],receptor_addr[1],communication_channel,&pck,sizeof(Packet<LoRaConfig>));

    struct Packet<byte> pck_ack;
    unsigned long startTime = millis();
    while(waitACK(lora, time_out_ACK, &pck_ack) == 0){
        if(millis() - startTime >= timeout_packet){
            return 0;
        }
        else{
            lora.sendFixedMessage(receptor_addr[0],receptor_addr[1],communication_channel,&pck,sizeof(Packet<LoRaConfig>));
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

void connectWiFi(){
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
}

String getCommands(){
    String extractedContent = " ";
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;

        http.begin(commands_url);

        int httpCode = http.GET();
        if (httpCode > 0) {
            if (httpCode == HTTP_CODE_OK) {
                String payload = http.getString();
                if(payload.length() <= 9){return "0";}
                int startPos = payload.indexOf("<p>");
                if (startPos != -1) {
                    startPos = payload.indexOf(">", startPos) + 1;
                    int endPos = payload.indexOf("</p>", startPos);
                    if (endPos != -1) {
                        extractedContent = payload.substring(startPos, endPos);
                        Serial.println("Valor Extraido Web Page = " + extractedContent);
                    } 
                    else {
                        Serial.println("Erro na obtenção do comando, não ha a tag </p>");
                        return "-1";
                    }
                }
                else {
                    Serial.println("Erro na obtenção do comando, não ha a tag <p>");
                    return "-1";
                }
            }
            else {
                Serial.printf("HTTP request failed with error code: %d\n", httpCode);
                return http.errorToString(httpCode).c_str();
            }
        }
        http.end();

    }
    return extractedContent;
}

byte loadConfig(int* config){
    read_commands_period = config[4]*1000;
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;

    configuration.ADDH = config[0];
    configuration.ADDL = config[1];
    configuration.OPTION.transmissionPower = config[2];
    configuration.TRANSMISSION_MODE.enableLBT = config[3];

    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    Serial.println("    Nova Configuração do LoRa   ");
    printParameters(configuration);
    c.close();
    return rs.code;
}

byte checkCRCFromWeb(String input, String crc){
    int lastIndex = input.lastIndexOf(';');
  
    if (lastIndex == -1){return -1;}
    String input_without_crc = input.substring(0, lastIndex);
    Serial.println("Comando = " + input_without_crc);
    const uint8_t *data = reinterpret_cast<const uint8_t *>(input_without_crc.c_str());
    uint16_t genCRC = crc16_ccitt(data, input_without_crc.length());
    Serial.println("CRC Dado = " + crc);
    Serial.println("CRC Gerado = " + String(genCRC));
    if( String(genCRC) == crc){return 1;}else{return 0;}
}

void sendResponse(int result){
    HTTPClient http;
    http.begin(response_url);

    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String postData = "key="+key+"&response="+result;
    int httpResponseCode = http.POST(postData);
    Serial.println("Resposta enviada = " + postData);
    if(httpResponseCode <= 0){
        Serial.print("Error in POST request. HTTP Response code: ");
        Serial.println(httpResponseCode);
    }
    http.end();
}

byte loadConfigForCommunication(int* config, byte op){
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;

    configuration.CHAN = config[2];
    configuration.SPED.airDataRate = config[3];
    if(op == 0){
        configuration.OPTION.RSSIAmbientNoise = config[5];
        configuration.TRANSMISSION_MODE.WORPeriod = config[6];
        configuration.TRANSMISSION_MODE.enableRSSI = config[8];
    }else{
        configuration.OPTION.RSSIAmbientNoise = config[4];
        configuration.TRANSMISSION_MODE.WORPeriod = config[5];
        configuration.TRANSMISSION_MODE.enableRSSI = config[6];
    }

    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
    printParameters(configuration);
    c.close();
    return rs.code;
}

void printLoRaConfig(){
    lora.begin();
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    printParameters(configuration);
    c.close();
}