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

int read_commands_period = 3000;

const char* ssid = "Eduardo Net";
const char* password = "edu19126";
const char* commands_url = "http://192.168.1.108:8081/communication/commands/";
const char* response_url = "http://192.168.1.108:8081/communication/response/";
const char* data_url = "http://192.168.1.108:8081/communication/data/";

String key = "6dc8a0fb";

void loadLoraConfig();
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
void loadDefaultLoraConfig();

void setup() {
    Serial.begin(9600);
    connectWiFi();
    loadDefaultLoraConfig();
    lora.setMode(MODE_0_NORMAL);  
}
 
void loop() {
    String command = getCommands();
    if(command == "-1"){return;}
    else if(command != "0"){
        byte op = verifyCommand(&command);
        if(op == -1){
            sendResponse(-1);
            return;
        }

        int configSize = (op == 1) ? 6 : 29;
        int* config = new int[configSize];
        parseCommandToConfig(command.substring(2).c_str(), config);
        if(checkCRCFromWeb(command, String(config[configSize - 1])) == 0){
            Serial.println("checksum Inválido");
            sendResponse(-1);
            return;
        }else{
            if(op == 0){
                int wasCorrectlyLoadedForCommunication = loadConfigForCommunication(config, op);
                if(wasCorrectlyLoadedForCommunication == 1){
                    sendResponse(sendLoraConfigPacket(config));
                }else{
                    Serial.println("Erro no carregamento das informações para comunicação com o módulo de sensoriamento");
                    sendResponse(-1);
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
        Serial.println("Erro: Primeiro caracter da mensagem não é um digito");
        return -1;
    }

    int op = firstChar - '0';
    if (op < 0 || op > 2) {
        Serial.println("Erro: Operação inválida");
        return -1;
    }
    return op;
}

byte recieveSensorsRead(struct Packet<SensorsRead>* pck){
    if(senderHandshake(lora, 1) == 0){
        Serial.println("Erro no handshake");
        return 0;
    }
    Serial.println("Handshake feito");
    Serial.println("");
    
    if(waitSensorsRead() == 1){
        printSensorReads(&pck->data);
        sendACK(lora, channel, 1);
    }
    return 1;
}

byte waitSensorsRead(){
    struct Packet<SensorsRead> pck;
    unsigned long startTime = millis();
    while (millis() - startTime < timeOutSensorsReadPacket){
        if (lora.available()  > 1){
            ResponseStructContainer rsc = lora.receiveMessage(sizeof(Packet<SensorsRead>));
            pck = *(Packet<SensorsRead>*) rsc.data;
            rsc.close();
            Serial.println("Pacote com leitura de sensores recebido");
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

void printSensorReads(struct SensorsRead* data) {
    Serial.print("Acelerometro: ");
    Serial.print(data->accelerometer[0]);
    Serial.print(", ");
    Serial.print(data->accelerometer[1]);
    Serial.print(", ");
    Serial.println(data->accelerometer[2]);

    Serial.print("Umidade do solo: ");
    Serial.println(data->soilMoisture);

    Serial.print("Temperatura do ar: ");
    Serial.println(data->airTemperature);

    Serial.print("Umidade do ar: ");
    Serial.println(data->airHumidity);

    Serial.print("Valor do sensor de chuva: ");
    Serial.println(data->rainSensorValue);
}

void loadLoraConfigPacket(struct Packet<LoRaConfig>* pck, int* config){
    pck->data.transmissionPower = config[16];
    pck->data.enableLBT = config[17];
    pck->data.CHAN = config[18];
    pck->data.WORPeriod = config[19];
    pck->data.airDataRate = config[20];
    pck->data.cryptH = config[21];
    pck->data.cryptL = config[22];
    pck->data.timeOutConfigPacket = config[23];
    pck->data.timeOutSensorsReadPacket = config[24];
    pck->data.timeOutHandshake = config[25];
    pck->data.timeOutSYNACK =config[26];
    pck->data.timeOutACK =config[27];
    Serial.println("Pacote de LoRaConfig montado: ");
    Serial.println(stringifyLoraConfig(&pck->data));
}

byte sendLoraConfigPacket(int* config){
    struct Packet<LoRaConfig> pck;
    if(senderHandshake(lora, 0) == 0){
        Serial.println("Erro no handshake");
        return 0;
    }
    Serial.println("Handshake feito");
    pck.OP = 0;
    loadLoraConfigPacket(&pck, config);
    String string_lora_config = stringifyLoraConfig(&pck.data);
    pck.Checksum = crc16_ccitt(reinterpret_cast<const uint8_t *>(string_lora_config.c_str()), string_lora_config.length());
    lora.sendFixedMessage(receptorAddr[0],receptorAddr[1],channel,&pck,sizeof(Packet<LoRaConfig>));
    Serial.println("Pacote de configuração enviado para " + String(receptorAddr[0])+String(receptorAddr[1]));

    struct Packet<byte> pck_ack;
    unsigned long startTime = millis();
    while(waitACK(lora, &pck_ack) == 0){
        if(millis() - startTime >= timeOutConfigPacket){
            return 0;
        }
        else{
            lora.sendFixedMessage(receptorAddr[0],receptorAddr[1],channel,&pck,sizeof(Packet<LoRaConfig>));
        }
    }
    Serial.println("Pacote foi recebido pelo destinatario");
    Serial.println("Testando configuração com " + String(receptorAddr[0])+String(receptorAddr[1]));
    if(senderHandshake(lora, 3) == 0){
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
        Serial.println("Conectando ao WiFi...");
    }
    Serial.println("Conectado ao WiFi");
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
        Serial.println(-1);
    }
    http.end();
}

byte loadConfigForCommunication(int* config, byte op){
    receptorAddr[0]=config[0];
    receptorAddr[1]=config[1];
    channel=config[4];
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    
    configuration.CHAN = channel;
    configuration.TRANSMISSION_MODE.WORPeriod = config[5];
    configuration.SPED.airDataRate = config[6];
    configuration.CRYPT.CRYPT_H = config[7];
    configuration.CRYPT.CRYPT_L = config[8];

    timeOutConfigPacket = config[9]*1000;
    timeOutSensorsReadPacket = config[10]*1000;
    timeOutHandshake = config[11]*1000;
    timeOutSYNACK =config[12]*1000;
    timeOutACK =config[13]*1000;

    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    Serial.println("    Nova Configuração do LoRa para essa comunicação   ");
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

void printLoRaConfig(){
    lora.begin();
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    printParameters(configuration);
    c.close();
}

void loadDefaultLoraConfig(){
    senderAddr[0] = 0;
    senderAddr[1] = 1;
    channel = 64;

    lora.begin();
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;
    configuration.CHAN = channel;
    configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    configuration.SPED.uartBaudRate = UART_BPS_9600;
    configuration.SPED.uartParity = MODE_00_8N1;
    configuration.ADDH = senderAddr[0];
    configuration.ADDL = senderAddr[1];
    configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;

    ResponseStatus rs = lora.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);
    printParameters(configuration);
    c.close();
}