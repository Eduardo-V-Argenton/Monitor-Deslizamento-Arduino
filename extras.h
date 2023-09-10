#ifndef EXTRAS_H
    #define EXTRAS_H
    
    unsigned long int timeOutConfigPacket = 15000;
    unsigned long int timeOutSensorsReadPacket = 15000;
    unsigned long int timeOutHandshake = 15000;

    struct SensorsRead{
        float accelerometer[3];
        int soilMoisture;
        int rainSensorValue;
    };

    struct LoRaConfig{
        float timeOutConfigPacket;
        float timeOutSensorsReadPacket;
        float timeOutHandshake;
        float timeOutSYNACK;
        float timeOutACK;
        byte CHAN;
        byte airDataRate;
        byte transmissionPower;
        byte WORPeriod;
        byte cryptH;
        byte cryptL;
        bool enableLBT;
    };

    String stringifyLoraConfig(struct LoRaConfig* config){
        String result = String(config->CHAN) + ";" +
                        String(config->airDataRate) + ";" +
                        String(config->transmissionPower) + ";" +
                        String(config->WORPeriod) + ";" +
                        String(config->enableLBT) + ";" +
                        String(config->cryptH) + ";" +
                        String(config->cryptL) + ";" +
                        String(config->timeOutConfigPacket) + ";" +
                        String(config->timeOutSensorsReadPacket) + ";" +
                        String(config->timeOutHandshake) + ";" +
                        String(config->timeOutSYNACK) + ";" +
                        String(config->timeOutACK);

        return result;
    }

    String stringifySensorsRead(const SensorsRead* sr) {
        String result = String(sr->accelerometer[0]) + ";" +
                        String(sr->accelerometer[1]) + ";" +
                        String(sr->accelerometer[2]) + ";" +
                        String(sr->soilMoisture) + ";" +
                        String(sr->rainSensorValue);
        return result;
    }

    void printParameters(struct Configuration configuration) {
        Serial.println("----------------------------------------");

        Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
        Serial.println(F(" "));
        Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
        Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
        Serial.println(F(" "));
        Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -&gt; "); Serial.println(configuration.getChannelDescription());
        Serial.println(F(" "));
        Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -&gt; "); Serial.println(configuration.SPED.getUARTParityDescription());
        Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -&gt; "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
        Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -&gt; "); Serial.println(configuration.SPED.getAirDataRateDescription());
        Serial.println(F(" "));
        Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -&gt; "); Serial.println(configuration.OPTION.getSubPacketSetting());
        Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -&gt; "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
        Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -&gt; "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
        Serial.println(F(" "));
        Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
        Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
        Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
        Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -&gt; "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());


        Serial.println("----------------------------------------");
    }

    uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
        uint16_t crc = 0xFFFF;

        for (size_t i = 0; i < length; ++i) {
            crc ^= (uint16_t)data[i] << 8;

            for (int j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }

    void parseCommandToConfig(const char* input, int* outputArray) {
        char *token;
        int index = 0;
        token = strtok((char *)input, ";");
        while (token != NULL) {
            outputArray[index++] = atoi(token);
            token = strtok(NULL, ";");
        }
    }

#endif
