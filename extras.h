#ifndef EXTRAS_H
    #define EXTRAS_H
    #include <CRC16.h>
    
    unsigned long int timeout_packet = 15000;
    unsigned long int time_out_SYNACK = 5000;
    unsigned long int time_out_ACK = 5000;
    unsigned long int time_out_handshake = 15000;
    
    CRC16 crc;

    struct SensorsRead{
        float accelerometer[3];
        float air_temperature;
        float air_humidity;
        int soil_moisture;
        int rain_sensor_value;
    };

    struct LoRaConfig{
        byte ADDH;
        byte ADDL;
        byte CHAN;
        byte uart_parity;
        byte uart_baud_rate;
        byte air_data_rate;
        byte sub_packet_option;
        byte transmission_power;
        bool enable_RSSI_ambient_noise;
        byte wor_period;
        bool enable_lbt;
        bool enable_rssi;
        bool enable_fixed_transmission;
    };

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

    template<typename T>
    u_int16_t serializeData(const T* data) {
        byte dataBuffer[sizeof(*data)];
        memcpy(dataBuffer, data, sizeof(*data));
        crc.add((uint8_t*)dataBuffer, sizeof(*data));
        u_int16_t checksum = crc.calc();
        crc.restart();
        return checksum;           
    }

#endif
