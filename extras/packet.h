#ifndef PACKET_H
    #define PACKET_H

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

#endif