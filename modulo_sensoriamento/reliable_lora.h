#ifndef RELIABLE_LORA_H
    #define RELIABLE_LORA_H
    #ifndef Packet_H
        #include "packet.h" 
    #endif
    #ifndef LoRa_E220_h
        #include <LoRa_E220.h> 
    #endif
    byte sendSYN(LoRa_E220 lora, byte receptor_addr[2],byte communication_channel, byte OP){
        struct Packet<byte> pck;
        pck.data = 0;
        pck.SYN = true;
        pck.OP = OP;
        ResponseStatus lora_response = lora.sendFixedMessage(receptor_addr[0],receptor_addr[1],communication_channel,&pck,sizeof(Packet<byte>));
        return lora_response.code;

    }

    byte sendACK(LoRa_E220 lora, byte receptor_addr[2], byte communication_channel, byte OP){
        struct Packet<byte> pck;
        pck.data = 0;
        pck.ACK = true;
        pck.OP = OP;
        ResponseStatus lora_response = lora.sendFixedMessage(receptor_addr[0],receptor_addr[1],communication_channel,&pck,sizeof(Packet<byte>));
        return lora_response.code;
    }

    byte sendSYNACK(LoRa_E220 lora, byte receptor_addr[2], byte communication_channel, byte OP){
        struct Packet<byte> pck;
        pck.data = 0;
        pck.SYN = true;
        pck.ACK = true;
        pck.OP = OP;
        ResponseStatus lora_response = lora.sendFixedMessage(receptor_addr[0],receptor_addr[1],communication_channel,&pck,sizeof(Packet<byte>));
        return lora_response.code;
    }

    byte waitSYN(LoRa_E220 lora, struct Packet<byte>* pck, byte* OP){
        if (lora.available()  > 1){
            ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(Packet<byte>));
            *pck = *(Packet<byte>*) rsc.data;
            rsc.close();
            if(pck->SYN == true){
                *OP = pck->OP;
                return 1;
            }
        }
        return 0;
    }

    byte waitSYNACK(LoRa_E220 lora, struct SysConfigs* sc, struct Packet<byte>* pck){
        unsigned long startTime = millis();
        while (millis() - startTime < sc->time_out_SYNACK){
            if (lora.available()  > 1){
                ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(Packet<byte>));
                *pck = *(Packet<byte>*) rsc.data;
                rsc.close();
                if(pck->SYN == true && pck->ACK == true){
                    return 1;
                }
            }
        }
        return 0;
    }

    byte waitACK(LoRa_E220 lora, struct SysConfigs* sc, struct Packet<byte>* pck){
        unsigned long startTime = millis();
        while (millis() - startTime < sc->time_out_ACK){
            if (lora.available()  > 1){
                ResponseStructContainer rsc = lora.receiveMessageRSSI(sizeof(Packet<byte>));
                *pck = *(Packet<byte>*) rsc.data;
                rsc.close();
                if(pck->ACK == true){
                    return 1;
                }
            }
        }
        return 0;
    }   
#endif
