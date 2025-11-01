#pragma once

#include <string>
#include "./protocol.h"
namespace sp_referee
{
    class Check
    {
        public:
        uint8_t getCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length, unsigned char uc_crc_8);

        uint32_t verifyCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length);

        void appendCRC8CheckSum(unsigned char* pch_message, unsigned int dw_length);

        uint16_t getCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length, uint16_t w_crc);

        uint32_t verifyCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length);

        void appendCRC16CheckSum(uint8_t* pch_message, uint32_t dw_length);

    };

}