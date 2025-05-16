#include "crc.hpp"
#include <string.h>

uint8_t computeCRC8(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal)
{
    uint16_t ResCRC = InitVal;

    while (--Length >= 0)
    {
        ResCRC ^= *pData++;
        for (int i = 0; i < 8; ++i)
            ResCRC = ResCRC & 0x80 ? (ResCRC << 1) ^ Poly : ResCRC << 1;
    }
    return ResCRC & 0xFF;
}

uint16_t computeCRC16(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal)
{
    uint16_t ResCRC = InitVal;

    while (--Length >= 0)
    {
        ResCRC ^= *pData++ << 8;
        for (int i = 0; i < 8; ++i)
            ResCRC = ResCRC & 0x8000 ? (ResCRC << 1) ^ Poly : ResCRC << 1;
    }
    return ResCRC & 0xFFFF;
}

uint32_t computeCRC32(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal)
{
    uint32_t ResCRC = InitVal;

    while (--Length >= 0)
    {
        ResCRC ^= *pData++ << 24;
        for (int i = 0; i < 8; ++i)
            ResCRC = ResCRC & 0x80000000 ? (ResCRC << 1) ^ Poly : ResCRC << 1;
    }
    return ResCRC & 0xFFFFFFFF;
}

int addCRC8(uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal)
{
    uint8_t crc = computeCRC8(pData, Length, Poly, InitVal);
    pData[Length] = crc;
    return Length + 1; // Długość danych + 1 bajt CRC
}

int addCRC16(uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal)
{
    uint16_t crc = computeCRC16(pData, Length, Poly, InitVal);
    pData[Length] = (crc >> 8) & 0xFF; // Starszy bajt
    pData[Length + 1] = crc & 0xFF;    // Młodszy bajt
    return Length + 2;                 // Długość danych + 2 bajty CRC
}

int addCRC32(uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal)
{
    uint32_t crc = computeCRC32(pData, Length, Poly, InitVal);
    pData[Length] = (crc >> 24) & 0xFF; // Najstarszy bajt
    pData[Length + 1] = (crc >> 16) & 0xFF;
    pData[Length + 2] = (crc >> 8) & 0xFF;
    pData[Length + 3] = crc & 0xFF; // Najmłodszy bajt
    return Length + 4;              // Długość danych + 4 bajty CRC
}

int verifyCRC8(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal)
{
    if (Length <= 1)
        return 0; // Za krótki bufor, musi być przynajmniej dane + CRC
    uint8_t crc = computeCRC8(pData, Length, Poly, InitVal);
    return (crc == 0); // CRC powinno być równe 0, jeśli dane są poprawne
}

int verifyCRC16(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal)
{
    if (Length <= 2)
        return 0; // Za krótki bufor, muszą być przynajmniej dane + CRC
    uint16_t crc = computeCRC16(pData, Length, Poly, InitVal);
    return (crc == 0); // CRC powinno być równe 0, jeśli dane są poprawne
}

int verifyCRC32(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal)
{
    if (Length <= 4)
        return 0; // Za krótki bufor, muszą być przynajmniej dane + CRC
    uint32_t crc = computeCRC32(pData, Length, Poly, InitVal);
    return (crc == 0); // CRC powinno być równe 0, jeśli dane są poprawne
}
