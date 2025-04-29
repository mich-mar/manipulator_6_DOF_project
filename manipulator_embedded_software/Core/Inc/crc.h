#ifndef CRC_H
#define CRC_H

#include <stdint.h>

uint8_t computeCRC8(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);
uint16_t computeCRC16(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);
uint32_t computeCRC32(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

int addCRC8(uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);
int addCRC16(uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);
int addCRC32(uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

int verifyCRC8(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);
int verifyCRC16(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);
int verifyCRC32(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/* Domyślne wartości dla CRC */
#define CRC8_INIT 0
#define CRC8_POLYNOMIAL 0x39

#define CRC16_INIT 0
#define CRC16_POLYNOMIAL 0x8005

#define CRC32_INIT 0
#define CRC32_POLYNOMIAL 0x04C11DB7

#endif /* CRC_H */