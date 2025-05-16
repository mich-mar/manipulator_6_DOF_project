#ifndef CRC_H
#define CRC_H

#include <stdint.h>

/**
 * @brief Oblicza 8-bitową sumę kontrolną CRC.
 *
 * @param pData Wskaźnik na dane wejściowe
 * @param Length Długość danych w bajtach
 * @param Poly Wielomian
 * @param InitVal Wartość początkowa
 * @return Obliczona wartość CRC8
 */
uint8_t computeCRC8(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/**
 * @brief Oblicza 16-bitową sumę kontrolną CRC.
 *
 * @param pData Wskaźnik na dane wejściowe
 * @param Length Długość danych w bajtach
 * @param Poly Wielomian
 * @param InitVal Wartość początkowa
 * @return Obliczona wartość CRC16
 */
uint16_t computeCRC16(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/**
 * @brief Oblicza 32-bitową sumę kontrolną CRC.
 *
 * @param pData Wskaźnik na dane wejściowe
 * @param Length Długość danych w bajtach
 * @param Poly Wielomian
 * @param InitVal Wartość początkowa
 * @return Obliczona wartość CRC32
 */
uint32_t computeCRC32(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/**
 * @brief Dodaje sumę kontrolną CRC8 na końcu bufora.
 *
 * @param pData Wskaźnik na dane wejściowe
 * @param Length Długość danych w bajtach (bez CRC)
 * @param Poly Wielomian
 * @param InitVal Wartość początkowa
 * @return Długość całego pakietu (dane + suma kontrolna)
 */
int addCRC8(uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/**
 * @brief Dodaje sumę kontrolną CRC16 na końcu bufora.
 *
 * @param pData Wskaźnik na dane wejściowe
 * @param Length Długość danych w bajtach (bez CRC)
 * @param Poly Wielomian
 * @param InitVal Wartość początkowa
 * @return Długość całego pakietu (dane + suma kontrolna)
 */
int addCRC16(uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/**
 * @brief Dodaje sumę kontrolną CRC32 na końcu bufora.
 *
 * @param pData Wskaźnik na dane wejściowe
 * @param Length Długość danych w bajtach (bez CRC)
 * @param Poly Wielomian
 * @param InitVal Wartość początkowa
 * @return Długość całego pakietu (dane + suma kontrolna)
 */
int addCRC32(uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/**
 * @brief Weryfikuje poprawność danych korzystając z sumy kontrolnej CRC8.
 *
 * @param pData Wskaźnik na dane wejściowe (wraz z sumą kontrolną)
 * @param Length Długość danych w bajtach (wraz z CRC)
 * @param Poly Wielomian
 * @param InitVal Wartość początkowa
 * @return 1, jeśli dane są poprawne, 0 w przeciwnym wypadku
 */
int verifyCRC8(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/**
 * @brief Weryfikuje poprawność danych korzystając z sumy kontrolnej CRC16.
 *
 * @param pData Wskaźnik na dane wejściowe (wraz z sumą kontrolną)
 * @param Length Długość danych w bajtach (wraz z CRC)
 * @param Poly Wielomian
 * @param InitVal Wartość początkowa
 * @return 1, jeśli dane są poprawne, 0 w przeciwnym wypadku
 */
int verifyCRC16(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/**
 * @brief Weryfikuje poprawność danych korzystając z sumy kontrolnej CRC32.
 *
 * @param pData Wskaźnik na dane wejściowe (wraz z sumą kontrolną)
 * @param Length Długość danych w bajtach (wraz z CRC)
 * @param Poly Wielomian
 * @param InitVal Wartość początkowa
 * @return 1, jeśli dane są poprawne, 0 w przeciwnym wypadku
 */
int verifyCRC32(const uint8_t *pData, int Length, uint32_t Poly, uint16_t InitVal);

/**
 * @name Domyślne wartości dla CRC
 * @{
 */
#define CRC8_INIT 0            ///< Domyślna wartość początkowa dla CRC-8
#define CRC8_POLYNOMIAL 0x39   ///< Domyślny wielomian dla CRC-8

#define CRC16_INIT 0          ///< Domyślna wartość początkowa dla CRC-16
#define CRC16_POLYNOMIAL 0x8005 ///< Domyślny wielomian dla CRC-16

#define CRC32_INIT 0           ///< Domyślna wartość początkowa dla CRC-32
#define CRC32_POLYNOMIAL 0x04C11DB7 ///< Domyślny wielomian dla CRC-32
/** @} */

#endif /* CRC_H */
