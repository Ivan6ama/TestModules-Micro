/**
 * @file comm_pro.h
 * @brief Protocolo de comunicación serial tipo "UNER".
 */

#ifndef COMM_PRO_H
#define COMM_PRO_H

#include <stdint.h>

#define SIZEBUFRX 256  ///< Tamaño del buffer de recepción
#define SIZEBUFTX 256  ///< Tamaño del buffer de transmisión

/**
 * @brief Estructura de recepción serial.
 */
typedef struct __attribute__((packed, aligned(1))) {
    uint8_t *buf;     ///< Buffer de recepción
    uint8_t iw;       ///< Índice de escritura
    uint8_t ir;       ///< Índice de lectura
    uint8_t header;   ///< Estado del header
    uint8_t length;   ///< Longitud del buffer
    uint8_t size;     ///< Tamaño del payload
    uint8_t data;     ///< Posición del dato
    uint8_t chks;     ///< Checksum
} _sRx;

/**
 * @brief Estructura de transmisión serial.
 */
typedef struct __attribute__((packed, aligned(1))) {
    uint8_t *buf;     ///< Buffer de transmisión
    uint8_t iw;       ///< Índice de escritura
    uint8_t ir;       ///< Índice de lectura
    uint8_t length;   ///< Longitud del buffer
    uint8_t chks;     ///< Checksum
} _sTx;

/**
 * @brief Union para facilitar el acceso a datos crudos.
 */
typedef union {
    uint8_t  ui8[4];
    int8_t   i8[4];
    uint16_t ui16[2];
    int16_t  i16[2];
    uint32_t ui32;
    int32_t  i32;
} _uWork;

/**
 * @brief Comandos disponibles en el protocolo.
 */
typedef enum {
    ALIVE = 0xF0,
    ACK = 0x0D,
    FIRMWARE = 0xF1,
    ASENSORS = 0xA0,
    MOTORTEST = 0xA1,
    SERVOANGLE = 0xA2,
    ULTRADIST = 0xA3,
    MOTORVEL = 0xA4,
    SERVOCONFIG = 0xA5,
    LASTBLACK = 0xA6,
    LASTWHITE = 0xA7,
    ROBOTDATA = 0xA8,
    TEST = 0xA9,
    UNKNOWN = 0xFF
} _eCmd;

/**
 * @brief Estados del protocolo de recepción.
 */
typedef enum {
    HEADER_U,
    HEADER_N,
    HEADER_E,
    HEADER_R,
    NBYTES,
    TOKEN,
    PAYLOAD
} _eProtocol;

/// Puntero a función para enviar datos
typedef void (*txFunct_t)(uint8_t *buf);

#ifdef __cplusplus
extern "C" {
#endif

// Funciones públicas

/**
 * @brief Inicializa la estructura del protocolo.
 */
void UNER_Init(_sTx *tx, uint8_t *txBuf, _sRx *rx, uint8_t *rxBuf);

/**
 * @brief Debe llamarse periódicamente para enviar y decodificar datos.
 */
void UNER_SerialTask(_sTx *tx, _sRx *rx);

/**
 * @brief Procesa cada byte recibido (se llama desde interrupción).
 */
void UNER_OnRxByte(_sRx *rx, uint8_t data);

/**
 * @brief Decodifica encabezado del protocolo.
 */
void UNER_DecodeHeader(_sRx *rx, _sTx *tx);

/**
 * @brief Decodifica el payload del mensaje recibido.
 */
void UNER_DecodePayload(uint8_t rxBuf, _sTx *tx);

/**
 * @brief Escribe el contenido de la respuesta en el buffer de transmisión.
 */
void UNER_WriteContentTx(uint8_t rxBuf, _sTx *tx);

/**
 * @brief Pone el Header del protocolo en el buffer de transmision
 */
uint8_t UNER_PutHeaderOnTx(_sTx *tx, _eCmd ID, uint8_t N);

/**
 * @brief Pone un byte de informacion del protocolo en el buffer de transmision
 */
uint8_t UNER_PutByteOnTx(_sTx *tx, uint8_t byte);

/**
 * @brief Escribe el contenido de un string en el protocolo para enviar mediante buffer de transmision
 */
uint8_t UNER_PutStrOnTx(_sTx *tx, const char *str);

/**
 * @brief Escribe el contenido de la respuesta en el buffer de transmisión.
 */
void UNER_SetTxFunction(txFunct_t func);

#ifdef __cplusplus
}
#endif

#endif // COMM_PRO_H
