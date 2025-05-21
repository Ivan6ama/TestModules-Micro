#ifndef COMM_PRO_H
#define COMM_PRO_H

#include <stdint.h>

#define SIZEBUFRX 256
#define SIZEBUFTX 256

// Estructuras empaquetadas 
typedef struct __attribute__((packed,aligned(1))){
    uint8_t     *buf;
    uint8_t     iw;
    uint8_t     ir;
    uint8_t     header;
    uint8_t     length;
    uint8_t     size;
    uint8_t     data;
    uint8_t     chks;
}_sRx;

typedef struct __attribute__((packed,aligned(1))){
    uint8_t     *buf;
    uint8_t     iw;
    uint8_t     ir;
    uint8_t     length;
    uint8_t     chks;
}_sTx;

typedef union{
    uint8_t     ui8[4];
    int8_t      i8[4];
    uint16_t    ui16[2];
    int16_t     i16[2];
    uint32_t    ui32;   
    int32_t     i32;
}_uWork;

typedef enum{
    ALIVE = 0xF0,
    ACK = 0x0D,
    FIRMWARE= 0xF1,
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
    UNKNOWN = 0xFF,
}_eCmd;

typedef enum{
    HEADER_U,
    HEADER_N,
    HEADER_E,
    HEADER_R,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocol;

typedef void (*txFunct_t)(uint8_t *buf);

// Funciones públicas
void UNER_Init(_sTx *tx, uint8_t *txBuf, _sRx *rx, uint8_t *rxBuf);
void UNER_SerialTask(_sTx *tx, _sRx *rx);
void UNER_OnRxByte(_sRx *rx, uint8_t data);
void UNER_DecodeHeader(_sRx *rx, _sTx *tx);
void UNER_DecodePayload(uint8_t rxBuf, _sTx *tx);
void UNER_WriteContentTx(uint8_t rxBuf, _sTx *tx);

uint8_t UNER_PutHeaderOnTx(_sTx *tx, _eCmd ID, uint8_t N);
uint8_t UNER_PutByteOnTx(_sTx *tx, uint8_t byte);
uint8_t UNER_PutStrOnTx(_sTx *tx, const char *str);

void UNER_SetTxFunction(txFunct_t func);

#endif // COMM_PRO_H
