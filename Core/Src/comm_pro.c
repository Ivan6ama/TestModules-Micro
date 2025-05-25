/**
 * @file comm_pro.c
 * @brief Implementación del protocolo de comunicación serial UNER.
 */

#include "comm_pro.h"

/// Puntero a función para enviar el buffer serial
static txFunct_t sendFunction;
/// Último comando decodificado
static _eCmd CMDID;
/// Estado actual del protocolo
static _eProtocol protocolState;

/**
 * @brief Asigna la función de transmisión para el protocolo.
 */
void UNER_SetTxFunction(txFunct_t func)
{
    sendFunction = func;
}

/**
 * @brief Inicializa las estructuras de transmisión y recepción.
 */
void UNER_Init(_sTx *tx, uint8_t *txBuf, _sRx *rx, uint8_t *rxBuf)
{
    tx->buf = txBuf;
    tx->iw = 0;
    tx->ir = 0;
    tx->length = SIZEBUFTX - 1;
    tx->chks = 0;

    rx->buf = rxBuf;
    rx->iw = 0;
    rx->ir = 0;
    rx->header = HEADER_U;
    rx->length = SIZEBUFRX - 1;
    rx->size = 0;
    rx->data = 0;
    rx->chks = 0;

    protocolState = HEADER_U;
    CMDID = UNKNOWN;
}

/**
 * @brief Lógica periódica de recepción y transmisión de datos.
 */
void UNER_SerialTask(_sTx *tx, _sRx *rx)
{
    if (rx->iw != rx->ir) {
        UNER_DecodeHeader(rx, tx);
    }

    if (tx->ir != tx->iw) {
        sendFunction(&tx->buf[tx->ir]);
        tx->ir++;
        tx->ir &= tx->length;
    }
}

/**
 * @brief Almacena cada byte recibido en el buffer.
 */
void UNER_OnRxByte(_sRx *rx, uint8_t data)
{
    rx->buf[rx->iw++] = data;
    rx->iw &= rx->length;
}


/**
 * @brief Decodifica el encabezado del protocolo UNER.
 * @param rx Buffer de recepción.
 * @param tx Buffer de transmisión.
 */
void UNER_DecodeHeader(_sRx *rx, _sTx *tx)
{
    uint8_t i = rx->iw;
    while(rx->ir != i){
        switch(rx->header){
            case HEADER_U:
                if(rx->buf[rx->ir] == 'U'){
                    rx->header = HEADER_N;
                }
            break;
            case HEADER_N:
                if(rx->buf[rx->ir] == 'N'){
                    rx->header = HEADER_E;
                }else{
                    rx->header = HEADER_U;
                    rx->ir--;
                }
            break;
            case HEADER_E:
                if(rx->buf[rx->ir] == 'E'){
                    rx->header = HEADER_R;
                }else{
                    rx->header = HEADER_U;
                    rx->ir--;
                }
            break;
            case HEADER_R:
                if(rx->buf[rx->ir] == 'R'){
                    rx->header = NBYTES;
                }else{
                    rx->header = HEADER_U;
                    rx->ir--;
                }
            break;
            case NBYTES:
                rx->header = TOKEN;
                rx->size = rx->buf[rx->ir];
            break;
            case TOKEN:
                if(rx->buf[rx->ir] == ':'){
                    rx->chks = 'U' ^ 'N' ^ 'E' ^ 'R' ^ ':' ^ rx->size;
                    rx->header = PAYLOAD;
                    rx->data = rx->ir + 1;
                    rx->data &= rx->length;
                }else{
                    rx->header = HEADER_U;
                    rx->ir--;
                }
            break;
            case PAYLOAD:
                rx->size--;
                if(rx->size > 0){
                    rx->chks ^= rx->buf[rx->ir];
                }else{
                    if(rx->buf[rx->ir] == rx->chks){
                        UNER_DecodePayload(rx->buf[rx->data], tx);
                    }
                    rx->header = HEADER_U;
                }
            break;
            default:
                rx->header = HEADER_U;
            break;
        }
        rx->ir++;
        rx->ir &= rx->length;
    }
}

void UNER_DecodePayload(uint8_t bufRx,_sTx *tx)
{
    switch(bufRx){
        case ALIVE:
            UNER_WriteContentTx(bufRx, tx);
        break;
        default:
            UNER_WriteContentTx(bufRx, tx);
        break;
    }
}

void UNER_WriteContentTx(uint8_t bufRx,_sTx *tx)
{
    switch(bufRx){
        case ALIVE:
            UNER_PutHeaderOnTx(tx, ALIVE, 2);
            UNER_PutByteOnTx(tx, ACK);
            UNER_PutByteOnTx(tx, tx->chks);
        break;
        default:
            UNER_PutHeaderOnTx(tx, (_eCmd)tx->buf[tx->iw], 2);
            UNER_PutByteOnTx(tx, UNKNOWN);
            UNER_PutByteOnTx(tx, tx->chks);
        break;
    }
}

uint8_t UNER_PutHeaderOnTx(_sTx *tx, _eCmd ID, uint8_t N)
{
    tx->chks = 0;
    tx->buf[tx->iw++] = 'U';
    tx->iw &= tx->length;
    tx->buf[tx->iw++] = 'N';
    tx->iw &= tx->length;
    tx->buf[tx->iw++] = 'E';
    tx->iw &= tx->length;
    tx->buf[tx->iw++] = 'R';
    tx->iw &= tx->length;
    tx->buf[tx->iw++] = N + 1;
    tx->iw &= tx->length;
    tx->buf[tx->iw++] = ':';
    tx->iw &= tx->length;
    tx->buf[tx->iw++] = ID;
    tx->iw &= tx->length;
    tx->chks ^= (N + 1);
    tx->chks ^= ('U' ^ 'N' ^ 'E' ^ 'R' ^ ID ^ ':');
    return tx->chks;
}

uint8_t UNER_PutByteOnTx(_sTx *tx, uint8_t byte)
{
    tx->buf[tx->iw++] = byte;
    tx->iw &= tx->length;
    tx->chks ^= byte;
    return tx->chks;
}

uint8_t UNER_PutStrOnTx(_sTx *tx, const char *str)
{
    uint16_t generalIndex = 0;
    while (str[generalIndex]) {
        tx->buf[tx->iw++] = str[generalIndex];
        tx->iw &= tx->length;
        tx->chks ^= str[generalIndex++];
    }
    return tx->chks;
}
