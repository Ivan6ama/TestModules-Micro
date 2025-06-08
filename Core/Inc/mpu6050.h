/*
 * mpu6050.h
 * -----------------------------------------------
 * Cabecera de la librería para el sensor MPU6050.
 * Define las direcciones de registro, offsets, escala fija,
 * y prototipos de funciones para lectura e inicialización.
 *
 * El MPU6050 es un sensor MEMS de 6 ejes (3 acelerómetro + 3 giroscopio),
 * que se comunica mediante el protocolo I2C.
 *
 * Autor: Gamarra Ivan
 * Fecha: Junio 2025
 */
#ifndef MPU6050_H_  // Evita inclusión múltiple
#define MPU6050_H_

#include "stdlib.h"
#include "stdint.h"
#include "string.h"


// Dirección del dispositivo (con AD0 = 0 → 0x68 << 1 = 0xD0 para escritura)
#define MPU6050_ADDR         0xD0

#define WHO_AM_I_REG         0x75  // Registro de identidad (debe devolver 0x68)
#define PWR_MGMT_1_REG       0x6B  // Registro para salir del modo de bajo consumo
#define GYRO_CONFIG_REG      0x1B  // Registro de configuración del giroscopio
#define ACCEL_CONFIG_REG     0x1C  // Registro de configuración del acelerómetro
#define ACCEL_XOUT_H_REG     0x3B  // Dirección base de lectura del acelerómetro
#define GYRO_XOUT_H_REG      0x43  // Dirección base de lectura del giroscopio


typedef struct {
	void (*WriteConfig)(uint8_t address, uint8_t control, uint8_t data);              // para 1 byte
    void (*Read)();  // para multibyte
} MPU_IO_Interface_t;

// Inicializa el sensor: saca del modo sleep y configura escala de medición
void MPU6050_Init(void);

// Lee el acelerómetro, aplica offset y escala a m/s² ×100
void MPU6050_Task(void);

// Lee el acelerómetro, aplica offset y escala a m/s² ×100
void MPU6050_Read_Accel(void);

// Lee el giroscopio, aplica offset y escala a °/s ×100
void MPU6050_Read_Gyro(void);

void MPU6050_RegisterIO(const MPU_IO_Interface_t* io);


#endif /* MPU6050_H_ */
