
#ifndef HDC1080_SENSOR_H_

#define HDC1080_SENSOR_H_



#include <stm32f4xx.h>

#include "i2Cxdriver.h"

#include "SystickDriver.h"





#define HDC1080_ADDRESS  0x40  //address slave


/**** MASCARAS DE CONFIGURACION   */

#define HDC1080_HUMIDITY_RES_Pos    0x8U                                       // Position del bits de resolucion de la humedad

#define HDC1080_HUMIDITY_RES        (3UL<<HDC1080_HUMIDITY_RES_Pos)   // Msk  0x0300

#define HDC1080_HUMIDITY_RES_0      (1UL<<HDC1080_HUMIDITY_RES_Pos)   // Msk  0x0100

#define HDC1080_HUMIDITY_RES_1      (2UL<<HDC1080_HUMIDITY_RES_Pos)   // Msk  0x0200

#define HDC1080_TEMPERATURE_RES_Pos 10U                                        // Position del bit de resolucion de la temperatura

#define HDC1080_TEMPERATURE_RES     (1UL<<HDC1080_TEMPERATURE_RES_Pos)         // Msk  0x0400

#define HDC1080_TYPE_TAKE_DATA_Pos  12U   // Posicion del bit de tipo de toma de datos

#define HDC1080_TYPE_TAKE_DATA      (1UL<<HDC1080_TYPE_TAKE_DATA_Pos)   // Msk  0x1000

#define SENSOR_ADD 0b1000000 // Unica direccion
#define TEMPERATURE 0x00

//Resolucion de la temperatura

enum{

	TEMPERATURE_RESOLUTION_14BIT =0,

	TEMPERATURE_RESOLUTION_11BIT=1,

}TEMP_RESO;

//Resolucion de la humedad

enum{

	HUMIDITY_RESOLUTION_14BIT=0,

	HUMIDITY_RESOLUTION_11BIT=1,

	HUMIDITY_RESOLUTION_8BIT=2

}HUMI_RESO;



enum{

	HDC1080_TEMPERATURE      = 0x00,

	HDC1080_HUMIDITY         = 0x01,

	HDC1080_CONFIGURATION    = 0x02,

	HDC1080_SERIAL_ID_FIRST  = 0xFB,

	HDC1080_SERIAL_ID_MID    = 0xFC,

	HDC1080_SERIAL_ID_LAST   = 0xFD,

	HDC1080_MANUFACTURER_ID  = 0xFE,

	HDC1080_DEVICE_ID        = 0xFF

};



enum{

	HDC_WRITE=0,

	HDC_READ,

};

enum{

	TEMPERATURE_OR_HUMIDITY=0,

	TEMPERATURE_AND_HUMIDITY,

};



typedef struct{

	uint8_t type_take_data;

	uint16_t resolution_temp;

	uint16_t resolution_humi;

}HDC1080_Config_t;



typedef struct

{

	I2C_Handler_t *ptrI2Cx;

	HDC1080_Config_t HDC1080_Config;

}HDC1080_Handler_t;



//Cabeceras de las funciones de configuracion y del funcionamiento del sistema

void hdc1080_config(HDC1080_Handler_t *ptrHandlerHDC);

uint32_t hdc1080_readTH(HDC1080_Handler_t *ptrHandlerHDC);

uint16_t hdc1080_readT(HDC1080_Handler_t *ptrHandlerHDC);

uint16_t hdc1080_readH(HDC1080_Handler_t *ptrHandlerHDC);

uint16_t hdc1080_readID(HDC1080_Handler_t *ptrHandlerHDC);

uint8_t hdc1080_gettimeconversion(void);



#endif /* HDC1080_SENSOR_H_ */


