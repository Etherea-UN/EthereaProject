
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "ExtiDriver.h"
#include "USARTxDriver.h"
#include "I2CxDriver.h"
#include "PWMDriver.h"
#include "PLLDriver.h"
#include "SysTickDriver.h"
#include "AdcDriver.h"
#include "RTCxDriver.h"
#include "arm_math.h"
#include "HDC1080_sensor.h"


//Definición de handlers GPIO
GPIO_Handler_t handlerBlinky 				= {0};		//Handler Blinky
GPIO_Handler_t handlerPinTX1				= {0};		//Handler Transmisión USART1
GPIO_Handler_t handlerPinRX1				= {0};		//Handler Recepción USART1
GPIO_Handler_t handlerPinTX6				= {0};		//Handler Transmisión USART6
GPIO_Handler_t handlerPinRX6				= {0};		//HAndler Recepción USART6
GPIO_Handler_t handlerValvePin				= {0};		//Handler Válvula

//Definición de handlers TIM
BasicTimer_Handler_t handlerTimerBlinky 	= {0};		//Handler Timer del blinky
BasicTimer_Handler_t handlerTimerData		= {0};		//Handler Timer de la muestra de datos

//Handlers comunicación serial USART
USART_Handler_t handlerUsart1 				= {0};		//Handler USART1
USART_Handler_t handlerUsart6				= {0};		//Handler USART6

//Handlers I2C
GPIO_Handler_t handlerI2cSDA				= {0};		//Handler SDA I2C acelerómetro
GPIO_Handler_t handlerI2cSCL 				= {0};		//Handler SCL I2C acelerómetro
I2C_Handler_t handlerAccelerometer 			= {0};		//Handler de la configuración I2C acelerómetro
I2C_Handler_t handlerBarometer				= {0};		//Handler de la configuración I2C barómetro

//Defiinicion de handlers Temp
GPIO_Handler_t handlerI2C_SCL  = {0};
GPIO_Handler_t handlerI2C_SDA  = {0};
I2C_Handler_t handler_HDC1080  = {0};

//Definición de variables
uint8_t rxData = 0;
uint8_t flagData = 0;
uint8_t dataOn = 0;
//handler sensotr
I2C_Handler_t i2cSensor     = {0};
//Definiciónes para el módulo GPS
uint8_t rxData6 = 0;

//Variables relacionadas con la comunicación I2C del Accel
uint16_t i2cBuffer[8];



//Variables relacionadas con el uso de los comandos en terminal
bool stringComplete;
uint16_t counterReception = 0;
char cmd[256] = {0};
char bufferReception[256] = {0};

void initSystem(void);
void readTemperature(void);
void readHumidity(void);
char bufferData[128] = {0};
uint8_t sendMsg = 0;
uint8_t receivedChar = '\0';
float32_t temperatura = 0;
uint8_t temp_tomada = 0;



//Definición de funciones
void initSystem(void);
void readTemperature(void);
void parseCommands(char *ptrBufferReception);

int main(void){

	//Activación del coprocesador matemático
	SCB->CPACR |= (0xF << 20);

	initSystem();

	/* Como la válvula es normalmente abierta, se requiere que inmediatamente
	 * el sistema inicie la válvula se cierre para que sea posible la conexión con
	 * el tanque de combustible
	 */
	GPIO_WritePin(&handlerValvePin, SET);

	/* Se imprimen los mensajes de inicio para dar info al usuario
	 * sobre el manejo del dispositivo
	 */

	writeMsg(&handlerUsart1, "\nPress . \n");

	/*Loop forever*/
	while(1){

		//Creamos una cadena de caracteres con los datos que llegan por el puerto serial
		//El caracter '@' nos indica que es el final de la cadena

		if ((rxData != '\0') && (rxData != '.')){
			bufferReception[counterReception] = rxData;
			counterReception++;

			//Se define el siguiente caracter para indicar que el string está completo
			if(rxData == ','){

				stringComplete = true;

				//Agrego esta línea para crear el string con null al final
				bufferReception[counterReception-1] = '\0';

				counterReception = 0;
			}
			//Para que no vuelva a entrar, Solo cambia debido a la interrupción
			rxData = '\0';
		}
		if (rxData == '.'){
			writeMsg(&handlerUsart1, "\n~Iniciando Sistema~\n");
			writeMsg(&handlerUsart1, "\nacc  -->  Calibración del Accel-Gyro \n");
			writeMsg(&handlerUsart1, "\nshow  -->  Presenta los datos actuales capturados por los sensores \n");
			writeMsg(&handlerUsart1, "\ndata  -->  Muestra de manera periodica los datos tomados por los sesores \n");
			writeMsg(&handlerUsart1, "\nstop  -->  Detiene la muestra de datos \n");
			writeMsg(&handlerUsart1, "\nvalve  -->  Alto o bajo para cerrar o abrir la valvula de combustible \n");
			rxData = '\0';
		}

		//Hacemos un análisis de la cadena de datos obtenida
		if(stringComplete){

			parseCommands(bufferReception);
			stringComplete = false;
		}


	}
	return 0;
}

void parseCommands(char *ptrBufferReception){

	sscanf(ptrBufferReception, "%s", cmd);

	if(strcmp(cmd, "acc") == 0){

		//Inicialización del MPU6050
		writeMsg(&handlerUsart1, "Comunicación iniciada\n");

		//Se cargan los datos de calibración correspondientes al Accel
		readTemperature();
		uint16_t datoT = (uint16_t)(i2cBuffer[0]<< 8)|(i2cBuffer[1]);
		float32_t grados = (((float32_t)datoT )/ 65536.0f) * (165.0f -40.0f);
		temperatura = (grados - 23.8);
		sprintf(bufferData, "Temperature: %.2f \n", temperatura);
		writeMsg(&handlerUsart1, bufferData);



	}
	else if(strcmp(cmd, "show") == 0){
		writeMsg(&handlerUsart1, "Comunicación iniciada\n");

		//Se cargan los datos de calibración correspondientes al Accel
		readTemperature();
		uint16_t datoT = (uint16_t)(i2cBuffer[0]<< 8)|(i2cBuffer[1]);
		float32_t grados = (((float32_t)datoT )/ 65536.0f) * (165.0f -40.0f);
		temperatura = (grados - 23.8);
		sprintf(bufferData, "Temperature: %.2f \n", temperatura);
		writeMsg(&handlerUsart1, bufferData);


	}

	else if(strcmp(cmd, "stop") == 0){
		dataOn = 0;
	}
	else if(strcmp(cmd, "valve") == 0){

		//Se cambia el estado el estado del pin
		GPIOxTooglePin(&handlerValvePin);

		if(GPIO_ReadPin(&handlerValvePin) == 1){
			writeMsg(&handlerUsart1, "\nVálvula cerrada \n");
		}
		else{
			writeMsg(&handlerUsart1, "\nVálvula abierta \n");
		}
	}
	else{
		writeMsg(&handlerUsart1, "\nError!: Wrong command \n");
	}
}
void initSystem(void){

	//Configuración del Blinky
	handlerBlinky.pGPIOx 									= GPIOC;
	handlerBlinky.GPIO_PinConfig.GPIO_PinNumber 			= PIN_0;
	handlerBlinky.GPIO_PinConfig.GPIO_PinMode 				= GPIO_MODE_OUT;
	handlerBlinky.GPIO_PinConfig.GPIO_PinOPType 			= GPIO_OTYPE_PUSHPULL;
	handlerBlinky.GPIO_PinConfig.GPIO_PinSpeed 				= GPIO_OSPEEDR_FAST;
	handlerBlinky.GPIO_PinConfig.GPIO_PinPuPdControl 		= GPIO_PUPDR_NOTHING;
	GPIO_Config(&handlerBlinky);

	//Configuración del TIM2 (Blinky)
	handlerTimerBlinky.ptrTIMx 								= TIM2;
	handlerTimerBlinky.TIMx_Config.TIMx_mode 				= BTIMER_MODE_UP;
	handlerTimerBlinky.TIMx_Config.TIMx_speed 				= BTIMER_SPEED_100us;
	handlerTimerBlinky.TIMx_Config.TIMx_period 				= 2500;
	handlerTimerBlinky.TIMx_Config.TIMx_interruptEnable 	= 1;
	BasicTimer_Config(&handlerTimerBlinky);

	//Configuración del TIM5 (Data)
	handlerTimerData.ptrTIMx 								= TIM5;
	handlerTimerData.TIMx_Config.TIMx_mode 					= BTIMER_MODE_UP;
	handlerTimerData.TIMx_Config.TIMx_speed 				= BTIMER_SPEED_100us;
	handlerTimerData.TIMx_Config.TIMx_period 				= 30000;
	handlerTimerData.TIMx_Config.TIMx_interruptEnable 		= 1;
	BasicTimer_Config(&handlerTimerData);

	//Configuración comunicación I2C (ACCEL)
	//SDA
	handlerI2cSDA.pGPIOx									= GPIOB;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinNumber				= PIN_9;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OTYPE_OPENDRAIN;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEEDR_FAST;
	handlerI2cSDA.GPIO_PinConfig.GPIO_PinAltFunMode			= AF4;
	GPIO_Config(&handlerI2cSDA);

	//SCL
	handlerI2cSCL.pGPIOx									= GPIOB;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinNumber				= PIN_8;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinOPType				= GPIO_OTYPE_OPENDRAIN;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinPuPdControl		= GPIO_PUPDR_NOTHING;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinSpeed				= GPIO_OSPEEDR_FAST;
	handlerI2cSCL.GPIO_PinConfig.GPIO_PinAltFunMode			= AF4;
	GPIO_Config(&handlerI2cSCL);



	//Configuración de pines para USART2
	//TX Pin (USART2)
	handlerPinTX1.pGPIOx									= GPIOA;
	handlerPinTX1.GPIO_PinConfig.GPIO_PinNumber				= PIN_2;
	handlerPinTX1.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerPinTX1.GPIO_PinConfig.GPIO_PinAltFunMode			= AF7;
	GPIO_Config(&handlerPinTX1);

	//RX Pin (USART1)
	handlerPinRX1.pGPIOx									= GPIOA;
	handlerPinRX1.GPIO_PinConfig.GPIO_PinNumber				= PIN_3;
	handlerPinRX1.GPIO_PinConfig.GPIO_PinMode				= GPIO_MODE_ALTFN;
	handlerPinRX1.GPIO_PinConfig.GPIO_PinAltFunMode			= AF7;
	GPIO_Config(&handlerPinRX1);

	//Configuración de la comunicación serial USART1
	handlerUsart1.ptrUSARTx	 								= USART2;
	handlerUsart1.USART_Config.USART_baudrate				= USART_BAUDRATE_9600;
	handlerUsart1.USART_Config.USART_PLL_EN					= PLL_DISABLE;
	handlerUsart1.USART_Config.USART_datasize				= USART_DATASIZE_8BIT;
	handlerUsart1.USART_Config.USART_parity					= USART_PARITY_NONE;
	handlerUsart1.USART_Config.USART_stopbits				= USART_STOPBIT_1;
	handlerUsart1.USART_Config.USART_mode					= USART_MODE_RXTX;
	handlerUsart1.USART_Config.USART_enableIntTX			= USART_TX_INTERRUP_DISABLE;
	handlerUsart1.USART_Config.USART_enableIntRX			= USART_RX_INTERRUP_ENABLE;
	USART_Config(&handlerUsart1);



}
void initinitHDC1080(void){
	handlerI2C_SCL.pGPIOx                                   = GPIOB;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinNumber      		= PIN_8;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinMode       		= GPIO_MODE_ALTFN;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinAltFunMode  		= AF4;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinOPType 			= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinPuPdControl 		= GPIO_PUPDR_PULLUP;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinSpeed 			= GPIO_OSPEEDR_FAST;

	GPIO_Config(&handlerI2C_SCL);


	handlerI2C_SDA.pGPIOx                         			= GPIOB;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinNumber      		= PIN_9;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinMode       		= GPIO_MODE_ALTFN;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinAltFunMode  		= AF4;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinOPType  			= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinPuPdControl 		= GPIO_PUPDR_PULLUP;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinSpeed 			= GPIO_OSPEEDR_FAST;

	GPIO_Config(&handlerI2C_SDA);


	i2cSensor.ptrI2Cx    = I2C1;
	i2cSensor.modeI2C = I2C_MODE_FM;
	i2cSensor.slaveAddress    = SENSOR_ADD;

	i2c_Config(&i2cSensor);

	}

uint8_t time_conversion=0;
uint16_t config_resolution(HDC1080_Handler_t *ptrHandlerHDC);

//Configuracion de la resolucion de los datos y el tipo de toma de datos

void hdc1080_config(HDC1080_Handler_t *ptrHandlerHDC){

	//generamos la configuracion que deseamos cargar

	uint16_t df_config=config_resolution(ptrHandlerHDC);

	//Enviamos una transaccion de configuracion

	// 1) Generamos una condicion de Start

	i2c_startTransaction((ptrHandlerHDC->ptrI2Cx));

	// 2) Enviamos la direccion de memoria para configurar la resolcion de la toma de datos

	i2c_sendSlaveAddressRW((ptrHandlerHDC->ptrI2Cx), HDC1080_ADDRESS,HDC_WRITE);

	// 3) Enviamos la direccion de memoria donde se quire cargar la configuracion

	i2c_sendMemoryaddress((ptrHandlerHDC->ptrI2Cx), HDC1080_CONFIGURATION);

	// 4) Enviamos los primeros 8 bits de configuracion enviando primero MSB

	i2c_sendDataByte(ptrHandlerHDC->ptrI2Cx,(uint8_t) (df_config>>8));

	// 5) Enviamos la segundos 8 bits de configuracion

	i2c_sendDataByte(ptrHandlerHDC->ptrI2Cx, (uint8_t) df_config);

	// 6) Generamos el stop

	i2c_stopTransaction(ptrHandlerHDC->ptrI2Cx);

}

//Temperatura y humedad

uint32_t hdc1080_readTH(HDC1080_Handler_t *ptrHandlerHDC){

	// 0) Creamos dos varibles para cargar y guardar los valores recibidos

	uint32_t readData=0;

	uint8_t  bufferData[4]={0};

	// 1) Generamos un Restart

	i2c_reStartTransaction((ptrHandlerHDC->ptrI2Cx));

	// 2) Enviamos la direccion de meRegistermoria y R/W

	i2c_sendSlaveAddressRW(ptrHandlerHDC->ptrI2Cx, HDC1080_ADDRESS,HDC_WRITE);

	// 3) Enviamos la direccion de memoria que deseamos leer

	i2c_sendMemoryaddress(ptrHandlerHDC->ptrI2Cx, HDC1080_TEMPERATURE);

	// 4) Esperamos a que la conversion en el sensor se finali

	delay_ms(hdc1080_gettimeconversion());  //sumamos dos al tiempo para que no quede muy preciso

	// 5) Generamos un restar para inical la lectura

	i2c_reStartTransaction(ptrHandlerHDC->ptrI2Cx);

	// 6) Enviamos la direccion del sensor y la orden de leer

	i2c_sendSlaveAddressRW(ptrHandlerHDC->ptrI2Cx, HDC1080_ADDRESS,HDC_READ);

	// 7) leemos los primeros 8 bits de informacion relacionada ala temperatura

	bufferData[0]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);

	// 8) generamos un ACKM

	i2c_sendAck(ptrHandlerHDC->ptrI2Cx);

	// 9) leemos el segundo byte de informacion relacionado ala temperatura

	bufferData[1]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);

	// 10) Generamos un ACKM

	i2c_sendAck(ptrHandlerHDC->ptrI2Cx);

	// 11) Leemos el primer byte de informacion relacionado ala humedad

	bufferData[2]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);

	// 12) Generamos un ACKM

	i2c_sendAck(ptrHandlerHDC->ptrI2Cx);

	// 13) Leemos el primer byte de informacion relacionado ala humedad

	bufferData[3]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);

	// 14) Finalizamos la lectura de datos con NACKM

	i2c_sendNoAck(ptrHandlerHDC->ptrI2Cx);

	// 15) Por ultimo generamos el stop en la linea de datos

	i2c_stopTransaction(ptrHandlerHDC->ptrI2Cx);

	readData=(bufferData[0]<<24)|bufferData[1]<<16|bufferData[2]<<8|bufferData[3]<<0;

	return readData;

}
uint16_t hdc1080_readT(HDC1080_Handler_t *ptrHandlerHDC){

	// 0) Creamos dos varibles para cargar y guardar los valores recibidos

	uint16_t readData=0;

	uint8_t  bufferData[2]={0};

	// 1) Generamos un Restart

	i2c_reStartTransaction((ptrHandlerHDC->ptrI2Cx));

	// 2) Enviamos la direccion de memoria y R/W

	i2c_sendSlaveAddressRW(ptrHandlerHDC->ptrI2Cx, HDC1080_ADDRESS,HDC_WRITE);

	// 3) Enviamos la direccion de memoria que deseamos leer

	i2c_sendMemoryaddress(ptrHandlerHDC->ptrI2Cx, HDC1080_TEMPERATURE);

	// 4) Esperamos a que la conversion en el sensor se finali

	time_conversion=80;

	delay_ms(hdc1080_gettimeconversion());  //sumamos dos al tiempo para que no quede muy preciso



	// 5) Generamos un restar para inical la lectura

	i2c_reStartTransaction(ptrHandlerHDC->ptrI2Cx);

	// 6) Enviamos la direccion del sensor y la orden de leer

	i2c_sendSlaveAddressRW(ptrHandlerHDC->ptrI2Cx, HDC1080_ADDRESS,HDC_READ);

	// 7) leemos los primeros 8 bits de informacion relacionada ala temperatura

	bufferData[0]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);



	// 8) generamos un ACKM

	i2c_sendAck(ptrHandlerHDC->ptrI2Cx);

	 // 9) leemos el segundo byte de informacion relacionado ala temperatura

	 bufferData[1]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);

	 // 14) Finalizamos la lectura de datos con NACKM

	 i2c_sendNoAck(ptrHandlerHDC->ptrI2Cx);

	// 15) Por ultimo generamos el stop en la linea de datos

	 i2c_stopTransaction(ptrHandlerHDC->ptrI2Cx);

	readData=(bufferData[0]<<8|bufferData[1]<<0);

	return readData;

}
uint16_t hdc1080_readH(HDC1080_Handler_t *ptrHandlerHDC){

	// 0) Creamos dos varibles para cargar y guardar los valores recibidos

	uint16_t readData=0;

	uint8_t  bufferData[2]={0};

	// 1) Generamos un Restart

	i2c_startTransaction((ptrHandlerHDC->ptrI2Cx));

	// 2) Enviamos la direccion de memoria y R/W

	i2c_sendSlaveAddressRW(ptrHandlerHDC->ptrI2Cx, HDC1080_ADDRESS,HDC_WRITE);

	// 3) Enviamos la direccion de memoria que deseamos leer

	i2c_sendMemoryaddress(ptrHandlerHDC->ptrI2Cx, HDC1080_HUMIDITY);

	// 4) Esperamos a que la conversion en el sensor se finali

//	time_conversion=80;

//	delay_ms(hdc1080_gettimeconversion());  //sumamos dos al tiempo para que no quede muy preciso



	// 5) Generamos un restar para inical la lectura

	i2c_reStartTransaction(ptrHandlerHDC->ptrI2Cx);

	// 6) Enviamos la direccion del sensor y la orden de leer

	i2c_sendSlaveAddressRW(ptrHandlerHDC->ptrI2Cx, HDC1080_ADDRESS,HDC_READ);



	// 7) leemos los primeros 8 bits de informacion relacionada ala temperatura

	bufferData[0]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);

	// 8) generamos un ACKM

	i2c_sendAck(ptrHandlerHDC->ptrI2Cx);

	// 9) leemos el segundo byte de informacion relacionado ala temperatura

	bufferData[1]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);

	// 14) Finalizamos la lectura de datos con NACKM

	i2c_sendNoAck(ptrHandlerHDC->ptrI2Cx);

	// 15) Por ultimo generamos el stop en la linea de datos

	i2c_startTransaction(ptrHandlerHDC->ptrI2Cx);

	readData=(bufferData[0]<<8|bufferData[1]<<0);

	return readData;

}
uint16_t hdc1080_readID(HDC1080_Handler_t *ptrHandlerHDC){

	// 0) Creamos dos varibles para cargar y guardar los valores recibidos

	uint16_t readData=0;

	uint8_t  bufferData[3] = {0};

	// 1) Generamos un Restart

	i2c_startTransaction((ptrHandlerHDC->ptrI2Cx));

	// 2) Enviamos la direccion de memoria y R/W

	i2c_sendSlaveAddressRW(ptrHandlerHDC->ptrI2Cx, HDC1080_ADDRESS,HDC_WRITE);

	// 3) Enviamos la direccion de memoria que deseamos leer

	i2c_sendMemoryaddress(ptrHandlerHDC->ptrI2Cx, HDC1080_SERIAL_ID_FIRST);

	// 4) Esperamos a que la conversion en el sensor se finali

//	time_conversion=80;

//	delay_ms(hdc1080_gettimeconversion());  //sumamos dos al tiempo para que no quede muy preciso



	// 5) Generamos un restar para inical la lectura

	i2c_reStartTransaction(ptrHandlerHDC->ptrI2Cx);

	// 6) Enviamos la direccion del sensor y la orden de leer

	i2c_sendSlaveAddressRW(ptrHandlerHDC->ptrI2Cx, HDC1080_ADDRESS,HDC_READ);



	// 7) leemos los primeros 8 bits de informacion relacionada ala temperatura

	bufferData[0]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);

	// 8) generamos un ACKM

	i2c_sendAck(ptrHandlerHDC->ptrI2Cx);

	// 9) leemos el segundo byte de informacion relacionado ala temperatura

	bufferData[1]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);



	i2c_sendAck(ptrHandlerHDC->ptrI2Cx);

	// 9) leemos el segundo byte de informacion relacionado ala temperatura

	bufferData[2]=i2c_readDataByte(ptrHandlerHDC->ptrI2Cx);

	// 14) Finalizamos la lectura de datos con NACKM

	i2c_sendNoAck(ptrHandlerHDC->ptrI2Cx);

	// 15) Por ultimo generamos el stop en la linea de datos

	i2c_stopTransaction(ptrHandlerHDC->ptrI2Cx);

	readData=(bufferData[0]<<8|bufferData[1]<<0);

	return readData;

}
uint16_t config_resolution(HDC1080_Handler_t *ptrHandlerHDC){


	uint16_t df_config =0; //Definimos una variable para cargar la configuracion

	// Cargamos la resolucion  de humedad seleccionada en la variable

	switch (ptrHandlerHDC->HDC1080_Config.resolution_humi) {

		case HUMIDITY_RESOLUTION_14BIT:

			df_config&=~HDC1080_HUMIDITY_RES;

			time_conversion+=7;  //la conversion de la humedad se demora 7ms

			break;

		case HUMIDITY_RESOLUTION_11BIT:

			df_config|=HDC1080_HUMIDITY_RES_0;

			time_conversion+=4;   //la conversion de la humedad se demora 4ms

			break;

		case HUMIDITY_RESOLUTION_8BIT:

			df_config|=HDC1080_HUMIDITY_RES_1;

			time_conversion+=3;   //la conversion de la humedad se demora 3ms

			break;

		default:

			break;

	}

	switch (ptrHandlerHDC->HDC1080_Config.resolution_temp) {

		case TEMPERATURE_RESOLUTION_14BIT:

			df_config&=~HDC1080_TEMPERATURE_RES;

			time_conversion+=7;   //la conversion de la temperatura se demora 7ms

			break;

		case TEMPERATURE_RESOLUTION_11BIT:

			df_config|=HDC1080_TEMPERATURE_RES;

			time_conversion+=4;   //la conversion de la temperatura se demora 4ms

		default:

			break;

	}

	switch (ptrHandlerHDC->HDC1080_Config.type_take_data) {



		case TEMPERATURE_OR_HUMIDITY:

			df_config&=~HDC1080_TYPE_TAKE_DATA;

			break;

		case TEMPERATURE_AND_HUMIDITY:

			df_config|=HDC1080_TYPE_TAKE_DATA;

			break;

		default:

			break;

	}

	return df_config;

}

void readTemperature(void){
	i2c_reStartTransaction(&i2cSensor);

	i2c_sendSlaveAddressRW(&i2cSensor, SENSOR_ADD, 0);

	i2c_sendMemoryaddress(&i2cSensor, TEMPERATURE);

	i2c_stopTransaction(&i2cSensor);
	//gpio_WritePin(&pinLedOK, SET);
	delay_ms(20);
	//gpio_WritePin(&pinLedOK, RESET);

	i2c_startTransaction(&i2cSensor);

	i2c_sendSlaveAddressRW(&i2cSensor, SENSOR_ADD, 1);

	/* 6. Activamos el envio de Ack */
	i2c_sendAck(&i2cSensor);

	/* 8. Leemos el dato que envia el esclavo */
	i2cBuffer[0] = i2c_readDataByte(&i2cSensor);

	/* 6. Generamos la condición de No-Ack para detener el envio de datos*/
	i2c_sendNoAck(&i2cSensor);

	/* Generamos la indicación para terminar la transacción*/
	i2c_stopTransaction(&i2cSensor);

	/* 8. Leemos el ultimo dato que envia el esclavo */
	i2cBuffer[1] = i2c_readDataByte(&i2cSensor);

}

void usart2Rx_Callback(void){
	//Leemos el valor del registro DR, donde se almacena el dato que llega.
	rxData = getRxData();
}

void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinky);
}

void BasicTimer5_Callback(void){

	//Bandera que se levanta cada medio segundo para mostrar los datos en consola
	flagData = 1;
}
