/*******************************************************************************
SORACOM LoRaWAN用 LCD HD44780 ドライバ 動作確認用サンプルソフトウェア
 - STM32L0用
                                                Copyright (c) 2018 Wataru KUNINO
********************************************************************************
本ソースリストは
STMicroelectronics が配布する下記のソフトウェアを使用し、
株式会社ソラコム が日本仕様ならびにSORACOM LoRaWANへ対応したものを、
国野亘 がLCD表示に対応させたものです。

I-CUBE-LRWAN
LoRaWAN software expansion for STM32Cube (UM2073)
http://www.st.com/ja/embedded-software/i-cube-lrwan.html
********************************************************************************

*******************************************************************************/

 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    01-June-2017
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "lcd_drv_hd44780.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*!
 * CAYENNE_LPP is myDevices Application server.
 */
//#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73

#define LPP_APP_PORT 99

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            10000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0
/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG                    ENABLE
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            3

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa will transmit a frame*/
static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed);

/* call back when LoRa has received a frame*/
static void LoraRxData( lora_AppData_t *AppData);

#ifdef TRACE
static void Print_App_Settings( void );
extern void Print_Lora_Settings( void );
static void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);
static bool printedOnce = 0;
static bool printSensorData = 0;
#endif

/* Private variables ---------------------------------------------------------*/
/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LoraTxData,
                                               LoraRxData};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;
static char AppLcdData[17];
static DeviceState_t AppDeviceState;

static uint16_t pressure = 0;
static int16_t temperature = 0;
static uint16_t humidity = 0;
static uint8_t batteryLevel = 1;
static sensor_t sensor_data;

/* added for Soracom 2017.9.4 */
#ifdef APPKEY_USE_TRNG
RNG_HandleTypeDef hrng;
#endif
                                               
#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx and Rx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static TimerEvent_t RxLedTimer;
static void OnTimerLedEvent( void );
static void OnTimerRxLedEvent( void );
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {TX_ON_TIMER,
                                    APP_TX_DUTYCYCLE,
                                    CLASS_A,
                                    LORAWAN_ADR_ON,
                                    DR_0,
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

uint32_t Forced_RX1_Window_ms = 800; // **KC**

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  /* STM32 HAL library initialization*/
  HAL_Init( );
  
  /* Configure the system clock*/
  SystemClock_Config( );
  
  /* Configure the debug mode*/
  DBG_Init( );
  
  /* Configure the hardware*/
  HW_Init( );
  
  /* USER CODE BEGIN 1 */
  lcd_init();
  
  #ifdef TRACE
    Print_App_Settings();
  #endif
  /* added for Soracom 2017.9.4 */
  #ifdef APPKEY_USE_TRNG
  __HAL_RCC_RNG_CLK_ENABLE();
  hrng.Instance = RNG;
  HAL_RNG_Init(&hrng);
  #endif
  /* USER CODE END 1 */
  
  /* Configure the Lora Stack*/
  lora_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  PRINTF("VERSION: %X\n\r", VERSION);
  lcd_putstr("V.");
  lcd_disp_hex((unsigned char)VERSION);
  lcd_putstr(" -- -- -- --");
  lcd_goto_line(2);
  lcd_putstr("LoRaWAN End_Node");
  AppLcdData[0]='\0';
  AppDeviceState=DEVICE_STATE_INIT;
  /*  LCD
      - 0123456789012345
      1 V.00 TX RX GW PS
  */
  
  /* main loop*/
  while( 1 )
  {
    /* run the LoRa class A state machine*/
    lora_fsm( );
    
    #ifdef TRACE
      if( printedOnce == 0 )
      {
        if ( lora_getDeviceState( ) == DEVICE_STATE_JOIN )
        {
          Print_Lora_Settings();
          printedOnce = 1;
        }
      }
    #endif      
    
    #ifdef TRACE  
      if ( printSensorData == 1 )
      {
        printSensorData = 0;
        int32_t d1, d2;
        // Print sensor data to local debug terminal
        PRINTF("\r\nSensor Data:\r\n");
        
        lcd_goto_line(2);
        floatToInt(sensor_data.temperature, &d1, &d2, 2);
        DBG_PRINTF("\tTemperature: %d.%02d \370C\r\n", d1,d2);
        if(d1<0){
          lcd_putch('-');
          d1 *= -1;
        }
        if(d1<10) lcd_disp_1(d1); else lcd_disp_2(d1);
        lcd_putch(0xDF);
        lcd_putstr("C ");
        floatToInt(sensor_data.humidity, &d1, &d2, 2);
        DBG_PRINTF("\tHumidity: %d.%02d %%\r\n", d1,d2);
        if(d1<10) lcd_disp_1(d1); else lcd_disp_2(d1);
        lcd_putstr("% ");
        floatToInt(sensor_data.pressure, &d1, &d2, 2);
        DBG_PRINTF("\tPressure: %d.%02d hPa\r\n", d1,d2);
        if(d1 >= 1000 ){
          lcd_putch('1');
          d1 -= 1000;
        }
        lcd_disp_3(d1);
        lcd_putstr("hPa   ");
      }
    #endif      
    switch( lora_getDeviceState() ){
      case DEVICE_STATE_INIT:
        AppDeviceState=DEVICE_STATE_INIT;
        lcd_goto_line(1);
        lcd_goto(11);
        lcd_putstr("-- --");
        break;
      case DEVICE_STATE_JOIN:
        AppDeviceState=DEVICE_STATE_JOIN;
        lcd_goto_line(1);
        lcd_goto(11);
        lcd_putstr("-- NC");
        break;
      case DEVICE_STATE_JOINED:
        AppDeviceState=DEVICE_STATE_JOINED;
        lcd_goto_line(1);
        lcd_goto(11);
        lcd_putstr("GW OK");
        break;
      case DEVICE_STATE_SEND:
        TimerInit( &TxLedTimer, OnTimerLedEvent );
        TimerSetValue(  &TxLedTimer, 1000);
        lcd_goto_line(1);
        lcd_goto(5);
        lcd_putstr("SD");
        TimerStart( &TxLedTimer );
        break;
      case DEVICE_STATE_CYCLE:
        lcd_goto_line(1);
        lcd_goto(14);
        lcd_putstr("CY");
        break;
      case DEVICE_STATE_SLEEP:
        break;
      default:
        break;
    }
    if( AppDeviceState != DEVICE_STATE_JOINED ) continue;
    
    DISABLE_IRQ( );
    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending 
     * and cortex will not enter low power anyway  */
    if ( lora_getDeviceState( ) == DEVICE_STATE_SLEEP )
    {
#ifndef LOW_POWER_DISABLE
      LowPower_Handler( );
#endif
    }
    ENABLE_IRQ();
    
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}

static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed)
{
  /* USER CODE BEGIN 3 */

#ifdef USE_B_L072Z_LRWAN1
  TimerInit( &TxLedTimer, OnTimerLedEvent );
  TimerSetValue(  &TxLedTimer, 5000);
  // LED_On( LED_RED1 ) ; 
  lcd_goto_line(1);
  lcd_goto(5);
  lcd_putstr("TX");
  if( AppDeviceState == DEVICE_STATE_JOINED ) lcd_putstr(" -- GW OK");
  TimerStart( &TxLedTimer );  
#endif  
  
#ifndef CAYENNE_LPP
  int32_t latitude, longitude = 0;
  uint16_t altitudeGps = 0;
#endif
  BSP_sensor_Read( &sensor_data );

#ifdef CAYENNE_LPP
  uint8_t cchannel=0;
  temperature = ( int16_t )( sensor_data.temperature * 10 );     /* in ｰC * 10 */
  pressure    = ( uint16_t )( sensor_data.pressure * 100 / 10 );  /* in hPa / 10 */
  humidity    = ( uint16_t )( sensor_data.humidity * 2 );        /* in %*2     */
  uint32_t i = 0;

  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData->Port = LPP_APP_PORT;
  
  *IsTxConfirmed =  LORAWAN_CONFIRMED_MSG;
  AppData->Buff[i++] = cchannel++;
  AppData->Buff[i++] = LPP_DATATYPE_BAROMETER;
  AppData->Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData->Buff[i++] = pressure & 0xFF;
  AppData->Buff[i++] = cchannel++;
  AppData->Buff[i++] = LPP_DATATYPE_TEMPERATURE; 
  AppData->Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData->Buff[i++] = temperature & 0xFF;
  AppData->Buff[i++] = cchannel++;
  AppData->Buff[i++] = LPP_DATATYPE_HUMIDITY;
  AppData->Buff[i++] = humidity & 0xFF;
#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 )
  /* The maximum payload size does not allow to send more data for lowest DRs */
#else
  AppData->Buff[i++] = cchannel++;
  AppData->Buff[i++] = LPP_DATATYPE_DIGITAL_INPUT; 
  AppData->Buff[i++] = batteryLevel*100/254;
  AppData->Buff[i++] = cchannel++;
  AppData->Buff[i++] = LPP_DATATYPE_DIGITAL_OUTPUT; 
  AppData->Buff[i++] = AppLedStateOn;
#endif  /* REGION_XX915 */
#else  /* not CAYENNE_LPP */

  temperature = ( int16_t )( sensor_data.temperature * 100 );     /* in ｰC * 100 */
  pressure    = ( uint16_t )( sensor_data.pressure * 100 / 10 );  /* in hPa / 10 */
  humidity    = ( uint16_t )( sensor_data.humidity * 10 );        /* in %*10     */
  latitude = sensor_data.latitude;
  longitude= sensor_data.longitude;
  uint32_t i = 0;

  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData->Port = LORAWAN_APP_PORT;
  
  *IsTxConfirmed =  LORAWAN_CONFIRMED_MSG;

#if defined( REGION_US915 ) || defined( REGION_US915_HYBRID ) || defined ( REGION_AU915 )
  AppData->Buff[i++] = AppLedStateOn;
  AppData->Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData->Buff[i++] = pressure & 0xFF;
  AppData->Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData->Buff[i++] = temperature & 0xFF;
  AppData->Buff[i++] = ( humidity >> 8 ) & 0xFF;
  AppData->Buff[i++] = humidity & 0xFF;
  AppData->Buff[i++] = batteryLevel;
  AppData->Buff[i++] = 0;
  AppData->Buff[i++] = 0;
  AppData->Buff[i++] = 0;
#else  /* not REGION_XX915 */
  AppData->Buff[i++] = AppLedStateOn;
  AppData->Buff[i++] = ( pressure >> 8 ) & 0xFF;
  AppData->Buff[i++] = pressure & 0xFF;
  AppData->Buff[i++] = ( temperature >> 8 ) & 0xFF;
  AppData->Buff[i++] = temperature & 0xFF;
  AppData->Buff[i++] = ( humidity >> 8 ) & 0xFF;
  AppData->Buff[i++] = humidity & 0xFF;
  AppData->Buff[i++] = batteryLevel;
  AppData->Buff[i++] = ( latitude >> 16 ) & 0xFF;
  AppData->Buff[i++] = ( latitude >> 8 ) & 0xFF;
  AppData->Buff[i++] = latitude & 0xFF;
  AppData->Buff[i++] = ( longitude >> 16 ) & 0xFF;
  AppData->Buff[i++] = ( longitude >> 8 ) & 0xFF;
  AppData->Buff[i++] = longitude & 0xFF;
  AppData->Buff[i++] = ( altitudeGps >> 8 ) & 0xFF;
  AppData->Buff[i++] = altitudeGps & 0xFF;
#endif  /* REGION_XX915 */
#endif  /* CAYENNE_LPP */
  //AppData->BuffSize = i;
  AppData->BuffSize = 8;
#ifdef TRACE
  printSensorData = 1;
#endif
  
  /* USER CODE END 3 */
}
    
static void LoraRxData( lora_AppData_t *AppData )
{
  int i;
#ifdef USE_B_L072Z_LRWAN1
  TimerInit( &RxLedTimer, OnTimerRxLedEvent );
  TimerSetValue(  &RxLedTimer, 5000);
  lcd_goto_line(1);
  lcd_goto(8);
  lcd_putstr("RX GW OK");
  TimerStart( &RxLedTimer );  
#endif  
  
  /* USER CODE BEGIN 4 */
  switch (AppData->Port)
  {
  case LORAWAN_APP_PORT:
    if( AppData->BuffSize >= 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        // LED_Off( LED_BLUE ) ; 
        LED_Off( LED2 ) ;  // Digital 13  PA5 
      }
      else
      {
        PRINTF("LED ON\n\r");
        // LED_On( LED_BLUE ) ; 
        LED_On( LED2 ) ;   // Digital 13  PA5 
      }
      //GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
      
      // LCD Output
      for(i=0;i<AppData->BuffSize;i++){
        AppLcdData[i]=AppData->Buff[i];
        if( i >= 15 ) break;
      }
      for(;i<15;i++){
        AppLcdData[i]=' ';
      }
      AppLcdData[15]='\0';
      lcd_goto_line(2);
      lcd_putstr(AppLcdData);
    }
    break;
  case LPP_APP_PORT:
  {
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
        // LED_Off( LED_BLUE ) ; 
        LED_Off( LED2 ) ;  // Digital 13  PA5 
      }
      else
      {
        PRINTF("LED ON\n\r");
        // LED_On( LED_BLUE ) ; 
        LED_On( LED2 ) ;   // Digital 13  PA5 
      }
    break;
  }
  default:
    break;
  }
  /* USER CODE END 4 */
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void )
{
  // LED_Off( LED_RED1 ) ; 
  lcd_goto_line(1);
  lcd_goto(5);
  lcd_putstr("--");
  if(AppLcdData[0]){
      lcd_goto_line(2);
      lcd_putstr(AppLcdData);
  }
}

static void OnTimerRxLedEvent( void )
{
  lcd_goto_line(1);
  lcd_goto(8);
  lcd_putstr("--");
}
#endif

#ifdef TRACE
static void Print_App_Settings( void )
{
  DBG_PRINTF("----- App Settings -----\n\r");
  DBG_PRINTF("\tDevEUI Source: %s\n\r", ((STATIC_DEVICE_EUI == 1)?"Static":"STM32 Unique ID"));
  #if (OVER_THE_AIR_ACTIVATION == 1)
    DBG_PRINTF("\tDevAddr Source: Automatic (OTAA)\n\r");
  #else
    DBG_PRINTF("\tDevAddr Source: %s\n\r", ((STATIC_DEVICE_ADDRESS == 1)?"Static":"STM32 Unique ID"));
  #endif
  #ifdef DEBUG 
    DBG_PRINTF("\tDebug: Enabled\n\r");
  #endif
  #ifdef TRACE 
    DBG_PRINTF("\tTrace: Enabled\n\r");
  #endif
  #ifdef LOW_POWER_DISABLE 
    DBG_PRINTF("\tLow Power Mode: Disabled\n\r");
  #else
    DBG_PRINTF("\tLow Power Mode: Enabled\n\r");
  #endif 
  #ifdef SENSOR_ENABLED 
    DBG_PRINTF("\tSensors: Enabled\n\r");
  #else
    DBG_PRINTF("\tSensors: Disabled\n\r");
  #endif     
  DBG_PRINTF("\tData Uplink Type: %s\n\r", ((LORAWAN_CONFIRMED_MSG == 1)?"Confirmed":"Unconfirmed"));
  DBG_PRINTF("\tADR: %s\n\r", ((LORAWAN_ADR_ON == 1)?"Enabled":"Disabled"));  
  DBG_PRINTF("\tJoin Request Trials: %d\n\r", JOINREQ_NBTRIALS);
  DBG_PRINTF("------------------------\n\r\n");
}


/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec)
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
