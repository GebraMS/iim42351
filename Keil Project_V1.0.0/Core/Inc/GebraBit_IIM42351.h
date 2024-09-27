/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
#ifndef	_IIM42652__H_
#define	_IIM42652__H_
#include "main.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "spi.h"
#include "string.h"
#include "RingBuffer.h"
#include "math.h"
/************************************************
 *         USER BANK 0 REGISTER MAP             *
 ***********************************************/ 
#define IIM42351_DEVICE_CONFIG     		 	0x11
#define IIM42351_DRIVE_CONFIG      		 	0x13
#define IIM42351_INT_CONFIG         		0x14
#define IIM42351_FIFO_CONFIG        		0x16
#define IIM42351_TEMP_DATA1         		0x1D
#define IIM42351_TEMP_DATA0         		0x1E
#define IIM42351_ACCEL_DATA         		0x1F
#define IIM42351_ACCEL_DATA_X1          31
#define IIM42351_ACCEL_DATA_X0          32
#define IIM42351_ACCEL_DATA_Y1        	33
#define IIM42351_ACCEL_DATA_Y0        	34
#define IIM42351_ACCEL_DATA_Z1       	  35
#define IIM42351_ACCEL_DATA_Z0       	  36
#define IIM42351_TMST_FSYNCH        		0x2B
#define IIM42351_INT_STATUS         		0x2D
#define IIM42351_FIFO_COUNTH        		0x2E
#define IIM42351_FIFO_COUNTL        		0x2F
#define IIM42351_FIFO_DATA          		0x30
#define IIM42351_APEX_DATA0         		0x31
#define IIM42351_APEX_DATA1         		0x32
#define IIM42351_APEX_DATA2         		0x33
#define IIM42351_APEX_DATA3         		0x34
#define IIM42351_APEX_DATA4         		0x35
#define IIM42351_APEX_DATA5         		0x36
#define IIM42351_INT_STATUS2        		0x37
#define IIM42351_INT_STATUS3        		0x38
#define IIM42351_SIGNAL_PATH_RESET  		0x4B
#define IIM42351_INTF_CONFIG0       		0x4C
#define IIM42351_INTF_CONFIG1       		0x4D
#define IIM42351_PWR_MGMT0          		0x4E
#define IIM42351_ACCEL_CONFIG0      		0x50
#define IIM42351_TEMP_FILT_CONFIG     	0x51
#define IIM42351_ACCEL_FILT_CONFIG 			0x52
#define IIM42351_ACCEL_CONFIG1      		0x53
#define IIM42351_TMST_CONFIG        		0x54
#define IIM42351_APEX_CONFIG0       		0x56
#define IIM42351_SMD_CONFIG         		0x57
#define IIM42351_FIFO_CONFIG1       		0x5F
#define IIM42351_FIFO_CONFIG2       		0x60
#define IIM42351_FIFO_CONFIG3       		0x61
#define IIM42351_FSYNC_CONFIG       		0x62
#define IIM42351_INT_CONFIG0        		0x63
#define IIM42351_INT_CONFIG1        		0x64
#define IIM42351_INT_SOURCE0        		0x65
#define IIM42351_INT_SOURCE1        		0x66
#define IIM42351_INT_SOURCE2        		0x67
#define IIM42351_INT_SOURCE3        		0x68
#define IIM42351_INT_SOURCE4        		0x69
#define IIM42351_FIFO_LOST_PKT0     		0x6C
#define IIM42351_FIFO_LOST_PKT1     		0x6D
#define IIM42351_SELF_TEST_CONFIG  	 		0x70
#define IIM42351_WHO_AM_I            	  0x75
#define IIM42351_REG_BANK_SEL       	  0x76
/*----------------------------------------------*
 *        USER BANK 0 REGISTER MAP End          *
 *----------------------------------------------*/ 
/************************************************
 *         USER BANK 1 REGISTER MAP             *
 ***********************************************/ 
#define IIM42351_SENSOR_CONFIG0					0x03
#define IIM42351_GYRO_CONFIG_STATIC2 		0x0B
#define IIM42351_GYRO_CONFIG_STATIC3	 	0x0C
#define IIM42351_GYRO_CONFIG_STATIC4	 	0x0D
#define IIM42351_GYRO_CONFIG_STATIC5 		0x0E
#define IIM42351_GYRO_CONFIG_STATIC6 		0x0F
#define IIM42351_GYRO_CONFIG_STATIC7 		0x10
#define IIM42351_GYRO_CONFIG_STATIC8 		0x11
#define IIM42351_GYRO_CONFIG_STATIC9 		0x12
#define IIM42351_GYRO_CONFIG_STATIC10		0x13 
#define IIM42351_XG_ST_DATA							0x5F
#define	IIM42351_YG_ST_DATA							0x60
#define IIM42351_ZG_ST_DATA							0x61
#define IIM42351_TMSTVAL0								0x62
#define	IIM42351_TMSTVAL1								0x63
#define	IIM42351_TMSTVAL2								0x64
#define IIM42351_INTF_CONFIG4						0x7A
#define IIM42351_INTF_CONFIG5						0x7B
#define IIM42351_INTF_CONFIG6						0x7C
/*----------------------------------------------*
 *        USER BANK 1 REGISTER MAP End          *
 *----------------------------------------------*/ 
/************************************************
 *         USER BANK 2 REGISTER MAP             *
 ***********************************************/ 
#define IIM42351_ACCEL_CONFIG_STATIC2		0x03
#define IIM42351_ACCEL_CONFIG_STATIC3		0x04
#define IIM42351_ACCEL_CONFIG_STATIC4		0x05
#define IIM42351_XA_ST_DATA							0x3B
#define IIM42351_YA_ST_DATA							0x3C
#define IIM42351_ZA_ST_DATA							0x3D
/*----------------------------------------------*
 *        USER BANK 2 REGISTER MAP End          *
 *----------------------------------------------*/ 
/************************************************
 *         USER BANK 3 REGISTER MAP             *
 ***********************************************/ 
#define IIM42351_PU_PD_CONFIG1					0x06
#define IIM42351_PU_PD_CONFIG2					0x0E
/*----------------------------------------------*
 *        USER BANK 3 REGISTER MAP End          *
 *----------------------------------------------*/ 
/************************************************
 *         USER BANK 4 REGISTER MAP             *
 ***********************************************/ 
#define IIM42351_FDR_CONFIG						 0x09
#define IIM42351_APEX_CONFIG1          0x40
#define	IIM42351_APEX_CONFIG2          0x41
#define IIM42351_APEX_CONFIG3          0x42
#define IIM42351_APEX_CONFIG4          0x43
#define	IIM42351_APEX_CONFIG5          0x44
#define	IIM42351_APEX_CONFIG6          0x45
#define IIM42351_APEX_CONFIG7          0x46
#define IIM42351_APEX_CONFIG8          0x47
#define IIM42351_APEX_CONFIG9          0x48
#define IIM42351_APEX_CONFIG10         0x49
#define IIM42351_ACCEL_WOM_X_THR       0x4A
#define IIM42351_ACCEL_WOM_Y_THR       0x4B
#define IIM42351_ACCEL_WOM_Z_THR       0x4C
#define	IIM42351_INT_SOURCE6           0x4D
#define IIM42351_INT_SOURCE7           0x4E
#define IIM42351_INT_SOURCE8           0x4F
#define	IIM42351_INT_SOURCE9           0x50
#define	IIM42351_INT_SOURCE10          0x51
#define IIM42351_OFFSET_USER0          0x77
#define IIM42351_OFFSET_USER1          0x78
#define IIM42351_OFFSET_USER2          0x79
#define IIM42351_OFFSET_USER3          0x7A
#define IIM42351_OFFSET_USER4          0x7B
#define IIM42351_OFFSET_USER5          0x7C
#define IIM42351_OFFSET_USER6          0x7D
#define IIM42351_IIM42351_OFFSET_USER7 0x7E
#define IIM42351_OFFSET_USER8          0x7F
/*----------------------------------------------*
 *        USER BANK 4 REGISTER MAP End          *
 *----------------------------------------------*/ 

/************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
 /************************************************
 *          Register Values Begin                *
 ***********************************************/ 
#define I3C_DISABLE                    0
#define DO_NOT_TAG_FSYNC_FLAG          0
#define FSYNC_TIME_STAMP_DISABLE       0
#define ENABLE_ALL_PACKETS_TO_FIFO    13
#define DISABLE_ALL_PACKETS_TO_FIFO    0
/*----------------------------------------------*
 *           Register Values End                *
 *----------------------------------------------*/
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/**************************************************
 * Values For BANK_SEL in REG_BANK_SEL Register   *
 **************************************************/ 
typedef enum bank_sel
{  
	BANK_0 = 0 ,                   								   /* Register Bank 0 selection */
	BANK_1     ,                    							   /* Register Bank 1 selection */
	BANK_2     ,                     								 /* Register Bank 2 selection */
	BANK_3     ,                     								 /* Register Bank 3 selection */ 
	BANK_4     
}IIM42351_Bank_Sel;
/****************************************************
 * Values For SPI_AP_4WIRE in INTF_CONFIG4 Register *
 ****************************************************/ 
typedef enum interface
{  
	NOT_SPI = 0     ,                  						   /* 0: AP interface uses 3-wire SPI mode           */ 
	IS_SPI                                 					 /* 1: AP interface uses 4-wire SPI mode (default) */ 
}IIM42351_Interface;
/**********************************************************
 * Values For SOFT_RESET_CONFIG in DEVICE_CONFIG Register *
 **********************************************************/ 
typedef enum Soft_Reset_Config
{
	IIM42351_RESET   = 0x01,                         /* 0: Normal (default) */
	IIM42351_NOT_RESET = 0x00,                       /* 1: Enable reset After writing 1 to this bitfield, wait 1ms for soft reset to be effective, before attempting any other register access */
} IIM42351_Soft_Reset_Config;  
/*****************************************************
 * Values For PIN9_FUNCTION in INTF_CONFIG5 Register *
 *****************************************************/ 
typedef enum Pin9_Function
{
	INT2   = 0,                 										 /* 00: INT2  functionalities for pin 9 */
	FSYNC  = 1,																			 /* 00: FSYNC functionalities for pin 9 */
	CLKIN  = 2																			 /* 00: CLKIN functionalities for pin 9 */
} IIM42351_PIN9_FUNCTION;
/******************************************************
 * Values For ACCEL_FS_SEL in ACCEL_CONFIG0  Register *
 ******************************************************/ 
typedef enum accel_fs_sel
{  
	FS_16g = 0 ,                    							  /* 000: �16g Full scale select for accelerometer */
	FS_8g      ,                							      /* 001: �8g  Full scale select for accelerometer */
	FS_4g      ,               								      /* 010: �4g  Full scale select for accelerometer */
	FS_2g            																/* 011: �2g  Full scale select for accelerometer */
}IIM42351_Accel_Fs_Sel;
/**************************************************
 *           Values For Scale_Factor              *
 **************************************************/ 
typedef enum 
{  
	SCALE_FACTOR_2048_LSB_g  = 2048    ,           
	SCALE_FACTOR_4096_LSB_g  = 4096    ,               
	SCALE_FACTOR_8192_LSB_g  = 8192    ,                     
	SCALE_FACTOR_16384_LSB_g = 16384           
}IIM42351_Accel_Scale_Factor;
/**************************************************
 * Values For ACCEL_ODR in ACCEL_CONFIG0 Register *
 **************************************************/ 
typedef enum accel_odr
{  
	ODR_8KHz    = 3,   														 /* 0011: 8kHz (LN mode)          */
	ODR_4KHz    = 4,    													 /* 0100: 4kHz (LN mode)          */
	ODR_2KHz    = 5,    													 /* 0101: 2kHz (LN mode)          */
	ODR_1KHz    = 6,    													 /* 0110: 1kHz (LN mode) (default)*/
	ODR_200Hz   = 7,    													 /* 0111: 200Hz (LP or LN mode)   */
	ODR_100Hz   = 8,    													 /* 1000: 100Hz (LP or LN mode)   */
	ODR_50Hz    = 9,    													 /* 1001: 50Hz (LP or LN mode)    */
	ODR_25Hz    = 10,   													 /* 1010: 25Hz (LP or LN mode)    */
	ODR_12Hz5   = 11,  														 /* 1011: 12.5Hz (LP or LN mode)  */
	ODR_6Hz25   = 12,   										  		 /* 1100: 6.25Hz (LP mode)        */
	ODR_3Hz125  = 13,  													   /* 1101: 3.125Hz (LP mode)       */
	ODR_1Hz5625 = 14,   								 					 /* 1110: 1.5625Hz (LP mode)      */
	ODR_500Hz   = 15    							 		 	   		 /*1111: 500Hz (LP or LN mode)    */
}IIM42351_Accel_ODR;
/*************************************************
 * Values For FIFO_MODE in FIFO_CONFIG Register  *
 **************************************************/ 
typedef enum FIFO_Config
{  
	BYPASS = 0 ,                                    /* 00: Bypass Mode (default) */
	STREAM_TO_FIFO      ,                           /*01: Stream-to-FIFO Mode    */
	STOP_ON_FULL                                    /*10: STOP-on-FULL Mode      */
}IIM42351_FIFO_MODE ;
/**************************************************
 *     Values For Disable And Enable Functions    *
 **************************************************/ 
typedef enum 
{  
	Disable = 0     ,                      
	Enable     
}IIM42351_Ability;
/*************************************************
 * Values For TMST_RES in TMST_CONFIG Register   *
 **************************************************/ 
typedef enum timestamp_resolution
{
	_1_uS   = 0 ,																		/* Time Stamp resolution: When set to 0 (default), time stamp resolution is 1 �s. When*/
	_16_uS                                          /* set to 1, resolution is 16�s */
} IIM42351_Timestamp_Resolution;
/****************************************************************************
 * Values For FIFO_COUNT_ENDIAN&SENSOR_DATA_ENDIAN in INTF_CONFIG0 Register *
 ****************************************************************************/ 
typedef enum                            
{  
	LITTLE = 0     ,                                 /* 0: FIFO count is reported in Little Endian format        */                     
	BIG     																		 		 /* 1: FIFO count is reported in Big Endian format (default) */
}IIM42351_Data_Endian;
/******************************************************
 * Values For FIFO_COUNT_REC in INTF_CONFIG0 Register *
 ******************************************************/ 
typedef enum 
{  
	IN_BYTES = 0     ,                               /* 0: FIFO count is reported in bytes        */  
	IN_RECORDS                                       /* 1: FIFO count is reported in records (1 record = 16 bytes for header + accel + temp */
}IIM42351_FIFO_Counting;                           /* sensor data + time stamp, or 8 bytes for header + accel + temp sensor data)          */
/********************************************************
 * Values For ACCEL_UI_FILT_ORD in MODE_SELECT Register *
 ********************************************************/ 
typedef enum 
{  
	_1_ORDER = 0  ,                                	 /* 00: 1st Order for ACCEL UI filter */
	_2_ORDER      ,                    							 /* 01: 2nd Order for ACCEL UI filter */
	_3_ORDER                                         /* 10: 3rd Order for ACCEL UI filter */
}IIM42351_UI_Filter_Order ;
/**************************************************
 * Values For ACCEL_MODE in PWR_MGMT0 Register    *
 **************************************************/ 
typedef enum
{
	IIM42351_LOW_NOISE  = 0x03,        							 /* 11: Places accelerometer in Low Noise (LN) Mode */
	IIM42351_LOW_POWER  = 0x02,											 /* 10: Places accelerometer in Low Power (LP) Mode */
	IIM42351_ACCEL_OFF  = 0x01											 /* 01: Turns accelerometer off                     */
} IIM42351_Power_Mode;
/**************************************************************
 * Values For ACCEL_UI_FILT_BW in  ACCEL_FILT_CONFIG Register *
 **************************************************************/ 
typedef enum
{
	LN_FILTER_BW_40 = 0x7 ,													 /* 7 BW=max(400Hz, ODR)/40 				 */
	LN_FILTER_BW_20 = 0x6 ,													 /* 6 BW=max(400Hz, ODR)/20 				 */
	LN_FILTER_BW_16 = 0x5 ,													 /* 5 BW=max(400Hz, ODR)/16 				 */
	LN_FILTER_BW_10 = 0x4 ,													 /* 4 BW=max(400Hz, ODR)/10 				 */
	LN_FILTER_BW_8  = 0x3 ,													 /* 3 BW=max(400Hz, ODR)/8 					 */
	LN_FILTER_BW_5  = 0x2 ,													 /* 2 BW=max(400Hz, ODR)/5 					 */
	LN_FILTER_BW_4  = 0x1 ,													 /* 1 BW=max(400Hz, ODR)/4 (default) */
	LN_FILTER_BW_2  = 0x0 													 /* 0 BW=ODR/2                       */
} IIM42351_Low_Noise_Filter_BW;
/**************************************************************
 * Values For ACCEL_UI_FILT_BW in ACCEL_FILT_CONFIG Register *
 **************************************************************/ 
typedef enum
{
	 LP_1x_AVG_FILTER  = 0x1 ,										   /* 1 1x AVG filter (default) 				 */
	 LP_16x_AVG_FILTER = 0x6                         /* 6 16x AVG filter          				 */
} IIM42351_Low_Power_Filter_AVG;
/*************************************************
 *         Values For Data Preparation           *
 **************************************************/ 
typedef enum 
{  
	IS_NOT_Ready = 0     ,                      
	IS_Ready     
}IIM42351_Preparation;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum 
{  
	FAILED = 0     ,                      
	DONE     
}IIM42351_Reset_Status;
 /*************************************************
 *  Defining IIM42351 Register & Data As Struct   *
 **************************************************/
typedef	struct iim42351
{
	  uint8_t                       Register_Cache;
		IIM42351_Bank_Sel             Bank_Sel;
	  IIM42351_Reset_Status         RESET;
		uint8_t 											INT_ASYNC_RESET;
	  uint8_t                       WHO_AM_I;
	  IIM42351_Interface            Interface;
		IIM42351_PIN9_FUNCTION        Pin9_Function;
		IIM42351_Ability 							FSYNC;
		IIM42351_Ability 							RTC_Mode ;
		IIM42351_Accel_Fs_Sel				  ACCEL_FS_SEL;
		IIM42351_Accel_Scale_Factor   SCALE_FACTOR;
		IIM42351_Accel_ODR					  ACCEL_ODR;
		IIM42351_UI_Filter_Order 			UI_FILTER_ORDER ;
		IIM42351_Power_Mode 					Power_Mode;
		IIM42351_Low_Noise_Filter_BW  LN_Filter_BW ;
		IIM42351_Low_Power_Filter_AVG LP_Filter_AVG;
		IIM42351_Ability 							Data_Ready_INT;		
		IIM42351_Preparation          DATA_STATUS;
    IIM42351_Data_Endian	        SENSOR_DATA_ENDIAN;
	  IIM42351_Ability              FIFO_STREAM;
		IIM42351_FIFO_MODE					  FIFO_MODE;
	  IIM42351_FIFO_Counting        FIFO_COUNTING;
	  IIM42351_Data_Endian          FIFO_COUNT_ENDIAN;
		IIM42351_Ability 							FIFO_High_Resolution;
		IIM42351_Ability              AllPackets_To_FIFO;
	  IIM42351_Ability              FIFO_WATERMARK;
	  uint16_t               				WATERMARK_Value;
	  uint8_t                       FIFO_DECIMATION_FACTOR;
		IIM42351_Ability              TMST_REGISTER;
		IIM42351_Timestamp_Resolution TMST_Resolution;
  	int16_t RAW_TEMP_DATA;
		int16_t RAW_ACCEL_DATA_X;
		int16_t RAW_ACCEL_DATA_Y;
		int16_t RAW_ACCEL_DATA_Z;
  	float VALID_TEMP_DATA;
		float VALID_ACCEL_DATA_X;
		float VALID_ACCEL_DATA_Y;
		float VALID_ACCEL_DATA_Z;
}GebraBit_IIM42351;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *Declare Read&Write IIM42351 Register Values Functions *
 ********************************************************/
extern	uint8_t	GB_IIM42351_Read_Reg_Data ( uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t* data);
extern	uint8_t GB_IIM42351_Read_Reg_Bits (uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t* data);
extern	uint8_t GB_IIM42351_Burst_Read(uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t *data, uint16_t byteQuantity);
extern	uint8_t GB_IIM42351_Write_Reg_Data(uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t data);
extern	uint8_t	GB_IIM42351_Write_Reg_Bits(uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t data);
extern	uint8_t GB_IIM42351_Burst_Write		( uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t *data, 	uint16_t byteQuantity);
/********************************************************
 *       Declare IIM42351 Configuration Functions       *
 ********************************************************/
extern void GB_IIM42351_Bank_Selection( IIM42351_Bank_Sel bsel);
extern void GB_IIM42351_Who_am_I(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Select_SPI4_Interface( IIM42351_Interface spisel);
extern void GB_IIM42351_Select_PIN9_Function( IIM42351_PIN9_FUNCTION pin9f);
extern void GB_IIM42351_DISABLE_FSYNC ( IIM42351_Ability able ) ;
extern void GB_IIM42351_DISABLE_RTC_Mode ( void ) ;
extern void GB_IIM42351_SET_Time_Stamp_Register(IIM42351_Ability ability);
extern void GB_IIM42351_Set_Timestamp_Resolution (  IIM42351_Timestamp_Resolution res ) ;
extern void GB_IIM42351_SET_INT_ASYNC_RESET_ZERO(void );
/********************************************************
 *          Declare IIM42351 FIFO Functions             *
 ********************************************************/
extern void GB_IIM42351_Set_FIFO_MODE ( IIM42351_FIFO_MODE mode ) ;
extern void GB_IIM42351_SET_FIFO_Count ( IIM42351_FIFO_Counting counting , IIM42351_Data_Endian endian ) ;
extern void GB_IIM42351_SET_AllPackets_To_FIFO( IIM42351_Ability allpack);
extern void GB_IIM42351_SET_FIFO_WATERMARK (IIM42351_Ability watermark , uint16_t wm );
extern void GB_IIM42351_SET_FIFO_Decimation_Factor (uint8_t factor );
extern void GB_IIM42351_FIFO_Configuration ( GebraBit_IIM42351 * iim42351  ) ;
extern void GB_IIM42351_SET_FIFO_High_Resolution( IIM42351_Ability highres);
/********************************************************
 *          Declare IIM42351 ACCEL Functions             *
 ********************************************************/
extern void GB_IIM42351_Set_ACCEL_FS ( GebraBit_IIM42351 * iim42351 , IIM42351_Accel_Fs_Sel fs )  ;
extern void GB_IIM42351_Set_ACCEL_ODR (  IIM42351_Accel_ODR odr ) ;
extern void GB_IIM42351_UI_Filter_Order (  IIM42351_UI_Filter_Order order ) ;
extern void GB_IIM42351_ACCEL_LN_Filter_Configuration( IIM42351_Low_Noise_Filter_BW filter);
extern void GB_IIM42351_ACCEL_LP_Filter_Configuration( IIM42351_Low_Power_Filter_AVG filter);
extern void GB_IIM42351_Set_Power_Management(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_SET_Data_Ready_Interrupt(IIM42351_Ability ability);
extern IIM42351_Preparation GB_IIM42351_Check_Data_Preparation(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_Sensor_Data_Endian ( IIM42351_Data_Endian * data_end  ) ;
/********************************************************
 *          Declare IIM42351 DATA Functions             *
 ********************************************************/
extern void GB_IIM42351_Get_Temp_Register_Raw_Data(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_Temp_Register_Valid_Data(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_ACCEL_DATA_X_Register_Raw(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_ACCEL_DATA_Y_Register_Raw(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_ACCEL_DATA_Z_Register_Raw(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_ACCEL_DATA_X_Register_Valid_Data(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_ACCEL_DATA_Y_Register_Valid_Data(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_ACCEL_DATA_Z_Register_Valid_Data(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_Temperature(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_XYZ_ACCELERATION(GebraBit_IIM42351 * iim42351);
extern void GB_IIM42351_Get_Data(GebraBit_IIM42351 * iim42351);
/********************************************************
 *          Declare IIM42351 HIGH LEVEL Functions       *
 ********************************************************/
extern void GB_IIM42351_Format_Data_Base_On_Endian(GebraBit_IIM42351 * iim42351, const uint8_t *datain, uint16_t *dataout);
extern void GB_IIM42351_Soft_Reset ( GebraBit_IIM42351 * iim42351 );
extern void GB_IIM42351_initialize( GebraBit_IIM42351 * iim42351 );
extern void GB_IIM42351_Configuration(GebraBit_IIM42351 * iim42351);
#endif


