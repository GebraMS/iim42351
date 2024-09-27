/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
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
 
#include	"GebraBit_IIM42351.h"

extern SPI_HandleTypeDef hspi1;
/*=========================================================================================================================================
 * @brief     Read data from spacial register.
 * @param     regAddr Register Address of ICP20100
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that register value is saved .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t	GB_IIM42351_Read_Reg_Data ( uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t* data)
{	
	uint8_t txBuf[2] = {regAddr|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	GB_IIM42351_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		*data = rxBuf[1];
	}
	return stat;
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of ICP20100 .
 * @param     regBank     Register Bank number .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42351_Read_Reg_Bits (uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;

	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}

	if (GB_IIM42351_Read_Reg_Data( regAddr, regBank, &tempData) == HAL_OK)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
		tempData &= mask; // zero all non-important bits in data
		tempData >>= (start_bit - len + 1); //shift data to zero position
		*data = tempData;
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
		*data = 0;
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of ICP20100 that reading multiple data start from this address
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     bytepcs Quantity of data that we want to read .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42351_Burst_Read(uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t *data, uint16_t byteQuantity)
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
	uint8_t status = HAL_ERROR;
	GB_IIM42351_Bank_Selection(regBank);
	pTxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1)); // reason of "+1" is for register address that comes in first byte
	pRxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, (byteQuantity + 1)*sizeof(uint8_t));

	pTxBuf[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	if (status == HAL_OK)
	{
		memcpy(data, &pRxBuf[1], byteQuantity*sizeof(uint8_t)); //here we dont have "+1" beacause we don't need first byte that was register data , we just need DATA itself
	}
	free(pTxBuf);
	free(pRxBuf);
	return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial register.
 * @param     regAddr Register Address of ICP20100
 * @param     regBank Register Bank number .
 * @param     data    Value that will be writen to register .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42351_Write_Reg_Data(uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t data)
{
	uint8_t txBuf[2] = {regAddr|0x00 , data}; //Write operation: set the 8th-bit to 0.
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	GB_IIM42351_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	return status;	
}

/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of ICP20100 .
 * @param     regBank     Register Bank number .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42351_Write_Reg_Bits(uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t txBuf[2];
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}
	if (GB_IIM42351_Read_Reg_Data( regAddr, regBank, &tempData) == HAL_OK)	
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= data; // combine data with existing byte

		txBuf[0] = regAddr;
		txBuf[1] = tempData;
	
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Write value to Multiple register address.
 * @param     regAddr First Register Address of ICP20100 that writing multiple data start from this address
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that multiple data are writen from .
 * @param     bytepcs Quantity of data that we want to write .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42351_Burst_Write		( uint8_t regAddr, IIM42351_Bank_Sel regBank, uint8_t *data, 	uint16_t byteQuantity)
{
	uint8_t txBuf[byteQuantity + 1]; // +1 is for register address that is 1 byte
	uint8_t rxBuf[byteQuantity + 1];
	uint8_t status = HAL_ERROR;
	GB_IIM42351_Bank_Selection(regBank);
	txBuf[0] = regAddr | 0x00; //Write operation: set the 8th-bit to 0.
	memcpy(txBuf+1, data, byteQuantity); // +1 is for set the address of data from [1]th position of array

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	return status;
}
/*=========================================================================================================================================
 * @brief     Select Register Bank.
 * @param     bsel   Bank number
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42351_Bank_Selection( IIM42351_Bank_Sel bsel)
{
  uint8_t rtxBuf[2] = {IIM42351_REG_BANK_SEL|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rrxBuf[2];
	uint8_t wtxBuf[2];
	uint8_t wrxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	uint8_t tempData = 0;
	uint8_t start_bit = START_MSB_BIT_AT_2 ;
	uint8_t len = BIT_LENGTH_3 ;
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, rtxBuf, rrxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		tempData = rrxBuf[1];
	}
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		bsel <<= (start_bit - len + 1); // shift data into correct position
		bsel &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= bsel; // combine data with existing byte

		wtxBuf[0] = IIM42351_REG_BANK_SEL;
		wtxBuf[1] = tempData;
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		stat = (HAL_SPI_TransmitReceive(&hspi1, wtxBuf, wrxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}
/*=========================================================================================================================================
 * @brief     Select SPI 4 Wire as interface
 * @param     spisel Determines SPI 4 Wire as interface or not 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42351_Select_SPI4_Interface( IIM42351_Interface spisel)
{
 GB_IIM42351_Write_Reg_Bits( IIM42351_INTF_CONFIG4,BANK_1, START_MSB_BIT_AT_1, BIT_LENGTH_1 , spisel);
 GB_IIM42351_Write_Reg_Bits( IIM42351_INTF_CONFIG6,BANK_1, START_MSB_BIT_AT_4, BIT_LENGTH_5 , I3C_DISABLE);
}
/*=========================================================================================================================================
 * @brief     Select Pin 9 Function
 * @param     pin9f Determines Pin 9 Function INT2 , FSYNC , CLKIN
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42351_Select_PIN9_Function( IIM42351_PIN9_FUNCTION pin9f)
{
 GB_IIM42351_Write_Reg_Bits( IIM42351_INTF_CONFIG5,BANK_1, START_MSB_BIT_AT_2, BIT_LENGTH_2 , pin9f);
}
/*=========================================================================================================================================
 * @brief     Read sensor Data endianess
 * @param     data_end Determines Data endianess BIG or LITTLE
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42351_Get_Sensor_Data_Endian ( IIM42351_Data_Endian * data_end  ) 
{
	GB_IIM42351_Read_Reg_Bits (IIM42351_INTF_CONFIG0, BANK_0 , START_MSB_BIT_AT_4, BIT_LENGTH_1, data_end);
}
/*=========================================================================================================================================
 * @brief     Reset IIM42351
 * @param     iim42351   GebraBit_IIM42351 Struct
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42351_Soft_Reset ( GebraBit_IIM42351 * iim42351 )
{
	uint8_t rest_done=0;
	do 
	 {
		GB_IIM42351_Write_Reg_Data( IIM42351_DEVICE_CONFIG ,  BANK_0, IIM42351_RESET); 
		HAL_Delay(1);
		GB_IIM42351_Read_Reg_Bits (IIM42351_INT_STATUS, BANK_0 , START_MSB_BIT_AT_4, BIT_LENGTH_1, &iim42351->RESET);
		if ( iim42351->RESET == DONE )
			break;
	 }while(1);
	GB_IIM42351_Select_SPI4_Interface( iim42351->Interface);
	GB_IIM42351_Select_PIN9_Function (iim42351->Pin9_Function);
	GB_IIM42351_Get_Sensor_Data_Endian ( &iim42351->SENSOR_DATA_ENDIAN  ) ;
}
/*=========================================================================================================================================
 * @brief     Set ACCEL Full Scale Range and select sensor SCALE FACTOR
 * @param     iim42351   GebraBit_IIM42351 Struct
 * @param     fs         Determines Full Scale Range among 2g , 4g , 8g , 16g
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42351_Set_ACCEL_FS ( GebraBit_IIM42351 * iim42351 , IIM42351_Accel_Fs_Sel fs ) 
{
  GB_IIM42351_Write_Reg_Bits( IIM42351_ACCEL_CONFIG0,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_3 , fs);
	switch(fs)
	 {
	  case FS_16g:
		iim42351->SCALE_FACTOR = SCALE_FACTOR_2048_LSB_g ;
    break;
		case FS_8g:
		iim42351->SCALE_FACTOR = SCALE_FACTOR_4096_LSB_g ;
    break;	
		case FS_4g:
		iim42351->SCALE_FACTOR = SCALE_FACTOR_8192_LSB_g ;
    break;	
		case FS_2g:
		iim42351->SCALE_FACTOR = SCALE_FACTOR_16384_LSB_g ;
    break;			
		default:
		iim42351->SCALE_FACTOR = SCALE_FACTOR_2048_LSB_g ;		
	 }
}
/*=========================================================================================================================================
 * @brief     Set ACCEL Output Data Rate
 * @param     odr       Determines Output Data Rate from 1.565 Hz to 8KHz 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Set_ACCEL_ODR (  IIM42351_Accel_ODR odr ) 
{
  GB_IIM42351_Write_Reg_Bits(IIM42351_ACCEL_CONFIG0,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_4 , odr);
}
/*=========================================================================================================================================
 * @brief     Set FIFO MODE
 * @param     mode     Determines FIFO MODE BYPASS ,  STREAM_TO_FIFO , STOP_ON_FULL
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Set_FIFO_MODE ( IIM42351_FIFO_MODE mode ) 
{
  GB_IIM42351_Write_Reg_Bits( IIM42351_FIFO_CONFIG,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_2 , mode);
}
/*=========================================================================================================================================
 * @brief     DISABLE FSYNC Function
 * @param     able     Determines FSYNC Function Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_DISABLE_FSYNC ( IIM42351_Ability able) 
{
  GB_IIM42351_Write_Reg_Bits(IIM42351_FSYNC_CONFIG,BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_3 ,DO_NOT_TAG_FSYNC_FLAG);
	GB_IIM42351_Write_Reg_Bits(IIM42351_TMST_CONFIG,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_1 , FSYNC_TIME_STAMP_DISABLE);
}
/*=========================================================================================================================================
 * @brief     Set Timestamp Resolution
 * @param     res           Determines Timestamp Resolution _16_uS ,  _1_uS
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Set_Timestamp_Resolution (  IIM42351_Timestamp_Resolution res) 
{
  GB_IIM42351_Write_Reg_Bits(IIM42351_TMST_CONFIG,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_1 , res);
}
/*=========================================================================================================================================
 * @brief     Set FIFO Count Setting 
 * @param     counting           Determines FIFO count is reported in bytes or record
 * @param     endian             Determines FIFO count is reported in Little Endian format or Big
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_SET_FIFO_Count ( IIM42351_FIFO_Counting counting , IIM42351_Data_Endian endian ) 
{
  GB_IIM42351_Write_Reg_Bits(IIM42351_INTF_CONFIG0,BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_1 , counting);
	GB_IIM42351_Write_Reg_Bits(IIM42351_INTF_CONFIG0,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , endian);
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Time Stamp Register
 * @param     ability     Determines Time Stamp Register  Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_SET_Time_Stamp_Register(IIM42351_Ability ability)
{
  GB_IIM42351_Write_Reg_Bits(IIM42351_TMST_CONFIG,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1 , ability);
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE All Packets To STORE in FIFO
 * @param     ability     Determines All Packets  STORE in FIFO or not
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_IIM42351_SET_AllPackets_To_FIFO(IIM42351_Ability allpack)
{
  if (allpack == Enable)
	 GB_IIM42351_Write_Reg_Bits( IIM42351_FIFO_CONFIG1,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_4 , ENABLE_ALL_PACKETS_TO_FIFO);
	else
	 GB_IIM42351_Write_Reg_Bits( IIM42351_FIFO_CONFIG1,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_4 , DISABLE_ALL_PACKETS_TO_FIFO);
}
/*
M403Z 
*/
/*=========================================================================================================================================
 * @brief     SET FIFO WATERMARK 
 * @param     watermark     Determines FIFO WATERMARK Enable or not
 * @param     wm            Determines FIFO WATERMARK Value
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_IIM42351_SET_FIFO_WATERMARK (IIM42351_Ability watermark , uint16_t wm)
{
	if( (watermark == Enable)&&(wm <= 2047) )
	{		
    GB_IIM42351_Write_Reg_Bits (IIM42351_FIFO_CONFIG1,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , Enable);
		GB_IIM42351_Write_Reg_Data (IIM42351_FIFO_CONFIG2,BANK_0,(uint8_t) (wm & 0xff));
	  GB_IIM42351_Write_Reg_Bits (IIM42351_FIFO_CONFIG3,BANK_0,START_MSB_BIT_AT_3, BIT_LENGTH_4 ,(uint8_t) (wm>> 8));	
	}	
}
/*=========================================================================================================================================
 * @brief     SET FIFO Decimation Factor
 * @param     factor   FIFO Decimation Factor Value
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_SET_FIFO_Decimation_Factor (uint8_t factor )
{
	if( factor <= 127 )
	{		
    GB_IIM42351_Write_Reg_Bits (IIM42351_FDR_CONFIG ,BANK_4, START_MSB_BIT_AT_6, BIT_LENGTH_7 , factor);
	}	
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Data Ready Interrupt
 * @param     ability    Determines Data Ready Interrupt Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_SET_Data_Ready_Interrupt(IIM42351_Ability ability)
{
	GB_IIM42351_Write_Reg_Bits(IIM42351_INT_SOURCE0,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_1 , ability);
}
/*=========================================================================================================================================
 * @brief     For register INT_CONFIG1 (bank 0 register 0x64) bit 4 INT_ASYNC_RESET, user should change setting to 0 from default setting 
              of 1, for proper INT1 and INT2 pin operation
 * @param     Nothing
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_SET_INT_ASYNC_RESET_ZERO(void )
{
	GB_IIM42351_Write_Reg_Bits( IIM42351_INT_CONFIG1,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , 0);
}
/*=========================================================================================================================================
 * @brief     Configure FIFO
 * @param     iim42351       Configure IIM42351 FIFO according to IIM42351 Struct FIFO_STREAM variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_FIFO_Configuration ( GebraBit_IIM42351 * iim42351  )
{
	if( iim42351->FIFO_STREAM==Enable )
	{
    GB_IIM42351_SET_FIFO_Count(IN_RECORDS , iim42351->FIFO_COUNT_ENDIAN );//LITTLE
	  GB_IIM42351_Set_FIFO_MODE ( iim42351->FIFO_MODE );//STOP_ON_FULL
	  GB_IIM42351_SET_Time_Stamp_Register(iim42351->TMST_REGISTER);//Enable
	  GB_IIM42351_SET_AllPackets_To_FIFO(iim42351->AllPackets_To_FIFO);//Enable
	  GB_IIM42351_SET_FIFO_WATERMARK(iim42351->FIFO_WATERMARK , iim42351->WATERMARK_Value);//Enable , 1
	  GB_IIM42351_SET_Data_Ready_Interrupt(iim42351->Data_Ready_INT);//Disable
	  GB_IIM42351_SET_FIFO_Decimation_Factor(iim42351->FIFO_DECIMATION_FACTOR);//FIFO_DECIMATION_FACTOR=0
	}
	else
	{
		GB_IIM42351_Set_FIFO_MODE  (iim42351->FIFO_MODE );//BYPASS
		GB_IIM42351_SET_AllPackets_To_FIFO(iim42351->AllPackets_To_FIFO);//--****************************---Disable
		GB_IIM42351_SET_Data_Ready_Interrupt(iim42351->Data_Ready_INT);//Enable
	}
}
/*=========================================================================================================================================
 * @brief     Set UI Filter Order
 * @param     order  Determines UI Filter Order _1_ORDER ,  _2_ORDER or _3_ORDER
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_UI_Filter_Order (  IIM42351_UI_Filter_Order order ) 
{
  GB_IIM42351_Write_Reg_Bits(IIM42351_ACCEL_CONFIG1,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_2 , order);
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE FIFO High Resolution
 * @param     highres    Determines FIFO High Resolution Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_SET_FIFO_High_Resolution( IIM42351_Ability highres)
{
  if (highres == Enable)
	 GB_IIM42351_Write_Reg_Bits( IIM42351_FIFO_CONFIG1,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , Enable);
	else 
	 GB_IIM42351_Write_Reg_Bits( IIM42351_FIFO_CONFIG1,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , Disable);
}
/*=========================================================================================================================================
 * @brief     Get Who am I Register Value From Sensor
 * @param     iim42351     IIM42351 Struct WHO_AM_I variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_IIM42351_Who_am_I(GebraBit_IIM42351 * iim42351)
{
	GB_IIM42351_Read_Reg_Data( IIM42351_WHO_AM_I, BANK_0,&iim42351->WHO_AM_I);
}	
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE RTC Mode
 * @param     Nothing
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_DISABLE_RTC_Mode ( void ) 
{
  GB_IIM42351_Write_Reg_Bits(IIM42351_INTF_CONFIG1,BANK_0, START_MSB_BIT_AT_2, BIT_LENGTH_1 , Disable);
}
/*=========================================================================================================================================
 * @brief     Set ACCEL LN Filter
 * @param     filter       Determines LN FILTER BW from 2 to 40 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_ACCEL_LN_Filter_Configuration( IIM42351_Low_Noise_Filter_BW filter)
{
	if(filter==IIM42351_LOW_NOISE)
   GB_IIM42351_Write_Reg_Bits( IIM42351_ACCEL_FILT_CONFIG,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_4 , filter);
}
/*=========================================================================================================================================
 * @brief     Set ACCEL LP Filter
 * @param     filter       Determines LP FILTER BW between LP_1x_AVG_FILTER or LP_16x_AVG_FILTER
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_ACCEL_LP_Filter_Configuration( IIM42351_Low_Power_Filter_AVG filter)
{
	if(filter==IIM42351_LOW_POWER)
   GB_IIM42351_Write_Reg_Bits( IIM42351_ACCEL_FILT_CONFIG,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_4 , filter);
}
/*=========================================================================================================================================
 * @brief     Set Power Mode
 * @param     iim42351       Configure IIM42351 Power Mode according to IIM42351 Struct Power_Mode variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Set_Power_Management(GebraBit_IIM42351 * iim42351)
{
 if(iim42351->Power_Mode==IIM42351_LOW_POWER)
 {
	GB_IIM42351_ACCEL_LP_Filter_Configuration(  iim42351->LP_Filter_AVG);
  GB_IIM42351_Write_Reg_Bits( IIM42351_PWR_MGMT0,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_2 , iim42351->Power_Mode); 
 }
  else if(iim42351->Power_Mode==IIM42351_LOW_NOISE)
 {
	GB_IIM42351_ACCEL_LN_Filter_Configuration(  iim42351->LN_Filter_BW);
  GB_IIM42351_Write_Reg_Bits(IIM42351_PWR_MGMT0,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_2 , iim42351->Power_Mode);  
 }
 else
 {
	GB_IIM42351_Write_Reg_Bits( IIM42351_PWR_MGMT0,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_2 , iim42351->Power_Mode); 
 }
 HAL_Delay(1);
}
/*=========================================================================================================================================
 * @brief     Check if Data is ready
 * @param     iim42351    Store data ready status on IIM42351 Struct DATA_STATUS variable
 * @return    IS_Ready or IS_NOT_Ready
 ========================================================================================================================================*/ 
IIM42351_Preparation GB_IIM42351_Check_Data_Preparation(GebraBit_IIM42351 * iim42351)
{
  GB_IIM42351_Read_Reg_Bits (IIM42351_INT_STATUS, BANK_0 , START_MSB_BIT_AT_3, BIT_LENGTH_1, &iim42351->DATA_STATUS); 
	return iim42351->DATA_STATUS;
}
/*=========================================================================================================================================
 * @brief     Format Data
 * @param     iim42351       Format Data Base On  IIM42351 Struct SENSOR_DATA_ENDIAN variable
 * @param     datain         raw input Data
 * @param     dataout        Formated output Data 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Format_Data_Base_On_Endian(GebraBit_IIM42351 * iim42351, const uint8_t *datain, uint16_t *dataout)
{
	if(iim42351->SENSOR_DATA_ENDIAN == BIG)
		*dataout = (datain[0] << 8) | datain[1];
	if(iim42351->SENSOR_DATA_ENDIAN == LITTLE)
		*dataout = (datain[1] << 8) | datain[0];
}
/*=========================================================================================================================================
 * @brief     initialize IIM42351
 * @param     iim42351     initialize IIM42351 according  GebraBit_IIM42351 Staruct values
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_initialize( GebraBit_IIM42351 * iim42351 )
{
  HAL_Delay(3);
	GB_IIM42351_Soft_Reset(iim42351);
	//GB_IIM42351_Set_ACCEL_FS(iim42351 , FS_4g );
	GB_IIM42351_Set_FIFO_MODE ( iim42351->FIFO_MODE  ) ;//BYPASS
	GB_IIM42351_DISABLE_FSYNC ( iim42351->FSYNC ) ;//Disable
	//GB_IIM42351_Set_Timestamp_Resolution ( iim42351->TMST_Resolution  ) ;
	//GB_IIM42351_FIFO_Configuration ( iim42351 ,Disable ) ;
	GB_IIM42351_SET_INT_ASYNC_RESET_ZERO(  );
	GB_IIM42351_UI_Filter_Order (  iim42351->UI_FILTER_ORDER ) ;
	//GB_IIM42351_SET_FIFO_High_Resolution( iim42351->FIFO_High_Resolution);
	GB_IIM42351_FIFO_Configuration ( iim42351 ) ;
	GB_IIM42351_Who_am_I(iim42351);
	//GB_IIM42351_ACCEL_LN_Filter_Configuration(iim42351 , LN_FILTER_BW_4);
	//iim42351->LN_Filter_BW  = LN_FILTER_BW_4  ;
	//iim42351->LP_Filter_AVG = LP_16x_AVG_FILTER ;	
}
/*=========================================================================================================================================
 * @brief     Configure IIM42351
 * @param     iim42351  Configure IIM42351 according  GebraBit_IIM42351 Staruct values
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Configuration(GebraBit_IIM42351 * iim42351)
{
  GB_IIM42351_DISABLE_RTC_Mode (  ) ;
	GB_IIM42351_Set_ACCEL_FS( iim42351 , iim42351->ACCEL_FS_SEL );
	GB_IIM42351_Set_ACCEL_ODR ( iim42351->ACCEL_ODR ); 
	GB_IIM42351_Set_Power_Management( iim42351 );
	HAL_Delay(20);	
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temprature from Register 
 * @param     iim42351  store Raw Data Of Temprature in GebraBit_IIM42351 Staruct RAW_TEMP_DATA
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_Temp_Register_Raw_Data(GebraBit_IIM42351 * iim42351)
{
	uint8_t temp_msb , temp_lsb;
  GB_IIM42351_Read_Reg_Data( IIM42351_TEMP_DATA1, BANK_0, &temp_msb);
	GB_IIM42351_Read_Reg_Data(IIM42351_TEMP_DATA0, BANK_0, &temp_lsb);
	iim42351->RAW_TEMP_DATA = (int16_t)((temp_msb << 8) | temp_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Temprature Base on Datasheet Formula 
 * @param     iim42351  store Valid Data Of Temprature in GebraBit_IIM42351 Staruct VALID_TEMP_DATA
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_Temp_Register_Valid_Data(GebraBit_IIM42351 * iim42351)
{
  iim42351->VALID_TEMP_DATA =(iim42351->RAW_TEMP_DATA / 132.48) + 25;
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis ACCEL from Register 
 * @param     iim42351  store Raw Data Of X Axis ACCEL DATA in GebraBit_IIM42351 Staruct RAW_ACCEL_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_ACCEL_DATA_X_Register_Raw(GebraBit_IIM42351 * iim42351)
{
	uint8_t accelx_msb , acclx_lsb;
  GB_IIM42351_Read_Reg_Data( IIM42351_ACCEL_DATA_X1, BANK_0, &accelx_msb);
	GB_IIM42351_Read_Reg_Data( IIM42351_ACCEL_DATA_X0, BANK_0, &acclx_lsb );
	iim42351->RAW_ACCEL_DATA_X = (int16_t)((accelx_msb << 8) | acclx_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis ACCEL from Register 
 * @param     iim42351  store Raw Data Of Y Axis ACCEL DATA in GebraBit_IIM42351 Staruct RAW_ACCEL_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_ACCEL_DATA_Y_Register_Raw(GebraBit_IIM42351 * iim42351)
{
	uint8_t accely_msb , accly_lsb;
  GB_IIM42351_Read_Reg_Data( IIM42351_ACCEL_DATA_Y1, BANK_0, &accely_msb);
	GB_IIM42351_Read_Reg_Data( IIM42351_ACCEL_DATA_Y0, BANK_0, &accly_lsb );
	iim42351->RAW_ACCEL_DATA_Y = (int16_t)((accely_msb << 8) | accly_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis ACCEL from Register 
 * @param     iim42351  store Raw Data Of Z Axis ACCEL DATA in GebraBit_IIM42351 Staruct RAW_ACCEL_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_ACCEL_DATA_Z_Register_Raw(GebraBit_IIM42351 * iim42351)
{
	uint8_t accelz_msb , acclz_lsb;
  GB_IIM42351_Read_Reg_Data( IIM42351_ACCEL_DATA_Z1, BANK_0, &accelz_msb);
	GB_IIM42351_Read_Reg_Data( IIM42351_ACCEL_DATA_Z0, BANK_0, &acclz_lsb );
	iim42351->RAW_ACCEL_DATA_Z = (int16_t)((accelz_msb << 8) | acclz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis ACCEL Base on GebraBit_IIM42351 Staruct SCALE_FACTOR 
 * @param     iim42351  store Valid Data Of X Axis ACCEL in GebraBit_IIM42351 Staruct VALID_ACCEL_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_ACCEL_DATA_X_Register_Valid_Data(GebraBit_IIM42351 * iim42351)
{
	float scale_factor = iim42351->SCALE_FACTOR;
  iim42351->VALID_ACCEL_DATA_X =(iim42351->RAW_ACCEL_DATA_X /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis ACCEL Base on GebraBit_IIM42351 Staruct SCALE_FACTOR 
 * @param     iim42351  store Valid Data Of Y Axis ACCEL in GebraBit_IIM42351 Staruct VALID_ACCEL_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_ACCEL_DATA_Y_Register_Valid_Data(GebraBit_IIM42351 * iim42351)
{
	float scale_factor = iim42351->SCALE_FACTOR;
  iim42351->VALID_ACCEL_DATA_Y =(iim42351->RAW_ACCEL_DATA_Y /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis ACCEL Base on GebraBit_IIM42351 Staruct SCALE_FACTOR 
 * @param     iim42351  store Valid Data Of Z Axis ACCEL in GebraBit_IIM42351 Staruct VALID_ACCEL_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_ACCEL_DATA_Z_Register_Valid_Data(GebraBit_IIM42351 * iim42351)
{
	float scale_factor = iim42351->SCALE_FACTOR;
  iim42351->VALID_ACCEL_DATA_Z =(iim42351->RAW_ACCEL_DATA_Z /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Temprature Directly 
 * @param     iim42351       GebraBit_IIM42351 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_Temperature(GebraBit_IIM42351 * iim42351)
{
  GB_IIM42351_Get_Temp_Register_Raw_Data  (iim42351);
	GB_IIM42351_Get_Temp_Register_Valid_Data(iim42351);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION Directly 
 * @param     iim42351       GebraBit_IIM42351 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_XYZ_ACCELERATION(GebraBit_IIM42351 * iim42351)
{
	GB_IIM42351_Get_ACCEL_DATA_X_Register_Raw(iim42351);
	GB_IIM42351_Get_ACCEL_DATA_X_Register_Valid_Data(iim42351);
	GB_IIM42351_Get_ACCEL_DATA_Y_Register_Raw(iim42351);
	GB_IIM42351_Get_ACCEL_DATA_Y_Register_Valid_Data(iim42351);
	GB_IIM42351_Get_ACCEL_DATA_Z_Register_Raw(iim42351);
	GB_IIM42351_Get_ACCEL_DATA_Z_Register_Valid_Data(iim42351);
}
/*=========================================================================================================================================
 * @brief     Get Data Directly 
 * @param     iim42351       GebraBit_IIM42351 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42351_Get_Data(GebraBit_IIM42351 * iim42351)
{
     if (IS_Ready == GB_IIM42351_Check_Data_Preparation(iim42351))
		 {
      GB_IIM42351_Get_Temperature( iim42351 )  ;
			GB_IIM42351_Get_XYZ_ACCELERATION (iim42351)  ;
		 }
}