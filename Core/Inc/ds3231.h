/*
 * ds3231.h
 *
 *  Created on: Jan 12, 2025
 *      Author: admin
 */

#ifndef INC_DS3231_H_
#define INC_DS3231_H_

#include "stm32f1xx_hal.h" /* Needed for I2c*/

/*
 * DEFFINES
 */
#define DS3231_I2C_ADDR  (0x68)/*The slave address byte contains
                                     the 7-bit DS3231 address,
                                     which is 1101000*/

#define  DS3231_REG_SECONDS           0x00
#define  DS3231_REG_MINUTES           0x01
#define  DS3231_REG_HOURS             0x02
#define  DS3231_REG_DAY               0x03
#define  DS3231_REG_DATE              0x04
#define  DS3231_REG_MONTH_CENTERY     0x05
     #define DS3231_BIT_CENTURY 		7
#define  DS3231_REG_YEAR              0x06
#define  DS3231_REG_ALARM1_SEC        0x07
#define  DS3231_REG_ALARM1_MIN        0x08
#define  DS3231_REG_ALARM1_HOUR       0x09
#define  DS3231_REG_ALARM1_DAY        0x0a
#define  DS3231_REG_ALARM2_MIN        0x0b
#define  DS3231_REG_ALARM2_HOUR       0x0c
#define  DS3231_REG_ALARM2_DAY        0x0d
     #define DS3231_BIT_AXMY			7
     #define DS3231_BIT_DYDT			6
#define  DS3231_REG_CONTROL           0x0e
	#define DS3231_BIT_EOSC			7
	#define DS3231_BIT_BBSQW		6
	#define DS3231_BIT_CONV			5
	#define DS3231_BIT_RS2			4
	#define DS3231_BIT_RS1			3
	#define DS3231_BIT_INTCN		2
	#define DS3231_BIT_A2IE			1
	#define DS3231_BIT_A1IE			0
#define  DS3231_REG_CNTR_STATUS       0x0f
	#define DS3231_BIT_OSF			7
	#define DS3231_BIT_EN32KHZ		3
	#define DS3231_BIT_BSY			2
	#define DS3231_BIT_A2F			1
	#define DS3231_BIT_A1F			0
#define  DS3231_REG_AGING_OFFSET      0x10
#define  DS3231_REG_MSB_OF_TEMP       0x11
#define  DS3231_REG_LSB_OF_TEMP       0x12

#define DS3231_TIMEOUT		HAL_MAX_DELAY

/*--------------------------------------------*/
typedef enum DS3231_Rate{
	DS3231_1HZ, DS3231_1024HZ, DS3231_4096HZ, DS3231_8192HZ
}DS3231_Rate;

typedef enum DS3231_InterruptMode{
	DS3231_SQUARE_WAVE_INTERRUPT, DS3231_ALARM_INTERRUPT
}DS3231_InterruptMode;

typedef enum DS3231_State{
	DS3231_DISABLED, DS3231_ENABLED
}DS3231_State;

typedef enum D3231_Alarm1Mode{
	DS3231_A1_EVERY_S = 0x0f, DS3231_A1_MATCH_S = 0x0e, DS3231_A1_MATCH_S_M = 0x0c, DS3231_A1_MATCH_S_M_H = 0x08, DS3231_A1_MATCH_S_M_H_DATE = 0x00, DS3231_A1_MATCH_S_M_H_DAY = 0x80,
}DS3231_Alarm1Mode;


typedef enum D3231_Alarm2Mode{
	DS3231_A2_EVERY_M = 0x07, DS3231_A2_MATCH_M = 0x06, DS3231_A2_MATCH_M_H = 0x04, DS3231_A2_MATCH_M_H_DATE = 0x00, DS3231_A2_MATCH_M_H_DAY = 0x80,
}DS3231_Alarm2Mode;

typedef struct{
	I2C_HandleTypeDef *i2cHandle;
	float timeSec;
	float timeMin;
	float timeHours;
	float timeDays;
	float timeYears;
} DS3231;

extern I2C_HandleTypeDef *_ds3231_ui2c;


void DS3231_Init(I2C_HandleTypeDef *hi2c);


/*
 * INITIALISATION
 */
uint8_t DS3231_Initialase(DS3231 *dev, I2C_HandleTypeDef *i2cHandle);

/*
 * DATA ACQUISTION
 */
void DS3231_SetRegByte(uint8_t regAddr, uint8_t val);
uint8_t DS3231_GetRegByte(uint8_t regAddr);

uint8_t DS3231_GetDayOfWeek(void);
uint8_t DS3231_GetDate(void);
uint8_t DS3231_GetMonth(void);
uint16_t DS3231_GetYear(void);

uint8_t DS3231_GetHour(void);
uint8_t DS3231_GetMinute(void);
uint8_t DS3231_GetSecond(void);

void DS3231_SetDayOfWeek(uint8_t dow);
void DS3231_SetDate(uint8_t date);
void DS3231_SetMonth(uint8_t month);
void DS3231_SetYear(uint16_t year);

void DS3231_SetHour(uint8_t hour_24mode);
void DS3231_SetMinute(uint8_t minute);
void DS3231_SetSecond(uint8_t second);

void DS3231_SetFullTime(uint8_t hour_24mode, uint8_t minute, uint8_t second);
void DS3231_SetFullDate(uint8_t date, uint8_t month, uint8_t dow, uint16_t year);

uint8_t DS3231_DecodeBCD(uint8_t bin);
uint8_t DS3231_EncodeBCD(uint8_t dec);

void DS3231_EnableBatterySquareWave(DS3231_State enable);
void DS3231_SetInterruptMode(DS3231_InterruptMode mode);
void DS3231_SetRateSelect(DS3231_Rate rate);
void DS3231_EnableOscillator(DS3231_State enable);

void DS3231_EnableAlarm2(DS3231_State enable);
void DS3231_SetAlarm2Mode(DS3231_Alarm2Mode alarmMode);
void DS3231_ClearAlarm2Flag();
void DS3231_SetAlarm2Minute(uint8_t minute);
void DS3231_SetAlarm2Hour(uint8_t hour_24mode);
void DS3231_SetAlarm2Date(uint8_t date);
void DS3231_SetAlarm2Day(uint8_t day);

void DS3231_EnableAlarm1(DS3231_State enable);
void DS3231_SetAlarm1Mode(DS3231_Alarm1Mode alarmMode);
void DS3231_ClearAlarm1Flag();
void DS3231_SetAlarm1Second(uint8_t second);
void DS3231_SetAlarm1Minute(uint8_t minute);
void DS3231_SetAlarm1Hour(uint8_t hour_24mode);
void DS3231_SetAlarm1Date(uint8_t date);
void DS3231_SetAlarm1Day(uint8_t day);

void DS3231_Enable32kHzOutput(DS3231_State enable);

uint8_t DS3231_IsOscillatorStopped();
uint8_t DS3231_Is32kHzEnabled();
uint8_t DS3231_IsAlarm1Triggered();
uint8_t DS3231_IsAlarm2Triggered();

int8_t DS3231_GetTemperatureInteger();
uint8_t DS3231_GetTemperatureFraction();






HAL_StatusTypeDef  DS3231_ReadTime(DS3231 *dev);

/*
 * LOW_LEVEL FUNCTIONS
 */

HAL_StatusTypeDef  DS3231_ReadRegister(DS3231 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef  DS3231_ReadRegisters(DS3231 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef  DS3231_WridedRegister(DS3231 *dev, uint8_t reg, uint8_t *data);

#endif /* INC_DS3231_H_ */
