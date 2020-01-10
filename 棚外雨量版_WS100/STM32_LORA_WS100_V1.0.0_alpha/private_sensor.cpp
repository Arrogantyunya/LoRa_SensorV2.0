#include "private_sensor.h"
#include <Arduino.h>
#include "RS485.h"
#include "MODBUS_RTU_CRC16.h" 
#include "MAX44009/i2c_MAX44009.h"
#include "SHT1x.h"
#include <libmaple/iwdg.h>

struct SENSOR_DATA Sensor_Data;
Sensor private_sensor;

bool g_Current_Rain_Status_Flag = false;
bool g_Last_Rain_Status_Flag = false;

bool g_Send_Air_SensorData_Flag = true;

//获取所有传感器数据
void Sensor::Get_All_Sensor_Data(void)
{
	iwdg_feed();

	Sensor_Data.g_Temp = sht10.readTemperatureC();//温度
	delay(100);
	Sensor_Data.g_Humi = sht10.readHumidity();//相对湿度
	delay(100);
	Read_Rainfall(&Sensor_Data.g_Is_Rain_or_Snow, RAINFALL_SENSOR_ADDR);//得到下雨
	delay(100);
	private_sensor.Read_Lux_and_UV(&Sensor_Data.g_Lux, &Sensor_Data.g_UV);//得到光照强度和紫外强度
	delay(100);

	Serial.print("temperature: ");
	Serial.println(Sensor_Data.g_Temp);
	Serial.print("humility: ");
	Serial.println(Sensor_Data.g_Humi);
	Sensor_Data.g_Is_Rain_or_Snow == 0 ? Serial.println("Sunny day") : (Sensor_Data.g_Is_Rain_or_Snow == 1 ? Serial.println("rainly day") : Serial.println("Rain Sensor abnormal!"));
}

/*
 *brief   : Initialize UV and Illumination sensors.
 *para    : None
 *return  : None
 */
void Sensor::CJMCU6750_Init(void)
{
	iwdg_feed();
	CJMCU6750.begin(PB15, PB14);

	CJMCU6750.beginTransmission(I2C_ADDR);
	CJMCU6750.write((IT_1 << 2) | 0x02);
	CJMCU6750.endTransmission();
	delay(500);

	if (max44009.initialize())  //Lux光照强度
		Serial.println("Sensor MAX44009 found...");
	else
		Serial.println("Light Sensor missing !!!");
}

/*
 *brief   : Read Lux and UV from sensor (I2C)
 *para    : Lux, UV
 *return  : None
 */
void Sensor::Read_Lux_and_UV(unsigned long *lux, unsigned int *uv)
{
	unsigned char msb = 0, lsb = 0;
	unsigned long mLux_value = 0;

	iwdg_feed();

	CJMCU6750_Init();

#if LUX_UV
	CJMCU6750.requestFrom(I2C_ADDR + 1, 1); //MSB
	delay(1);

	if (CJMCU6750.available())
		msb = CJMCU6750.read();

	CJMCU6750.requestFrom(I2C_ADDR + 0, 1); //LSB
	delay(1);

	if (CJMCU6750.available())
		lsb = CJMCU6750.read();

	*uv = (msb << 8) | lsb;
	*uv *= 0.005625;
	Serial.print("UV : "); //output in steps (16bit)
	Serial.println(*uv);
#endif

	max44009.getMeasurement(mLux_value);

	*lux = mLux_value / 1000L;
	Serial.print("light intensity value:");
	Serial.print(*lux);
	Serial.println(" Lux");
}

/*
 *brief   : ModBus协议采集是否降雨或降雪
 *para    : 无符号字符型的开关量参数、ModBus雨雪传感器地址
 *return  : 无
 */
void Sensor::Read_Rainfall(unsigned char *rainfall, unsigned char address)
{
	unsigned char Rainfall_Cmd[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00 };
	unsigned char Receive_Data[7] = { 0 };
	unsigned int CRC16 = 0, CRC16_temp = 0;
	unsigned char Rainfall_temp = 255;
	unsigned char i = 0, j = 0;

	iwdg_feed();

	Rainfall_Cmd[0] = address;

	CRC16 = N_CRC16(Rainfall_Cmd, 6);
	Rainfall_Cmd[6] = CRC16 >> 8;
	Rainfall_Cmd[7] = CRC16 & 0xFF;

	RS485_Serial.write(Rainfall_Cmd, 8);
	delay(100);

	while (RS485_Serial.available() > 0)
	{
		if (i >= 7)
		{
			i = 0;
			j++;
			if (j >= 10) break;
		}
		Receive_Data[i++] = RS485_Serial.read();
	}
	if (i > 0) 
	{
		i = 0;
		CRC16 = N_CRC16(Receive_Data, 5);
		CRC16_temp = (Receive_Data[5] << 8) | Receive_Data[6];

		if (CRC16_temp == CRC16)
			Rainfall_temp = Receive_Data[4];
	}
	*rainfall = Rainfall_temp;
}

bool Sensor::Detect_Rain(void)
{
	Sensor_Data.g_Is_Rain_or_Snow = 0;
	iwdg_feed();
	Read_Rainfall(&Sensor_Data.g_Is_Rain_or_Snow, RAINFALL_SENSOR_ADDR);
	delay(200);
	if (Sensor_Data.g_Is_Rain_or_Snow == 1) {
		g_Current_Rain_Status_Flag = true;
		return true;
	}
	else if (Sensor_Data.g_Is_Rain_or_Snow == 0) {
		g_Current_Rain_Status_Flag = false;
		return false;
	}
	else {
		return false;
	}
}