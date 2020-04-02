#include "private_sensor.h"
#include <Arduino.h>
#include "RS485.h"
#include "MODBUS_RTU_CRC16.h" 
#include "MAX44009/i2c_MAX44009.h"
#include "SHT1x.h"
#include "fun_periph.h"
#include "Adafruit_ADS1015.h"

struct SENSOR_DATA Sensor_Data;
Sensor private_sensor;

void Sensor::Get_All_Sensor_Data(void)
{
	int retry_num = 0;	int retry = 3;
	PWR_485_ON;

	delay(1000);//经过测试，新版的PH的测量，为了使温湿度数据完整读出，需要等待温湿度传感器初始化完成。所以增加1s的延时，否则可能出现温度数据读取不出来。
	Sensor_Data.g_Temp = sht10.readTemperatureC();
	delay(100);
	Sensor_Data.g_Humi = sht10.readHumidity();
	delay(100);
	while (private_sensor.Read_Solid_Humi_and_Temp(&Sensor_Data.g_Solid_Humi, &Sensor_Data.g_Solid_Temp, &Sensor_Data.g_Solid_Temp_Flag, 0x01) != 1 && retry_num < retry)
	{
		retry_num++;
	}
	retry_num = 0;
	delay(100);
	while ((private_sensor.Read_Salt_and_Cond(&Sensor_Data.g_Salt, &Sensor_Data.g_Cond, 0x01) != 1) && (retry_num < retry))
	{
		retry_num++;
	}
	retry_num = 0;
	delay(100);
	while ((private_sensor.Read_Soild_PH(&Sensor_Data.g_Solid_PH, 0x02) != 1) && (retry_num < retry))
	{
		retry_num++;
	}
	delay(100);
	private_sensor.Read_Lux_and_UV(&Sensor_Data.g_Lux, &Sensor_Data.g_UV);
	delay(100);
	private_sensor.Read_Displacement(&Sensor_Data.g_Displacement);
	delay(100);


	// PWR_485_OFF;

	Serial.print("temperature: ");//空气温度
	Serial.println(String(Sensor_Data.g_Temp) + "℃");
	Serial.print("humility: ");//空气湿度
	Serial.println(String(Sensor_Data.g_Humi) + "%RH");
	Serial.print("Lux: ");//光照强度
	Serial.println(String(Sensor_Data.g_Lux) + "lux");
	Serial.print("UV: ");//紫外线强度
	Serial.println(String(Sensor_Data.g_UV) + "W/m2");
	Serial.print("Solid temperature: ");//土壤温度
	Serial.println(String(Sensor_Data.g_Solid_Temp / 100) + "℃");
	Serial.print("Solid PH: ");//土壤酸碱度
	Serial.println(String(float(Sensor_Data.g_Solid_PH)/100) + "");
	Serial.print("Solid humility: ");//土壤湿度
	Serial.println(String(Sensor_Data.g_Solid_Humi) + "%RH");
	Serial.print("Solid salt: ");//土壤盐度
	Serial.println(String(Sensor_Data.g_Salt) + "mg/L");
	Serial.print("Solid cond: ");//土壤电导率
	Serial.println(String(Sensor_Data.g_Cond) + "us/cm");
	Serial.print("Displacement:");//植物杆径
	Serial.println(String(Sensor_Data.g_Displacement) + "μm");
}

/*
 *brief   : Initialize UV and Illumination sensors.
 *para    : None
 *return  : None
 */
void Sensor::CJMCU6750_Init(void)
{
	CJMCU6750.begin(PB15, PB14);

	CJMCU6750.beginTransmission(UV_I2C_ADDR);
	CJMCU6750.write((IT_1 << 2) | 0x02);//0x100|0x02 = 0x102
	CJMCU6750.endTransmission();
	delay(500);

	if (max44009.initialize())  //Lux
		Serial.println("Sensor MAX44009 found...");
	else
		Serial.println("Light Sensor missing !!!");
}

/*
 *brief   : Initialize UV and Illumination sensors.
 *para    : None
 *return  : None
 */
void Sensor::ADS115_Init(void)
{

	// CJMCU6750.begin(PB15, PB14);

	// CJMCU6750.beginTransmission(UVI2C_ADDR);
	// CJMCU6750.write((IT_1 << 2) | 0x02);
	// CJMCU6750.endTransmission();
	// delay(500);

	// if (max44009.initialize())  //Lux
	// 	Serial.println("Sensor MAX44009 found...");
	// else
	// 	Serial.println("Light Sensor missing !!!");
}

/*
 *brief   : Read Lux and UV from sensor (I2C)
 *para    : Lux, UV
 *return  : None
 */
void Sensor::Read_Lux_and_UV(unsigned long* lux, unsigned int* uv)
{
	unsigned char msb = 0, lsb = 0;
	unsigned long mLux_value = 0;

	CJMCU6750_Init();

#if LUX_UV
	CJMCU6750.requestFrom(UV_I2C_ADDR + 1, 1); //MSB
	delay(1);

	if (CJMCU6750.available())
		msb = CJMCU6750.read();

	CJMCU6750.requestFrom(UV_I2C_ADDR + 0, 1); //LSB
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
 *brief   : According to the ID address and register(ModBus) address of the soil sensor, read temperature and humidity
 *para    : humidity, temperature, address
 *return  : None
 */
bool Sensor::Read_Solid_Humi_and_Temp(float* humi, unsigned int* temp, unsigned char* temp_flag, unsigned char addr)
{
#if PR_3000_ECTH_N01
	unsigned char Send_Cmd[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00 };
#else
	unsigned char Send_Cmd[8] = { 0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00 };
#endif
	unsigned char Receive_Data[10] = { 0 };
	unsigned char Length = 0;
	unsigned int Send_CRC16 = 0, Receive_CRC16 = 0, Verify_CRC16 = 0;
	float hum = 65535.0;
	unsigned int humi_temp = 0xFFFF, tem_temp = 0xFFFF;
	unsigned char temperature_flag = 0;

	Send_Cmd[0] = addr; //Get sensor address

	Send_CRC16 = N_CRC16(Send_Cmd, 6);
	Send_Cmd[6] = Send_CRC16 >> 8;
	Send_Cmd[7] = Send_CRC16 & 0xFF;

	RS485_Serial.write(Send_Cmd, 8);
	delay(300);

	while (RS485_Serial.available() > 0) 
	{
		if (Length >= 9)
		{
			Serial.println("土壤温湿度回执Length >= 9");
			Length = 0;
			break;
		}
		Receive_Data[Length++] = RS485_Serial.read();
	}
	//Serial.println(String("土壤温湿度回执Length = ") + Length);

	Serial.println("---土壤温湿度回执---");
	if (Length > 0)
	{
		for (size_t i = 0; i < Length; i++)
		{
			Serial.print(Receive_Data[i], HEX);
			Serial.print(" ");
		}
	}
	Serial.println("");
	Serial.println("---土壤温湿度回执---");

	if (Receive_Data[0] == 0) {
		Verify_CRC16 = N_CRC16(&Receive_Data[1], 7);
		Receive_CRC16 = Receive_Data[8] << 8 | Receive_Data[9];
	}
	else {
		Verify_CRC16 = N_CRC16(Receive_Data, 7);
		Receive_CRC16 = Receive_Data[7] << 8 | Receive_Data[8];
	}

	if (Receive_CRC16 == Verify_CRC16) {
		if (Receive_Data[0] == 0) {
			humi_temp = Receive_Data[4] << 8 | Receive_Data[5];
			tem_temp = Receive_Data[6] << 8 | Receive_Data[7];
		}
		else {
			humi_temp = Receive_Data[3] << 8 | Receive_Data[4];
			tem_temp = Receive_Data[5] << 8 | Receive_Data[6];
		}

#if PR_3000_ECTH_N01
		hum = (float)humi_temp / 100.0;
#else
		hum = (float)humi_temp / 10.0;
#endif

		temperature_flag = ((tem_temp >> 15) & 0x01);
	}
	else
	{
		Serial.println("Read Solid Humi and Temp Error!!!<Read_Solid_Humi_and_Temp>");
		return 0;
	}
	

	*humi = hum;
	*temp = tem_temp;
	*temp_flag = temperature_flag;
	return 1;
}

/*
 *brief   : According to the ID address and register(ModBus) address of the soil sensor, read PH
 *para    : Solid_PH, address
 *return  : None
 */
bool Sensor::Read_Soild_PH(unsigned int * Solid_PH, unsigned char addr)
{
#if ST_500_Soil_PH
	unsigned char Send_Cmd[8] = { 0x02, 0x03, 0x00, 0x08, 0x00, 0x01,0x00,0x00 };//02 03 0008 0001 05C8
#elif JXBS_3001_PH
	unsigned char Send_Cmd[8] = { 0x02, 0x03, 0x00, 0x06, 0x00, 0x01, 0x00, 0x00 };
#endif
	unsigned char Receive_Data[9] = { 0 };
	unsigned char Length = 0;
	unsigned int Send_CRC16 = 0, Receive_CRC16 = 0, Verify_CRC16 = 0;
	unsigned int ph_temp = 0xFFFF;

	Send_Cmd[0] = addr; //Get sensor address

	Send_CRC16 = N_CRC16(Send_Cmd, 6);
	Send_Cmd[6] = Send_CRC16 >> 8;
	Send_Cmd[7] = Send_CRC16 & 0xFF;

	RS485_Serial.write(Send_Cmd, 8);
	delay(300);

	while (RS485_Serial.available() > 0)//02 03 02 02BC FC95就是0x2BC = 700,结果需要除以100，那么就是7
	{
		if (Length >= 8)
		{
			Serial.println("土壤酸碱度回执Length >= 8");
			Length = 0;
			break;
		}
		Receive_Data[Length++] = RS485_Serial.read();
	}

	//Serial.println(String("土壤酸碱度回执Length = ") + Length);

	Serial.println("---土壤酸碱度回执---");
	if (Length > 0)
	{
		for (size_t i = 0; i < Length; i++)
		{
			Serial.print(Receive_Data[i], HEX);
			Serial.print(" ");
		}
	}
	Serial.println("");
	Serial.println("---土壤酸碱度回执---");

	Verify_CRC16 = N_CRC16(Receive_Data, 5);
	Receive_CRC16 = Receive_Data[5] << 8 | Receive_Data[6];

	if (Receive_CRC16 == Verify_CRC16) {
		//Serial.println("SUCCESS");
		//delay(3000);
		ph_temp = Receive_Data[3] << 8 | Receive_Data[4];
		Serial.println(String("ph_temp = ")+ph_temp);
	}
	else
	{
		Serial.println("PH Read Error!!!<Read_Soild_PH>");
		return 0;
	}
#if ST_500_Soil_PH
	*Solid_PH = ph_temp * 10;
	return 1;
#elif JXBS_3001_PH
	*Solid_PH = ph_temp;
	return 1;
#endif
}

/*
 *brief   : According to the ID address and register(ModBus) address of the soil sensor, read salt and conductivity
 *para    : salt, conductivity, address
 *return  : None
 */
bool Sensor::Read_Salt_and_Cond(unsigned int* salt, unsigned int* cond, unsigned char addr)
{
#if PR_3000_ECTH_N01
	unsigned char Send_Cmd[8] = { 0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00 };
#else
	unsigned char Send_Cmd[8] = { 0x01, 0x03, 0x00, 0x14, 0x00, 0x02, 0x00, 0x00 };
#endif
	unsigned char Receive_Data[10] = { 0 };
	unsigned char Length = 0;
	unsigned int Send_CRC16 = 0, Receive_CRC16 = 0, Verify_CRC16 = 0;
	unsigned int salt_temp = 0xFFFF;
	unsigned int cond_temp = 0xFFFF;

	Send_Cmd[0] = addr; //Get sensor address

	Send_CRC16 = N_CRC16(Send_Cmd, 6);
	Send_Cmd[6] = Send_CRC16 >> 8;
	Send_Cmd[7] = Send_CRC16 & 0xFF;

	RS485_Serial.write(Send_Cmd, 8);
	delay(300);

	while (RS485_Serial.available() > 0)
	{
		if (Length >= 9)
		{
			Serial.println("土壤盐电导率回执Length >= 9");
			Length = 0;
			break;
		}
		Receive_Data[Length++] = RS485_Serial.read();
	}
	//Serial.println(String("土壤盐电导率回执Length = ") + Length);
	Serial.println("---土壤盐电导率回执---");
	if (Length > 0)
	{
		for (size_t i = 0; i < 9; i++)
		{
			Serial.print(Receive_Data[i], HEX);
			Serial.print(" ");
		}
	}
	Serial.println("");
	Serial.println("---土壤盐电导率回执---");

	Verify_CRC16 = N_CRC16(Receive_Data, 7);
	Receive_CRC16 = Receive_Data[7] << 8 | Receive_Data[8];

	if (Receive_CRC16 == Verify_CRC16) {
		salt_temp = Receive_Data[5] << 8 | Receive_Data[6];
		cond_temp = Receive_Data[3] << 8 | Receive_Data[4];
	}
	else
	{
		Serial.println("Read Salt and Cond Error!!!<Read_Salt_and_Cond>");
		return 0;
	}
	

	*salt = salt_temp;
	*cond = cond_temp;
	return 1;
}

/*
 *brief   : Read Displacement from sensor (ADC)
 *para    : Displacement
 *return  : None
 */
void Sensor::Read_Displacement(unsigned int *Displacement)
{
	int16_t adc0, adc1, adc2, adc3;
	float Voltage = 0.0;//声明一个变量 用于存储检测到的电压值
	unsigned int Displacement_temp = 0xFF;
	adc0 = ads.readADC_SingleEnded(0);
	adc1 = ads.readADC_SingleEnded(1);
	adc2 = ads.readADC_SingleEnded(2);
	adc3 = ads.readADC_SingleEnded(3);

	// Serial.print("AIN0: "); Serial.println(adc0);
	// Serial.print("AIN1: "); Serial.println(adc1);
	// Serial.print("AIN2: "); Serial.println(adc2);
	// Serial.print("AIN3: "); Serial.println(adc3);
	if (adc3 < 0)
	{
		//读取错误，异常处理
		adc3 = 0;
	}
	
	Voltage = (adc3 * 0.1875);    //把数值转换成电压值，0.1875是电压因数
	Serial.println(String("Voltage = ") + String(Voltage,2) + "mv");
	
	//0-3300mv对应0-20mm	==>	0-3300000μv对应0-20000μm
	//1mv对应0.006mm=6μm	==>	1μv对应0.006μm
	Displacement_temp = int(Voltage*6);
	Serial.println(String("Displacement_temp = ") + String(Displacement_temp) + "μm");
	Serial.println("---------");Serial.flush();

	*Displacement = Displacement_temp;
}

/**
 * [ADS1115Configure description]
 * @Author     叶鹏程
 * @DateTime   2019-08-19T20:44:30+0800
 * @discrption : 芯片初始化配置
 *  
 * @param      mux                      [输入通道配置]
 * @param      pga                      [量程设置]
 * @param      mode                     [单次或持续转换模式]
 * @param      dr                       [采样速率设置]
 */
void ADS1115Configure(ADS1115Conf mux, ADS1115Conf pga, ADS1115Conf mode, ADS1115Conf dr){

  uint16_t conf;

  ADS1115.beginTransmission(ADS1115_I2C_address);
  ADS1115.write(0x01);  //set Address Pointer Register as conf Register将地址指针寄存器设置为conf寄存器
  ADS1115.endTransmission();

  /*read chip's conf register data */
  ADS1115.requestFrom(ADS1115_I2C_address, 2);
  conf = 0;
  conf = ADS1115.read();
  conf = conf<<8;
  conf |= ADS1115.read();

  /*change it*/
  conf = 0x8000;
  conf &= (~(0x0007<< 12)); conf |= mux   << 12;
  conf &= (~(0x0007<< 9 )); conf |= pga   << 9;
  conf &= (~(0x0007<< 8 )); conf |= mode  << 8;
  conf &= (~(0x0007<< 5 )); conf |= dr    << 5;

  // conf = 0xf483;
  /* trans back*/
   ADS1115.beginTransmission(ADS1115_I2C_address);
   ADS1115.write(0x01);  //set Address Pointer Register as conf Register将地址指针寄存器设置为conf寄存器
   ADS1115.write(uint8_t(conf>>8));  //conf MSB
   ADS1115.write(uint8_t(conf));  //conf LSB
   ADS1115.endTransmission();

    /**/
  ADS1115.beginTransmission(ADS1115_I2C_address);
  ADS1115.write(0x00);  //set Address Pointer Register as conf Register将地址指针寄存器设置为conf寄存器
  ADS1115.endTransmission();
}

/**
 * [ADS1115SetChannel description]
 * @Author     叶鹏程
 * @DateTime   2019-08-19T20:43:57+0800
 * @discrption :芯片输入通道设置
 *  
 * @param      mux                      [description]
 */
void ADS1115SetChannel(ADS1115Conf mux){

  uint16_t conf;

  ADS1115.beginTransmission(ADS1115_I2C_address);
  ADS1115.write(0x01);  //set Address Pointer Register as conf Register
  ADS1115.endTransmission();

  /*read chip's conf register data */
  ADS1115.requestFrom(ADS1115_I2C_address, 2);
  conf = 0;
  conf = ADS1115.read();
  conf = conf<<8;
  conf |= ADS1115.read();

  /*change it*/
  conf = conf & (~(0x0007<< 12)); conf |= mux << 12;
  

  /* trans back*/
  ADS1115.beginTransmission(ADS1115_I2C_address);
  ADS1115.write(0x01);  //set Address Pointer Register as conf Register
  ADS1115.write(uint8_t(conf>>8));  //conf MSB
  ADS1115.write(uint8_t(conf));  //conf LSB
  ADS1115.endTransmission();

   /**/
  ADS1115.beginTransmission(ADS1115_I2C_address);
  ADS1115.write(0x00);  //set Address Pointer Register as conf Register
  ADS1115.endTransmission();
}

long ADS1115ReadResult(void){
  long ret;

  ADS1115.requestFrom(ADS1115_I2C_address, 2);
  ret = 0;
  ret = ADS1115.read();
  Serial.println(String("ret1 = ") + ret);
  ret = ret<<8;
  ret |= ADS1115.read();
  Serial.println(String("ret12= ") + ret);

  return ret;
}