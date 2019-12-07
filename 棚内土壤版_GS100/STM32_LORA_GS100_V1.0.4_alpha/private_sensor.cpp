#include "private_sensor.h"
#include <Arduino.h>
#include "RS485.h"
#include "MODBUS_RTU_CRC16.h" 
#include "MAX44009/i2c_MAX44009.h"
#include "SHT1x.h"

struct SENSOR_DATA Sensor_Data;
Sensor private_sensor;

void Sensor::Get_All_Sensor_Data(void)
{
    PWR_485_ON;

	delay(100);
    Sensor_Data.g_Temp = sht10.readTemperatureC();
    delay(100);
    Sensor_Data.g_Humi = sht10.readHumidity();
    delay(100);
    private_sensor.Read_Solid_Humi_and_Temp(&Sensor_Data.g_Solid_Humi, &Sensor_Data.g_Solid_Temp, &Sensor_Data.g_Solid_Temp_Flag, 0x01);
    delay(100);
    private_sensor.Read_Salt_and_Cond(&Sensor_Data.g_Salt, &Sensor_Data.g_Cond, 0x01);
    delay(100);
    private_sensor.Read_Lux_and_UV(&Sensor_Data.g_Lux, &Sensor_Data.g_UV);
    delay(100);

    PWR_485_OFF;

    Serial.print("temperature: ");
    Serial.println(Sensor_Data.g_Temp);
    Serial.print("humility: ");
    Serial.println(Sensor_Data.g_Humi);
    Serial.print("Solid temperature: ");
    Serial.println(Sensor_Data.g_Solid_Temp / 100);
    Serial.print("Solid humility: ");
    Serial.println(Sensor_Data.g_Solid_Humi);
    Serial.print("Solid salt: ");
    Serial.println(Sensor_Data.g_Salt);
    Serial.print("Solid cond: ");
    Serial.println(Sensor_Data.g_Cond);
}

/*
 *brief   : Initialize UV and Illumination sensors.
 *para    : None
 *return  : None
 */
void Sensor::CJMCU6750_Init(void)
{
  CJMCU6750.begin(PB15, PB14);

  CJMCU6750.beginTransmission(I2C_ADDR);
  CJMCU6750.write((IT_1 << 2) | 0x02);
  CJMCU6750.endTransmission();
  delay(500);

  if (max44009.initialize())  //Lux
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
 *brief   : According to the ID address and register(ModBus) address of the soil sensor, read temperature and humidity
 *para    : humidity, temperature, address
 *return  : None
 */
void Sensor::Read_Solid_Humi_and_Temp(float *humi, unsigned int *temp, unsigned char *temp_flag, unsigned char addr)
{
  #if PR_3000_ECTH_N01
  unsigned char Send_Cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00};
  #else
  unsigned char Send_Cmd[8] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00};
  #endif
  unsigned char Receive_Data[10] = {0};
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
  delay(100);
  
  while (RS485_Serial.available() > 0){
    Receive_Data[Length++] = RS485_Serial.read();
  }

  if (Receive_Data[0] == 0){
    Verify_CRC16 = N_CRC16(&Receive_Data[1], 7);
    Receive_CRC16 = Receive_Data[8] << 8 | Receive_Data[9];
  }else{
    Verify_CRC16 = N_CRC16(Receive_Data, 7);
    Receive_CRC16 = Receive_Data[7] << 8 | Receive_Data[8];
  }

  if (Receive_CRC16 == Verify_CRC16){
    if (Receive_Data[0] == 0){
      humi_temp = Receive_Data[4] << 8 | Receive_Data[5];
      tem_temp  = Receive_Data[6] << 8 | Receive_Data[7];
    }else{
      humi_temp = Receive_Data[3] << 8 | Receive_Data[4];
      tem_temp  = Receive_Data[5] << 8 | Receive_Data[6];     
    }

    #if PR_3000_ECTH_N01
      hum = (float)humi_temp / 100.0;
    #else
      hum = (float)humi_temp / 10.0;
    #endif

    temperature_flag = ((tem_temp >> 15) & 0x01);
  }

  *humi = hum;
  *temp = tem_temp;
  *temp_flag = temperature_flag;
}

/*
 *brief   : According to the ID address and register(ModBus) address of the soil sensor, read salt and conductivity
 *para    : salt, conductivity, address
 *return  : None
 */
void Sensor::Read_Salt_and_Cond(unsigned int *salt, unsigned int *cond, unsigned char addr)
{
  #if PR_3000_ECTH_N01
  unsigned char Send_Cmd[8] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00};
  #else
  unsigned char Send_Cmd[8] = {0x01, 0x03, 0x00, 0x14, 0x00, 0x02, 0x00, 0x00};
  #endif
  unsigned char Receive_Data[9] = {0};
  unsigned char Length = 0;
  unsigned int Send_CRC16 = 0, Receive_CRC16 = 0, Verify_CRC16 = 0;
  unsigned int salt_temp = 0xFFFF;
  unsigned int cond_temp = 0xFFFF;

  Send_Cmd[0] = addr; //Get sensor address
  
  Send_CRC16 = N_CRC16(Send_Cmd, 6);
  Send_Cmd[6] = Send_CRC16 >> 8;
  Send_Cmd[7] = Send_CRC16 & 0xFF;
  
  RS485_Serial.write(Send_Cmd, 8);
  delay(100);
  
  while (RS485_Serial.available() > 0)
  {
    Receive_Data[Length++] = RS485_Serial.read();
    if (Length >= 9)
      Length = 0;
  }

  Verify_CRC16 = N_CRC16(Receive_Data, 7);
  Receive_CRC16 = Receive_Data[7] << 8 | Receive_Data[8];

  if (Receive_CRC16 == Verify_CRC16){
    salt_temp = Receive_Data[5] << 8 | Receive_Data[6];
    cond_temp = Receive_Data[3] << 8 | Receive_Data[4];
  }

  *salt = salt_temp;
  *cond = cond_temp;
}