#ifndef _PRIVATE_SENSOR_H
#define _PRIVATE_SENSOR_H

#define LUX_UV            1

#define PR_3000_ECTH_N01  1

#define ST_500_Soil_PH		0//一款长的PH，第一代的PH
#define JXBS_3001_PH      1//两根针的款式，第二代的PH

#define UV_I2C_ADDR 0x38 //UV sensor
#define ADS1115_I2C_address 0x48 //位移传感器

enum ADS1115Conf{
  AIN0_2AIN1  =  0b000, // << 12,
  AIN0_2AIN3  =  0b001, // << 12,
  AIN1_2AIN3  =  0b010, // << 12,
  AIN2_2AIN3  =  0b011, // << 12,
  AIN0_2GND   =  0b100, // << 12,
  AIN1_2GND   =  0b101, // << 12,
  AIN2_2GND   =  0b110, // << 12,
  AIN3_2GND   =  0b111, // << 12,

  FSR_6_144V  =  0b000, // << 9,
  FSR_4_096V  =  0b001, // << 9,
  FSR_2_048V  =  0b010, // << 9,
  FSR_1_024V  =  0b011, // << 9,
  FSR_0_512V  =  0b100, // << 9,
  FSR_0_256V  =  0b101, // << 9,

  CONTINU_CONV = 0, //<<8
  SINGLE_SHOT =  1,

  SPS_8       =  0b000, // << 5,
  SPS_16      =  0b001, // << 5,
  SPS_32      =  0b010, // << 5,
  SPS_64      =  0b011, // << 5,
  SPS_128     =  0b100, // << 5,
  SPS_250     =  0b101, // << 5,
  SPS_475     =  0b110, // << 5,
  SPS_860     =  0b111, // << 5,

}typedef ADS1115Conf;

//Integration Time
#define IT_1_2 0x0 //1/2T
#define IT_1   0x1 //1T
#define IT_2   0x2 //2T
#define IT_4   0x3 //4T

struct SENSOR_DATA{
  float             g_Temp;//温度
  unsigned char     g_Temp_Flag;//
  float             g_Humi;//相对湿度
  unsigned long int g_Lux;//光照强度
  unsigned int      g_Solid_Temp;//土壤温度
  unsigned char     g_Solid_Temp_Flag;//
  float             g_Solid_Humi;//土壤湿度
  unsigned int      g_Salt;//土壤盐分
  unsigned int      g_Cond;//土壤电导率
  unsigned int      g_UV;//紫外线强度
  unsigned int      g_Solid_PH;//土壤PH值（由原来的大气压力转变）
  unsigned int      g_Displacement;//位移量
  //二氧化碳浓度
  //TVOCs浓度
};

class Sensor{
public:
  void Get_All_Sensor_Data(void);
private:
  void CJMCU6750_Init();
  void ADS115_Init();
  void Read_Lux_and_UV(unsigned long *lux, unsigned int *uv);
  bool Read_Solid_Humi_and_Temp(float *humi, unsigned int *temp, unsigned char *temp_flag, unsigned char addr);
  bool Read_Soild_PH(unsigned int *Solid_PH, unsigned char addr);
  bool Read_Salt_and_Cond(unsigned int *salt, unsigned int *cond, unsigned char addr);
  void Read_Displacement(unsigned int *Displacement);
};

void ADS1115Configure(ADS1115Conf mux, ADS1115Conf pga, ADS1115Conf mode, ADS1115Conf dr);
void ADS1115SetChannel(ADS1115Conf mux);
long ADS1115ReadResult(void);

extern struct SENSOR_DATA Sensor_Data;
extern Sensor private_sensor;

#endif