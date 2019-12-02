#ifndef _PRIVATE_SENSOR_H
#define _PRIVATE_SENSOR_H

#define LUX_UV				1

#define PR_3000_ECTH_N01	1

#define ST_500_Soil_PH		1

#define I2C_ADDR 0x38 //UV sensor
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
  unsigned int      g_Solid_PH;//大气压力（改为了土壤PH值）
  //二氧化碳浓度
  //TVOCs浓度
};

class Sensor{
public:
  void Get_All_Sensor_Data(void);
private:
  void CJMCU6750_Init();
  void Read_Lux_and_UV(unsigned long *lux, unsigned int *uv);
  void Read_Solid_Humi_and_Temp(float *humi, unsigned int *temp, unsigned char *temp_flag, unsigned char addr);
  void Read_Soild_PH(unsigned int *Solid_PH, unsigned char addr);
  void Read_Salt_and_Cond(unsigned int *salt, unsigned int *cond, unsigned char addr);
};

extern struct SENSOR_DATA Sensor_Data;
extern Sensor private_sensor;

#endif