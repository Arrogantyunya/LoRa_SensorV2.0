#ifndef _PRIVATE_SENSOR_H
#define _PRIVATE_SENSOR_H

#define LUX_UV            1

#define PR_3000_ECTH_N01  1

#define I2C_ADDR 0x38 //UV sensor
//Integration Time
#define IT_1_2 0x0 //1/2T
#define IT_1   0x1 //1T
#define IT_2   0x2 //2T
#define IT_4   0x3 //4T

#define RAINFALL_SENSOR_ADDR                0x02

struct SENSOR_DATA{
  float             g_Temp;
  unsigned char     g_Temp_Flag;
  float             g_Humi;
  unsigned long int g_Lux;
  unsigned int      g_Cond;
  unsigned int      g_UV;
  unsigned char     g_Is_Rain_or_Snow;
};

class Sensor{
public:
  bool Detect_Rain(void);
  void Get_All_Sensor_Data(void);
private:
  void CJMCU6750_Init();
  void Read_Lux_and_UV(unsigned long *lux, unsigned int *uv);
  void Read_Rainfall(unsigned char *rainfall, unsigned char address);
};

extern struct SENSOR_DATA Sensor_Data;
extern Sensor private_sensor;

extern bool g_Current_Rain_Status_Flag;
extern bool g_Last_Rain_Status_Flag;

extern bool g_Send_Air_SensorData_Flag;

#endif