#ifndef _PRIVATE_SENSOR_H
#define _PRIVATE_SENSOR_H

#define g_Wait_Receive_Time 500
#define g_Wait_Collect_Time 100

#define LUX_UV            1

#define PR_3000_ECTH_N01  1
#define RS_FXJT_N01       1   //风向变送器
#define RS_FSJT_N01       1   //风速变送器

//RS485传感器地址定义
#define RAINFALL_SENSOR_ADDR        0x02  //雨雪传感器
#define WIND_RATE_SENSOR_ADDR       0x06  //风速传感器
#define WIND_DIRECTION_SENSOR_ADDR  0x07  //风向传感器

//I2C传感器地址定义
#define I2C_ADDR 0x38 //UV sensor


//Integration Time
#define IT_1_2 0x0 //1/2T
#define IT_1   0x1 //1T
#define IT_2   0x2 //2T
#define IT_4   0x3 //4T

struct SENSOR_DATA{
  float             g_Temp;             //温度
  unsigned char     g_Temp_Flag;        //
  float             g_Humi;             //湿度
  float             g_Air_Wind_Speed;   //风速
  unsigned int      g_Wind_DirCode;     //风向
  unsigned long int g_Lux;              //光照强度
  unsigned int      g_Cond;             //
  unsigned int      g_UV;               //紫外线
  unsigned char     g_Is_Rain_or_Snow;  //是否下雨
};

class Sensor{
public:
  bool Detect_Rain(void);
  void Get_All_Sensor_Data(void);
private:
  void CJMCU6750_Init();
  void Read_Lux_and_UV(unsigned long *lux, unsigned int *uv);
  void Read_Rainfall(unsigned char *rainfall, unsigned char address);
  void Read_Wind_Speed(float *wind_speed, unsigned int address);
  void Read_Wind_Direction(unsigned int *wind_direction, unsigned char address);
};

extern struct SENSOR_DATA Sensor_Data;
extern Sensor private_sensor;

extern bool g_Current_Rain_Status_Flag;
extern bool g_Last_Rain_Status_Flag;

extern bool g_Send_Air_SensorData_Flag;

#endif