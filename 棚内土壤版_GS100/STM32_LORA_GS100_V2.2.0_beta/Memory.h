#ifndef _Memory_H
#define _Memory_H

#include "AT24CXX.h"
#include <libmaple/bkp.h>
#include <Arduino.h>

#define EP_WP_PIN                           PB5

/*使用EEPROM储存芯片的宏定义地址*/
/*EEPROM*/
#define EEPROM_MIN_ADDR                     0
#define EEPROM_MAX_ADDR                     255 
/*SN码相关操作保存地址*/
#define SN_OPERATION_FLAG_ADDR                  11
#define SN_BKP_OPERATION_FLAG_ADDR              12
#define SN_BASE_ADDR                            13
#define SN_END_ADDR                             21
#define SN_VERIFY_ADDR                          22
#define SN_BKP_BASE_ADDR                        23
#define SN_BKP_END_ADDR                         31
#define SN_BKP_VERIFY_ADDR                      32
#define SN_ACCESS_NETWORK_FLAG_ADDR             33
//软件版本和硬件版本保存地址
//Software version and hardware version save address.
#define SOFT_VERSION_BASE_ADDR         34
#define HARD_VERSION_BASE_ADDR         36
//工作组号
//work group
#define GROUP_NUMBER_BASE_ADDR              38
#define GROUP_NUMBER_END_ADDR               42
#define GROUP_NUMBER_VERIFY_ADDR            43
#define GROUP_NUMBER_FLAG_ADDR              44
//工作区域号
//work area number
#define AREA_ADDR                           45
#define AREA_VERIFY_ADDR                    46
#define AREA_FLAG_ADDR                      47

#define COLLECT_TIME_HIGH_ADDR              50
#define COLLECT_TIME_LOW_ADDR               51
#define COLLECT_TIME_FLAG_ADDR              52

#define LORA_PARA_CONFIG_FLAG_ADDR          53

/* LoRa通信模式配置成网关还是节点 0xF0节点; 0xF1网关 */
#define LORA_COM_MODE_ADDR                      76
#define LORA_COM_MODE_FLAG_ADDR                 77
#define LORA_COM_MODE_VERIFY_ADDR               78

#define EP_LORA_ADDR_BASE_ADDR                  79
#define EP_LORA_ADDR_END_ADDR                   86
#define EP_LORA_ADDR_VERIFY_ADDR                87
#define EP_LORA_ADDR_SAVED_FLAG_ADDR            88

#define Lora_TRMode_ADDR                        89
#define Lora_TRMode_Set_ADDR                    90
#define LORA_SYNC_ADDR                          91
#define Lora_SYNC_Set_ADDR                      92

class EEPROM_Operations : protected AT24Cxx{
public:
    void EEPROM_GPIO_Config(void);
protected:
    void EEPROM_Write_Enable(void);
    void EEPROM_Write_Disable(void);
};

class SN_Operations : public EEPROM_Operations{
public:
    bool Save_SN_Code(unsigned char *dat);
    bool Save_BKP_SN_Code(unsigned char *dat);
    bool Read_SN_Code(unsigned char *dat);
    bool Read_BKP_SN_Code(unsigned char *dat);
    void Read_Random_Seed(unsigned char *random_seed);
    bool Verify_Save_SN_Code(void);
    bool Verify_Save_BKP_SN_Code(void);
    bool Clear_SN_Save_Flag(void);
    bool Clear_BKP_SN_Save_Flag(void);
    bool Set_SN_Access_Network_Flag(void);
    bool Clear_SN_Access_Network_Flag(void);
    bool Verify_SN_Access_Network_Flag(void);
    bool Self_Check(unsigned char *dat);
};

class LoRa_Config : public EEPROM_Operations {
public:
	bool Save_LoRa_Config_Flag(void);
	bool Verify_LoRa_Config_Flag(void);
	bool Clear_LoRa_Config_Flag(void);

	bool Save_LoRa_Com_Mode_Flag(void);
	bool Clear_LoRa_Com_Mode_Flag(void);
	bool Save_LoRa_Com_Mode(unsigned char mode);
	unsigned char Read_LoRa_Com_Mode(void);

	void Clear_LoRa_Addr_Flag(void);
	void Save_LoRa_Addr_Flag(void);
	bool Verify_LoRa_Addr_Flag(void);
	bool Read_LoRa_Addr(unsigned char* addr);
	bool Save_LoRa_Addr(unsigned char* addr);

    bool Verify_LoRa_TRMode_Flag(void);//验证Lora的TRmode是否被配置过
    bool Save_LoRa_TRMode(unsigned char TRMode);//保存TRMode
    unsigned char Read_LoRa_TRMode(void);//读取TRMode

    bool Verify_LoRa_SYNC_Flag(void);//验证Lora的SYNC是否被配置过
    bool Save_LoRa_SYNC(unsigned char SYNC);//保存SYNC
    unsigned char Read_LoRa_SYNC(void);//读取SYNC
};

class Soft_Hard_Vertion : public EEPROM_Operations {
public:
	void Save_hardware_version(unsigned char number_high, unsigned char number_low);
	void Save_Software_version(unsigned char number_high, unsigned char number_low);
	unsigned char Read_hardware_version(unsigned char number_addr);
	unsigned char Read_Software_version(unsigned char number_addr);
};

class Control_Information : public EEPROM_Operations{
public:
    bool Save_Group_Number(unsigned char *group_num);
    void Read_Group_Number(unsigned char *group_num);
    bool Check_Group_Number(void);
    bool Verify_Group_Number_Flag(void);
    bool Clear_Group_Number(void);

    bool Save_Area_Number(unsigned char area_num);
    unsigned char Read_Area_Number(void);
    bool Check_Area_Number(void);
    bool Verify_Area_Number_Flag(void);
    bool Clear_Area_Number(void);

    bool Save_Roll_Work_Voltage_and_Report_Interval(unsigned char *voltage_value);
    unsigned int Read_Roll_Low_Voltage_Limit_Value(void);
    unsigned int Read_Roll_High_Voltage_Limit_Value(void);
    unsigned char Read_Roll_Report_Status_Interval_Value(void);
};

class Work_Parameter_Information : public EEPROM_Operations{
public:
    bool Save_Collect_Time(unsigned int time);
    unsigned int Read_Collect_Time(void);
    bool Verify_Collect_Time_Flag(void);
};

/*Create EEPROM object*/
extern EEPROM_Operations EEPROM_Operation;
/*Create SN code operation object*/
extern SN_Operations SN;
/*Create control information object*/
extern Control_Information Control_Info;
/*Create software and hardware object*/
extern Soft_Hard_Vertion Vertion;
/*Create work parameters information object*/
extern Work_Parameter_Information WorkParameter_Info;

extern LoRa_Config LoRa_Para_Config;

#endif
