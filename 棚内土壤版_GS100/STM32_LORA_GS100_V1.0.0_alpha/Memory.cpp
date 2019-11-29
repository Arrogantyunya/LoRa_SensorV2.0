/************************************************************************************
 * Code and comments : KeeganLu
 * Date：2019/3/11
 * 
 * The main function of this file is to save, verify, operate and read the information
 * related to the device, such as the SN code, software and hardware version number,
 * area number, workgroup number, etc. Private information such as motor roll opening,
 * measuring distance flag, etc. A common interface for each class is provided in the
 * header file.
 * 
 * If you have any questions, please send an email to me： idlukeqing@163.com
*************************************************************************************/

/************************************************************************************
 * 
 * 代码与注释：卢科青
 * 日期：2019/3/11
 * 该文件主要功能是依托EEPROM和芯片自带备份寄存器，保存、校验、操作、读取设备相关的信息，
 * 如通用的SN码，软件、硬件版本号，区域号，工作组号等。私有信息如电机卷膜开度、测量行程标志
 * 位等。头文件中提供了各个类的公共接口。
 * 
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "Memory.h"
#include "User_CRC8.h"
#include <libmaple/bkp.h>
#include "public.h"
#include "fun_periph.h"

/*Create EEPROM object*/
EEPROM_Operations EEPROM_Operation;
/*Create SN code operation object*/
SN_Operations SN;
/*Create LoRa Config object*/
LoRa_Config LoRa_Para_Config;
/*Create control information object*/
Control_Information Control_Info;
/*Create software and hardware object*/
Soft_Hard_Vertion Vertion;
/*Create work parameters information object*/
Work_Parameter_Information WorkParameter_Info;

/*
 @brief     : 设置EEPROM读写引脚
              Set the R/W pin of EEPROM.
 @para      : None
 @return    : None
 */
void EEPROM_Operations::EEPROM_GPIO_Config(void)
{
    pinMode(EP_WP_PIN, OUTPUT);
    digitalWrite(EP_WP_PIN, HIGH);
	Serial.println("EEPROM读写引脚配置完成");
    I2C_Init();
}

/*
 @brief     : 上拉该引脚，禁止EEPROM写操作
              Pull up pin in order to disable EEPROM write.
 @para      : None
 @return    : None
 */
void EEPROM_Operations::EEPROM_Write_Disable(void)
{
    delay(5);
    digitalWrite(EP_WP_PIN, HIGH);
}

/*
 @brief     : 下拉该引脚，允许EEPROM写操作
              Pull up pin in order to disable EEPROM write.
 @para      : None
 @return    : None
 */
void EEPROM_Operations::EEPROM_Write_Enable(void)
{
    digitalWrite(EP_WP_PIN, LOW);
    delay(5);
}

/*
 @brief     : 写SN码到EEPROM
              Write SN code to EEPROM
 @para      : SN array
 @return    : true or false
 */
bool SN_Operations::Save_SN_Code(unsigned char *sn_code)
{
    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 9; i++)
        AT24CXX_WriteOneByte(SN_BASE_ADDR + i, sn_code[i]);

    unsigned char SN_Verify = GetCrc8(&sn_code[0], 9); // Calculate CRC8 for SN code.
    AT24CXX_WriteOneByte(SN_VERIFY_ADDR, SN_Verify);
    //Verify that SN is writing successfully.
    unsigned char SN_Code_Temp[9];
    for (unsigned char i = 0; i < 9; i++)
        SN_Code_Temp[i] = AT24CXX_ReadOneByte(SN_BASE_ADDR + i);

    unsigned char SN_Verify_Temp = GetCrc8(&SN_Code_Temp[0], 9);
    if (SN_Verify_Temp != SN_Verify){
        EEPROM_Write_Disable();
        return false;
    }else{
        AT24CXX_WriteOneByte(SN_OPERATION_FLAG_ADDR, 0x55); //Already saved, set flag.
        EEPROM_Write_Disable();
        return true;
    }
}

/*
 @brief     : 写SN码到EEPROM的SN备份地址
              Write backup SN code to EEPROM
 @para      : SN array
 @return    : true or false
 */
bool SN_Operations::Save_BKP_SN_Code(unsigned char *sn_code)
{
    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 9; i++)
        AT24CXX_WriteOneByte(SN_BKP_BASE_ADDR + i, sn_code[i]);

    unsigned char SN_BKP_Verify = GetCrc8(&sn_code[0], 9);  // Calculate CRC8 for SN code.
    AT24CXX_WriteOneByte(SN_BKP_VERIFY_ADDR, SN_BKP_Verify);
    //Verify that SN is writing successfully.
    unsigned char SN_Code_Temp[9];
    for (unsigned char i = 0; i < 9; i++)
        SN_Code_Temp[i] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + i);

    unsigned char SN_BKP_Verify_Temp = GetCrc8(&SN_Code_Temp[0], 9);
    if (SN_BKP_Verify_Temp != SN_BKP_Verify){
        EEPROM_Write_Disable();
        return false;
    }else{
        AT24CXX_WriteOneByte(SN_BKP_OPERATION_FLAG_ADDR, 0x55); ////Already saved, set flag.
        EEPROM_Write_Disable();
        return true;
    }
}

/*
 @brief     : 从EEPROM读取SN码，同时也是校验EEPROM中的SN码是否损坏。
              Read SN code from EEPROM,and check whether SN code in EEPROM is damaged.
 @para      : SN array
 @return    : true or false
 */
bool SN_Operations::Read_SN_Code(unsigned char *sn_code)
{
    unsigned char SN_Temp[9] = {0};

    for (unsigned char i = 0; i < 9; i++)
        SN_Temp[i] = AT24CXX_ReadOneByte(SN_BASE_ADDR + i);

    unsigned char SN_Verify = AT24CXX_ReadOneByte(SN_VERIFY_ADDR);
    unsigned char SN_Verify_Temp = GetCrc8(&SN_Temp[0], 9);
    //Verify SN's CRC8.
    if (SN_Verify != SN_Verify_Temp)
        return false;
    else{
        for (unsigned char i = 0; i < 9; i++)
            sn_code[i] = SN_Temp[i];
        return true;
    }
}

/*
 @brief     : 从EEPROM读取备份的SN码，同时也是校验EEPROM中的备份SN码是否损坏。
              Read backup SN code from EEPROM,and check whether SN code in EEPROM is damaged.
 @para      : SN array
 @return    : true or false
 */
bool SN_Operations::Read_BKP_SN_Code(unsigned char *sn_code)
{
    unsigned char SN_BKP_Temp[9] = {0};

    for (unsigned char i = 0; i < 9; i++)
        SN_BKP_Temp[i] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + i);

    unsigned char SN_BKP_Verify = AT24CXX_ReadOneByte(SN_BKP_VERIFY_ADDR);
    unsigned char SN_BKP_Verify_Temp = GetCrc8(&SN_BKP_Temp[0], 9);
    //Verify SN's CRC8
    if (SN_BKP_Verify != SN_BKP_Verify_Temp)
        return false;
    else{
        for (unsigned char i = 0; i < 9; i++)
            sn_code[i] = SN_BKP_Temp[i];
        return true;
    }
}

void SN_Operations::Read_Random_Seed(unsigned char *random_seed)
{
    unsigned char Random_Temp[2];
    Random_Temp[0] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + 7);
    Random_Temp[1] = AT24CXX_ReadOneByte(SN_BKP_BASE_ADDR + 8);
    *random_seed = GetCrc8(Random_Temp, 2);
}

/*
 @brief     : 验证是否已经写入SN码，0x55表示已经写入SN码
              Verify that the SN check code already written,0x55 indicates that the SN code has been written.
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Verify_Save_SN_Code(void)
{
    if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 @brief     : 验证是否已经写入备份SN码，0x55表示已经写入SN码
              Verify that the backup SN check code already written,0x55 indicates that the SN code has been written.
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Verify_Save_BKP_SN_Code(void)
{
    if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 @brief     : 清除保存SN码标志位，慎用！
              Clear the flag that has been written to SN, use with caution!
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Clear_SN_Save_Flag(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SN_OPERATION_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();

    if (AT24CXX_ReadOneByte(SN_OPERATION_FLAG_ADDR) == 0x00)
        return true;
    else
        return false;
}

/*
 @brief     : 清除保存备份SN码标志位，慎用！
              Clear the backup flag that has been written to SN, use with caution!
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Clear_BKP_SN_Save_Flag(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SN_BKP_OPERATION_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();

    if (AT24CXX_ReadOneByte(SN_BKP_OPERATION_FLAG_ADDR) == 0x00)
        return true;
    else
        return false;
}

/*
 @brief     : 如果注册服务器成功，设置注册成功标志位
              If the registered server is successful, set the flag of registration.
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Set_SN_Access_Network_Flag(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SN_ACCESS_NETWORK_FLAG_ADDR, 0X55);
    EEPROM_Write_Disable();
    if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

/*
 @brief     : 清除注册到服务器标志位，这将意味着本机将重新请求注册到服务器，慎用！
              Remove the registry to the server flag, which means that the device 
              will again request the server to register to the server, use with caution!
 @para      : None
 @return    : true or false
 */
bool SN_Operations::Clear_SN_Access_Network_Flag(void)
{
    if (digitalRead(K1) == LOW){
        delay(5000);
        if (digitalRead(K1) == LOW){
            EEPROM_Write_Enable();
            AT24CXX_WriteOneByte(SN_ACCESS_NETWORK_FLAG_ADDR, 0x00);
            EEPROM_Write_Disable();
            if (AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x00)
                return true;
            else
                return false;  
        }
    } 
}

/*
 @brief     : 验证是否已经注册到服务器，0x55表示已经注册。
              Verify that the server is registered,0x55 indicates has been registered.
 @para      : None
 @return    : true or false              
 */
bool SN_Operations::Verify_SN_Access_Network_Flag(void)
{
    bool Bool_Value;
    AT24CXX_ReadOneByte(SN_ACCESS_NETWORK_FLAG_ADDR) == 0x55 ? Bool_Value = true : Bool_Value = false;
    return Bool_Value;
}

/*
 @brief     : 验证读取的SN是否错误，如果其中一个SN失败，则用另一个完整的SN覆盖它。
              Verify that the read SN is wrong, and if one of the SN fails, override it with the other intact.
 @para      : SN array
 @return    : true or false
 */
bool SN_Operations::Self_Check(unsigned char *sn_code)
{
    bool Bool_Value;
    if (Read_SN_Code(sn_code) && Read_BKP_SN_Code(sn_code)){
        Serial.println("Read SN code and read backup SN code success...");
        Bool_Value = true;

    }else if (Read_SN_Code(sn_code)){
        Serial.println("Read SN code success but read backup SN code failed...");
        Save_BKP_SN_Code(sn_code) == true ? Bool_Value = true : Bool_Value = false;

    }else if (Read_BKP_SN_Code(sn_code)){
        Serial.println("Read backup SN code success but read SN code failed...");
        Save_SN_Code(sn_code) == true ? Bool_Value = true : Bool_Value = false;

    }else{ 
        Serial.println("All SN store ERROR!");
        Clear_SN_Save_Flag();
        Clear_BKP_SN_Save_Flag();
    }
    return Bool_Value;
}

void LoRa_Config::Save_LoRa_Config_Flag(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(LORA_PARA_CONFIG_FLAG_ADDR, 0x55);
    EEPROM_Write_Disable();
}

bool LoRa_Config::Verify_LoRa_Config_Flag(void)
{
    if (AT24CXX_ReadOneByte(LORA_PARA_CONFIG_FLAG_ADDR) == 0x55)
        return true;
    else
        return false;
}

void LoRa_Config::Clear_LoRa_Config_Flag(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(LORA_PARA_CONFIG_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();   
}

/*
 @brief     : 保存该设备的软件版本号
              Save device's software version.
 @para      : Version number high byte, version number low byte.
 @return    : None
 */
void Soft_Hard_Vertion::Save_Software_version(unsigned char number_high, unsigned char number_low)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(SOFT_VERSION_BASE_ADDR, number_high);
    AT24CXX_WriteOneByte(SOFT_VERSION_BASE_ADDR + 1,  number_low);
    EEPROM_Write_Disable();
}

unsigned char Soft_Hard_Vertion::Read_hardware_version(unsigned char number_addr)
{
	unsigned char version = AT24CXX_ReadOneByte(number_addr);
	return version;
}

unsigned char Soft_Hard_Vertion::Read_Software_version(unsigned char number_addr)
{
	unsigned char version = AT24CXX_ReadOneByte(number_addr);
	return version;
}

/*
 @brief     : 保存该设备的硬件版本号
              Save device's hardware version.
 @para      : Version number high byte, version number low byte.
 @return    : None
 */
void Soft_Hard_Vertion::Save_hardware_version(unsigned char number_high, unsigned char number_low)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(HARD_VERSION_BASE_ADDR, number_high);
	AT24CXX_WriteOneByte(HARD_VERSION_BASE_ADDR + 1, number_low);
    EEPROM_Write_Disable();
}

/*
 @brief     : 保存工作组号
              Save the work group number.
 @para      : group number(array)
 @return    : true or false
 */
bool Control_Information::Save_Group_Number(unsigned char *group_num)
{
    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 5; i++)
        AT24CXX_WriteOneByte(GROUP_NUMBER_BASE_ADDR + i, group_num[i]);

    unsigned char CRC8 = GetCrc8(&group_num[0], 5);
    AT24CXX_WriteOneByte(GROUP_NUMBER_VERIFY_ADDR, CRC8);

    unsigned char Group_Temp[5];
    for (unsigned char i = 0; i < 5; i++){
        if (AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i) != group_num[i])
            return false;
    }

    if (AT24CXX_ReadOneByte(GROUP_NUMBER_VERIFY_ADDR) == CRC8){
        AT24CXX_WriteOneByte(GROUP_NUMBER_FLAG_ADDR, 0x55);
        EEPROM_Write_Disable();
        return true;
    }else{
        EEPROM_Write_Disable();
        return false;
    }
}

/*
 @brief     : 读取工作组号
              Read the workgroup number.
 @para      : workgroup number(array)
 @return    : None
 */
void Control_Information::Read_Group_Number(unsigned char *group_num)
{
    for (unsigned char i = 0; i < 5; i++)
        group_num[i] = AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i);
}

/*
 @brief     : 验证保存的工作组号是否损坏
              Verify that the saved groupwork number is damaged.
 @para      : None
 @return    : true or false
*/
bool Control_Information::Check_Group_Number(void)
{
    unsigned char Group_Temp[5]; 
    for (unsigned char i = 0; i < 5; i++)
        Group_Temp[i] =AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i);
    unsigned char Verify_CRC8_Temp = GetCrc8(&(Group_Temp[0]), 5);
    if (Verify_CRC8_Temp == AT24CXX_ReadOneByte(GROUP_NUMBER_VERIFY_ADDR))
        return true;
    else
        return false;
}

/*
 @brief     : 验证是否已经保存过工作组号
              Verify that the workgroup number has been saved.
 @para      : None
 @return    : true or false
 */
bool Control_Information::Verify_Group_Number_Flag(void)
{
    bool Bool_Value;
    AT24CXX_ReadOneByte(GROUP_NUMBER_FLAG_ADDR) == 0X55 ? Bool_Value = true : Bool_Value = false;
    return Bool_Value;
}

/*
 @brief     : 清除本机的工作组号，慎用！
              Clear the workgroup number of the device, use with caution!
 @para      : None
 @return    : true or false              
 */
bool Control_Information::Clear_Group_Number(void)
{
    EEPROM_Write_Enable();
    for (unsigned char i = 0; i < 5; i++){
        AT24CXX_WriteOneByte(GROUP_NUMBER_BASE_ADDR + i, 0x00);
        if (AT24CXX_ReadOneByte(GROUP_NUMBER_BASE_ADDR + i) != 0x00){
            EEPROM_Write_Disable();
            return false;
        }
    }
    AT24CXX_WriteOneByte(GROUP_NUMBER_FLAG_ADDR, 0x00);
    EEPROM_Write_Disable();
    return true;
}

/*
 @brief     : 保存区域号
              Save the area number.
 @para      : area number
 @return    : true or false              
 */
bool Control_Information::Save_Area_Number(unsigned char area_num)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(AREA_ADDR, area_num);
    AT24CXX_WriteOneByte(AREA_VERIFY_ADDR, GetCrc8(&area_num, 1));

    if (AT24CXX_ReadOneByte(AREA_ADDR) == area_num){
        AT24CXX_WriteOneByte(AREA_FLAG_ADDR, 0x55);
        EEPROM_Write_Disable();
        return true;
    }else{
        EEPROM_Write_Disable();
        return false;
    }
}

/*
 @brief     : 读取区域号
              Read the area number.
 @para      : None
 @return    : Area number.
 */
unsigned char Control_Information::Read_Area_Number(void)
{
    return (AT24CXX_ReadOneByte(AREA_ADDR));
}

/*
 @brief     : 验证保存的区域号是否损坏
              Verify that the saved area number is damaged.
 @para      : None
 @return    : true or false
*/
bool Control_Information::Check_Area_Number(void)
{
    unsigned char Area_Temp = AT24CXX_ReadOneByte(AREA_ADDR);
    unsigned char Verify_Temp = GetCrc8(&Area_Temp, 1);
    if (Verify_Temp == AT24CXX_ReadOneByte(AREA_VERIFY_ADDR))
        return true;
    else
        return false;
}

/*
 @brief     : 验证区域号是否已经保存
              Verify that area number has been saved.
 @para      : None
 @return    : true or false
 */
bool Control_Information::Verify_Area_Number_Flag(void)
{
    bool Bool_Value;
    AT24CXX_ReadOneByte(AREA_FLAG_ADDR) == 0x55 ? Bool_Value = true : Bool_Value = false;
    return Bool_Value;
}

/*
 @brief     : 清除区域号，慎用！
              Clear the area number, use wht caution!
 @para      : None
 @return    : true or fasle
 */
bool Control_Information::Clear_Area_Number(void)
{
    EEPROM_Write_Enable();
    AT24CXX_WriteOneByte(AREA_ADDR, 0X00);  //Default area number 
    EEPROM_Write_Disable();
}

/*
 @brief     : 保存卷膜工作电压阈值和状态上报间隔
              Saved the roll working voltage threshold and interval of report status.
 @para      : voltage threshold value, interval of reprot status(array)
 @return    : true or false                       
 */
bool Control_Information::Save_Roll_Work_Voltage_and_Report_Interval(unsigned char *threshold_value)
{
    // EEPROM_Write_Enable();
    // for (unsigned char i = 0; i < 5; i++){
    //     AT24CXX_WriteOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + i, threshold_value[i]);
    //     if (AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_END_ADDR + i) != threshold_value[i]){
    //         EEPROM_Write_Disable();
    //         return false;
    //     }
    // }
    // EEPROM_Write_Disable();
    // return true;
}

/*
 @brief     : 读取最小电压阈值
              Read the minimum voltage threshold.
 @para      : None
 @return    : voltage value.
 */
unsigned int Control_Information::Read_Roll_Low_Voltage_Limit_Value(void)
{
    //return (AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR) << 8 | AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + 1));
}

/*
 @brief     : 读取最大电压阈值
              Read the maximum voltage threshold.
 @para      : None
 @return    : voltage value.
*/
unsigned int Control_Information::Read_Roll_High_Voltage_Limit_Value(void)
{
    //return (AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + 2) << 8 | AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + 3));
}

/*
 @brief     : 读取上报实时状态间隔值
              Read report interval value of realtime status.
 @para      : None
 @return    : interval value.
 */
unsigned char Control_Information::Read_Roll_Report_Status_Interval_Value(void)
{
    //return (AT24CXX_ReadOneByte(ROLL_THRESHOLD_VALUE_BASE_ADDR + 4));    
}

/*
 *brief     : Save Received collect time from gateway to EEPROM. If it is the same as that already stored, it is not saved
 保存接收到的从网关到EEPROM的收集时间。如果它与已存储的相同，则不保存它
 *para      : collect time (S)
 *return    : true or false
 */
bool Work_Parameter_Information::Save_Collect_Time(unsigned int time)
{
    unsigned int Time = time;
    unsigned int Time_Temp;

    if (Time > 43200 || Time < 5)  //max: 12 hours; min: 15seconds.
        return false;

    Time_Temp = AT24CXX_ReadOneByte(COLLECT_TIME_HIGH_ADDR) << 8 | AT24CXX_ReadOneByte(COLLECT_TIME_LOW_ADDR);

    if (Time_Temp == Time){
        Serial.println("collect data already stored...");
        return true;
    }else{
        EEPROM_Write_Enable();        
        AT24CXX_WriteOneByte(COLLECT_TIME_HIGH_ADDR, Time >> 8);
        AT24CXX_WriteOneByte(COLLECT_TIME_LOW_ADDR, Time & 0xFF);
        AT24CXX_WriteOneByte(COLLECT_TIME_FLAG_ADDR, 0x55);
        EEPROM_Write_Disable();
        return true;
    }
}

/*
 *brief     : Read collect time from EEPROM阅读从EEPROM收集时间
 *para      : None
 *return    : collect time (S)
 */
unsigned int Work_Parameter_Information::Read_Collect_Time(void)
{
    return ((AT24CXX_ReadOneByte(COLLECT_TIME_HIGH_ADDR) << 8) | (AT24CXX_ReadOneByte(COLLECT_TIME_LOW_ADDR)));
}

/*
 *brief     : Verify that the collect time has been stored.验证收集时间已被存储
 *para      : None.
 *return    : true or false.
 */
bool Work_Parameter_Information::Verify_Collect_Time_Flag(void)
{
    if (AT24CXX_ReadOneByte(COLLECT_TIME_FLAG_ADDR) == 0x55)//收集时间标志地址
        return true;
    else
        return false;
}


