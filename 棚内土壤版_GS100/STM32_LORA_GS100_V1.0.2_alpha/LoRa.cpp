/************************************************************************************
 * Code and comments : KeeganLu
 * Date：2019/3/18
 * 
 * This document is only for the LoRa wireless module of the MHL9LF model of renyu 
 * technology and may not be applicable to other models or LoRa modules of other 
 * manufacturers.The main functions of this file include configuring pin mode of MHL9LF 
 * model LoRa module, reading various LoRa parameter information through AT instruction, 
 * configuring various LoRa parameters through AT instruction, etc.For this LoRa module, 
 * it should be noted that every time the AT instruction is sent, it should pay attention 
 * to whether the returned data is an ERROR flag. If so, it is recommended to do some 
 * corresponding exception processing.For more information about LoRa wireless module of 
 * MHL9LF model, please refer to renyu's technical documentation.
 * A common interface for each class is provided in the header file.
 * 
 * If you have any questions, please send an email to me： idlukeqing@163.com
*************************************************************************************/

/************************************************************************************
 * 
 * 代码与注释：卢科青
 * 日期：2019/3/18
 * 
 * 该文件仅仅是针对仁钰科技的MHL9LF型号的LoRa无线模块所编写，不一定适用于其他型号，或其他厂商
 * 的LoRa模块。该文件主要功能有配置MHL9LF型号LoRa模块的引脚模式、通过AT指令读取各类LoRa参数
 * 信息、通过AT指令配置各类LoRa参数等。对于该LoRa模块，需要注意的是，每一次发送的AT指令都要
 * 留意返回的数据是否是ERROR标志，如果是，建议做一些相应的异常处理。
 * 更多MHL9LF型号的LoRa无线模块信息，请参考仁钰公司的技术文档。
 * 头文件中提供了各个类的公共接口。
 * 
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "LoRa.h"
#include "BCD_CON.h"
#include <libmaple/iwdg.h>
#include "public.h"
#include "fun_periph.h"

/*Create LoRa object*/
LoRa LoRa_MHL9LF;

/*
 @brief     : 配置LoRa模块相关引脚
              Configurate the LoRa model related pins.
 @para      : None
 @return    : None
 */
void LoRa::LoRa_GPIO_Config(void)
{
    pinMode(LORA_PWR_PIN, OUTPUT);
    pinMode(AT_CMD_PIN, OUTPUT);
    pinMode(WAKEUP_PIN, OUTPUT);
    pinMode(RESET_PIN, OUTPUT);
    LORA_PWR_ON;
    delay(100);
    digitalWrite(WAKEUP_PIN, LOW);
    digitalWrite(RESET_PIN, HIGH);
}

/*
 @brief     : 配置LoRa串口波特率
              Configurate LoRa serial prot related baud rate.
 @para      : baudrate(such as 4800, 9600, 115200)
 @return    : None
 */
void LoRa::BaudRate(unsigned int baudrate)
{
    LoRa_Serial.begin(baudrate);
}

/*
 @brief     : 配置LoRa模式。
              Configurate LoRa's mode.
 @para      : AT_status : The high level is AT mode, and low level is passthrough mode.
 @return    : None
 */
void LoRa::Mode(LoRa_Mode AT_status)
{
    AT_status == AT ? digitalWrite(AT_CMD_PIN, HIGH) : digitalWrite(AT_CMD_PIN, LOW);
    delay(100);
}

/*
 @brief     : 决定LoRa是否重启
              Determine whether LoRa is reset.
 @para      : Is_reset  : Reset to a low level not less than 100ms. Default to high level.
 @return    : None
 */
void LoRa::IsReset(bool Is_reset)
{
    if (Is_reset == true){
        digitalWrite(RESET_PIN, LOW);
        delay(150); //150ms
        digitalWrite(RESET_PIN, HIGH);
    }
}

/*
 @brief     : 将LoRa模块完全断电
 @param     : 无
 @return    : 无
 */
void LoRa::LoRa_Shutdown(void)
{   
    LORA_PWR_OFF;
    /*注意！，如果下面这个end()已经执行过一次，并且中途没有重启打开begin，第二次end()，会死机！*/
    LoRa_Serial.end();
    pinMode(PA2, OUTPUT); //TX
    pinMode(PA3, OUTPUT); //RX
    digitalWrite(PA2, LOW);
    digitalWrite(PA3, LOW);
    digitalWrite(RESET_PIN, LOW);    
}

/*
 @brief     : 将完全断电的LoRa模块重启
 @param     : 无
 @return    : 无
 */
void LoRa::LoRa_Restart(void)
{
    LORA_PWR_ON;
    BaudRate(9600);
    //digitalWrite(RESET_PIN, HIGH);
    IsReset(true);
    Mode(PASS_THROUGH_MODE);
}

/*
 @brief     : 检测是否收到错误回执
              Detects if an error receipt has been received.
 @para      : verify_data(array)
              data_buffer(array).get error receipt type.
 @return    : error type or no error.
 */
unsigned char LoRa::Detect_Error_Receipt(unsigned char *verify_data)
{
    unsigned char Error;
    if (verify_data[0] == 'E' && verify_data[1] == 'R'){
        Error = (verify_data[2] - '0') * 10 + (verify_data[3] - '0');
        Serial.println(Error);
        return Error;
    }else{
        return No_Err;
    }
}

/*
 @brief     : 用于设置LoRa参数后，如果设置参数成功，接收到回执“OK”
              After setting the LoRa parameter, if the parameter is set successfully,
              the receipt is received with "OK".
 @para      : verify_data(array)
              data_buffer(array).get string "OK"    
 @return    : true or false                    
 */
bool LoRa::Detect_OK_Receipt(unsigned char *verify_data, unsigned char *data_buffer)
{
    if (verify_data[0] == 'O' && verify_data[1] == 'K'){
        data_buffer[0] = verify_data[0];
        data_buffer[1] = verify_data[1];
        return true;
    }else{
        return false;
    }  
}

/*
 @brief     : LoRa模块核心函数，用来发送AT指令给LoRa模块，同时也包含了判断是否接到查询数据、或设置OK，
              或设置失败等信息。
              该函数用来查询参数
              The core function of LoRa module, which is used to send AT commands to LoRa module,
              also contains information such as whether to receive query data, or set OK, or
              set failure.
              The function is use to inquire parameters.
 @para      : cmd ---> AT commands
              data_buffer ---> received data
              data_len ---> received data length
 @return    : received data type (bytes, error, OK)
 */
Receive_Type LoRa::AT_Query_Cmd(char *cmd, unsigned char *data_buffer, unsigned char *data_len = 0)
{
    unsigned char Receive_Len = 0, Copy_Len = 0;
    unsigned char addr_temp[100] = {0};
    bool Is_Valid = false;

    LoRa_Serial.print(cmd);
    delay(100);
    while (LoRa_Serial.available() > 0){
        if (Receive_Len >= 100) return Invalid;
        addr_temp[Receive_Len++] = LoRa_Serial.read();
    }
    Receive_Len > 0 ? Is_Valid = true : Is_Valid = false;

    if (Receive_Len == 0)
        Serial.println("No receive data !!!");

    if (Is_Valid == true){
        if ((addr_temp[0] == '\r' && addr_temp[1] == '\n') && (addr_temp[Receive_Len - 1] == '\n' && addr_temp[Receive_Len - 2] == '\r')){
           for (unsigned char i = 2; i < Receive_Len - 2; i++)
               addr_temp[Copy_Len++] =  addr_temp[i];

           if (Detect_Error_Receipt(&addr_temp[0]) != No_Err) return ERROR;
           if(Parse_Command(addr_temp, Copy_Len, data_buffer, &data_len) == true)
               return Bytes;
            else
                return Invalid;
        }else
            return Invalid;
    }else
        return Invalid;
}

/*
 @brief     : LoRa模块核心函数，用来发送AT指令给LoRa模块，同时也包含了判断是否接到查询数据、或设置OK，
              或设置失败等信息。
              该函数用来设置参数
              The core function of LoRa module, which is used to send AT commands to LoRa module,
              also contains information such as whether to receive query data, or set OK, or
              set failure.
              The function is use to set parameter.
 @para      : cmd ---> AT commands
              para ---> Set LoRa parameters value.
              data_buffer ---> received data
 @return    : received data type (bytes, error, OK)
 */
Receive_Type LoRa::AT_Config_Cmd(char *cmd, char * para, unsigned char *data_buffer)
{
    unsigned char Receive_Len = 0, Copy_Len = 0;
    unsigned char addr_temp[100] = {0};
    char AT_cmd[100] = {0};
    unsigned char i = 0, j = 0;
    bool Is_Valid = false;

    for (; cmd[i] != '\0'; i++)
        AT_cmd[i] = cmd[i];

    for (; para[j] != '\0'; j++)
        AT_cmd[i++] = para[j];

    AT_cmd[i++] = '\r';
    AT_cmd[i++] = '\n';

    Serial.write(AT_cmd);

    LoRa_Serial.write(AT_cmd, i);
    delay(100);
    while (LoRa_Serial.available() > 0){
        if (Receive_Len >= 100) return Invalid;
        addr_temp[Receive_Len++] = LoRa_Serial.read();
    }
    Receive_Len > 0 ? Is_Valid = true : Is_Valid = false;

    if (Is_Valid == true){
        if ((addr_temp[0] == '\r' && addr_temp[1] == '\n') && (addr_temp[Receive_Len - 1] == '\n' && addr_temp[Receive_Len - 2] == '\r'))
        {
           for (unsigned char i = 2; i < Receive_Len - 2; i++){
               addr_temp[Copy_Len++] =  addr_temp[i];
           }
           if (Detect_Error_Receipt(&addr_temp[0]) != No_Err) return ERROR;
           if (Detect_OK_Receipt(&addr_temp[0], data_buffer) == true) return OK;
        }else
            return Invalid;
    }else
        return Invalid;
}

/*
 @brief     : 在AT指令查询参数后，用于判断返回的参数是何种参数，然后传递给对应的函数解析获取参数
              After AT commands queries the paramters, it is used to determine what kind of parameters are returnd,
              and then it is passed to the corresponding function to parse and get the parameters.
 @para      : addr_temp ---> received receipt.
              len ---> This receipt length
              data_buffer ---> received data
              data_len ---> received data length
 @return    : true or false
 */
bool LoRa::Parse_Command(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer, unsigned char **data_len)
{
    unsigned char Which_Cmd;
    unsigned char i = 0, j = 0;

    if (addr_temp[1] == 'C' && addr_temp[2] == 'S' && addr_temp[3] == 'Q'){
        Which_Cmd = CSQ;
    }else
        Which_Cmd = CMOMON;

    while (addr_temp[i] != ':')
        i++;
    i++;
    for (; i <= len; i++)
        addr_temp[j++] = addr_temp[i];
    
    switch (Which_Cmd){
        case CMOMON : **data_len = (j -1) / 2; Get_Bytes(addr_temp, j - 1, data_buffer); break;
        case CSQ    : **data_len = 2; Get_CSQ(addr_temp, j - 1, data_buffer); break;
    }
    return true;
}

/*
 @brief     : 得到一串的16进制参数，如单播地址，组播地址等。
              Get a string of Hex parameters, such as unicast address, multicast address, etc.
 @para      : addr_temp ---> received receipt
              len ---> received receipt length
              data_buffer ---> received hex bytes.
 @return    : None
 */
void LoRa::Get_Bytes(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer)
{
    bool Single_Hex_Flag = false;
    String_to_Hex(&addr_temp[0], len);
    for (unsigned char i = 0, j = 0; i <= len; i++){
        if (i % 2 == 0){
            data_buffer[j] = addr_temp[i] * 10;
            data_buffer[j] == 0 ? Single_Hex_Flag = true : Single_Hex_Flag = false;
        }else{
            data_buffer[j++] += addr_temp[i];
            if (Single_Hex_Flag == false)
               Type_Conv.Hex_To_Dec(&data_buffer[j - 1]);
        }
    }
}

/*
 @brief     : 得到信噪比和接收信号强度参数。
              Get SNR and RSSI intensity parameters.
 @para      : addr_temp ---> received receipt
              len ---> received receipt length
              data_buffer --> received hex bytes
 @return    : None
 */
void LoRa::Get_CSQ(unsigned char *addr_temp, unsigned char len, unsigned char *data_buffer)
{
    unsigned char i = 0, j = 0;
    for (; addr_temp[i] != ','; i++)
        data_buffer[i] = addr_temp[i];

    data_buffer[i++] = 0x55;

    for (j = i; j < len; j++)
        data_buffer[j] = addr_temp[j];
}

/*
 @brief     : LoRa返回的回执信息是ASCLL码，将ASCLL码转换成16进制。
              The return receipt information from LoRa is ASCLL code,
              which is converted into hex code.
 @para      : str ---> ASCLL code
              len ---> ASCLL code length
 @return    : true or false
 */
bool LoRa::String_to_Hex(unsigned char *str, unsigned char len)
{
    for (unsigned char i = 0; i < len; i++){
        if (str[i] >= '0' && str[i] <= '9'){
            str[i] -= '0';

        }else if (str[i] >= 'A' && str[i] <= 'F'){
            str[i] -= '7';

        }else if (str[i] >= 'a' && str[i] <= 'f'){
            str[i] -= 'W';
            
        }else{
            return false;
        }
    }
    return true;
}

/*
 @brief     : 发送AT指令接口。可以通过该函数设置LoRa模块，或是查询设置信息。
              Send AT instruction interface.
              This function can be used to set up the LoRa module, or 
              to query the settings information.
 @para      : data_buffer ---> Receive LoRa information.
              is_query ---> Query instruciton or set instruction.
              *cmd ---> AT instruction.
              *para ---> AT parameter.
 @return    : true or false
 */
bool LoRa::LoRa_AT(unsigned char *data_buffer, bool is_query, char *cmd, char *para)
{
  unsigned char Receive_length = 0;
  LoRa_MHL9LF.Mode(AT);
  unsigned char Return_Type;

  if (is_query == true)
      Return_Type = LoRa_MHL9LF.AT_Query_Cmd(cmd, data_buffer, &Receive_length);
  else
      Return_Type = LoRa_MHL9LF.AT_Config_Cmd(cmd, para, data_buffer);

  switch (Return_Type){
    case OK     : Serial.println("Set para OK");  LoRa_MHL9LF.Mode(PASS_THROUGH_MODE); return true; break;
    case ERROR  : Serial.println("Receipt ERROR..."); LoRa_MHL9LF.Mode(PASS_THROUGH_MODE); return false; break;
    case Bytes  : 
                for (unsigned char i = 0; i < Receive_length; i++){
                Serial.print(data_buffer[i], HEX);
                Serial.print(" ");
                }
                Serial.println();
                LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
                return true; 
                break;

    case Invalid : LoRa_MHL9LF.Mode(PASS_THROUGH_MODE); return false; break;

    default     : LoRa_MHL9LF.Mode(PASS_THROUGH_MODE); return false; break;
  }
  iwdg_feed();
}

bool LoRa::Rewrite_ID(void)
{
    unsigned char RcvBuffer[4];
    char WriteAddr[9];
    unsigned char i, j = 0;

    /*读取LoRa通信地址*/
    LoRa_AT(RcvBuffer, true, AT_ADDR_, 0);

    /*从DEC转换成HEX*/
    for (unsigned char i = 0; i < 8; i++)
        i % 2 == 0 ? WriteAddr[i] = RcvBuffer[j] / 16 : WriteAddr[i] = RcvBuffer[j++] % 16;

    for (unsigned char i = 0; i < 8; i++)
    {
        if (WriteAddr[i] >= 0 && WriteAddr[i] <= 9)
            WriteAddr[i] += '0';
        else if (WriteAddr[i] >= 10 && WriteAddr[i] <= 15)
            WriteAddr[i] += '7';
    }
    WriteAddr[8] = '\0';

    /*写入读出来的地址*/
    if(!LoRa_AT(RcvBuffer, false, AT_ADDR, WriteAddr))
        return false;
    
    return true;
}

/*
 @brief     : 初始化LoRa配置。如果没有配置，无法与网关通信。
              Initialize the LoRa configuration, you cannot
              communicate with the gateway without configuration.
 @para      : None
 @return    : None
 */
void LoRa::Parameter_Init(void)
{
    Serial.println("Configurate LoRa parameters...");
    unsigned char RcvBuffer[10];
    unsigned char StatusBuffer[15] = {0};
    bool SetStatusFlag;
    unsigned char i = 0, j;
    unsigned char TryNum = 0;

    do{
        i = 0;
        SetStatusFlag = true;
        StatusBuffer[i++] = Rewrite_ID();
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_TFREQ, "1C578DE0");
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_RFREQ, "1C03AE80");     
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_RIQ, "00"); 
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_NET, "01"); 
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_TSF, "09"); 
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_RSF, "09"); 
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_SIP, "01"); 
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_BW, "07"); 
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_POW, "14");
        StatusBuffer[i++] = LoRa_AT(RcvBuffer, false, AT_TIQ, "00");   

        for (j = 0; j < i; j++)
        {
            if (StatusBuffer[j] == 0)
            {
                Serial.println("LoRa parameters set ERROR! <Parameter_Init>");
                SetStatusFlag = false;
                TryNum++;
                LoRa_Restart();
                LoRa_Shutdown();
                delay(2000);
                break;
            }
        }
        LoRa_Restart();

    }while (!SetStatusFlag && TryNum < 10);

    /*
      *如果连续尝试100次配置LoRa参数都失败，说明LoRa模块已损坏。
      *直接进入死循环，同时LoRa参数错误灯状态闪烁，等待维修或更换设备！
     */
    if (!SetStatusFlag)
    {
        LED_SET_LORA_PARA_ERROR;
        while (1)
        {
            Serial.println("Set LoRa paramter ERROR, please check the LoRa module <Parameter_Init>");
            delay(3000);      
        }
    }
}
