#ifndef _RECEIPT_H
#define _RECEIPT_H

#include <Arduino.h>

#define DEVICE_TYPE_ID  0x0001

enum ReceiptStatus{
  FactoryMode = 0, AskUploadParamsOk, AskUploadParamsErr, AssignGroupIdArrayOk, AssignGroupIdArrayErr, SetSnAndSlaverCountOk, 
  SetSnAndSlaverCountErr, SetWorkParaOK, SetWorkParaErr
};

enum MotorStatus{
  MotorFactoryMode = 0, ROLL_OK, HIGH_POSITION_LIMIT_EXCEPTION, LOW_POSITION_LIMIT_EXCEPTION, LOW_POWER, MOTOR_EXCEPTION, MOTOR_CURRENT_EXCEPTION, 
  ROLLING, CMD_EXCEPTION, NOT_INITIALIZED, STORE_EXCEPTION, RESET_ROLLING, RESET_ROLLOK, RS485_EXCEPTION, FORCE_STOP
};

extern unsigned char g_Motor_Status;

/*
 @brief   : 设置当前设备工作状态
            Sets the current working state of the device.
 @para    : status ---> current status.
 @return  : None
*/
inline void Set_Motor_Status(unsigned char status) {g_Motor_Status = status;}

/*
 @brief   : 读取当前设备工作状态
            Read the current working state of the device.
 @para    : None
 @return  : Current working status.        
*/
inline unsigned char Read_Motor_Status(void) {return (g_Motor_Status);}

class Receipt{
public:
    void Report_General_Parameter(void);
    void Request_Set_Group_Number(void);
    void Request_Device_SN_and_Channel(void);
    void Working_Parameter_Receipt(void);
    void General_Receipt(unsigned char status, unsigned char send_times);
    void Send_Sensor_Data(void);
private:
  void Receipt_Random_Wait_Value(unsigned long int *random_value);
  void Clear_Server_LoRa_Buffer(void);
  void Print_Debug(unsigned char *base_addr, unsigned char len);
};

extern Receipt Message_Receipt;

#endif