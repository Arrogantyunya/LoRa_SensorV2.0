#ifndef _COMMAND_ANALYSIS_H
#define _COMMAND_ANALYSIS_H

#include <Arduino.h>

enum Frame_ID{
  Work_Para, Set_Group_Num, SN_Area_Channel, Work_Status,Work_Limit,Frame_Error	
};

extern bool g_Access_Network_Flag;  //Verify that the device is already registered.
extern bool g_Stop_Work_Flag;       //true: Force to stop reset roll or opening roll.
extern bool g_Mass_Command_Flag;
extern bool g_Get_Para_Flag; //waiting to receive acquisition parameter.

class Command_Analysis{
public:
  void Receive_LoRa_Cmd(void);

private:
  Frame_ID FrameID_Analysis(void);
  bool Verify_CRC8(unsigned char verify_data_base_addr, unsigned char verify_data_len);
  bool Verify_Device_Type_ID(void);
  void Verify_Mass_Commands(void);
  bool Verify_Area_Number(void);
  bool Verify_Work_Group(void);
  bool Verify_Frame_Validity(unsigned char verify_data_base_addr, unsigned char verify_data_len, bool area_flag, bool group_flag);
  
private:
  void Receive_Data_Analysis(void);
  void Query_Current_Work_Para(void);
  void Set_Group_Number(void);
  void Set_SN_Area_Channel(void);
  void Detailed_Work_Status(void);
  void ResetRoll_Command(void);
  void Opening_Command(void);
  void Working_Limit_Command(void);  
  void Stop_Work_Command(void);
};

/*Create command analysis project*/
extern Command_Analysis LoRa_Command_Analysis;

#endif