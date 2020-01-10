/************************************************************************************
 * Code and comments : KeeganLu
 * Date：2019/3/17
 *
 * The purpose of this file is to receive the instruction data sent by the server
 * through the LoRa wireless module, and then parse these instructions.Instructions
 * include general instructions (binding SN, setting area number, setting working
 * group number, inquiring the state of the machine, etc.) and private instructions
 * (resetting roll of film, setting opening roll film  and setting film working
 * threshold, etc.).Received instructions to verify CRC8, some instructions to
 * verify the area number or work group number the machine will perform the corresponding
 * function.
 * A common interface for each class is provided in the header file.
 *
 * If you have any questions, please send an email to me： idlukeqing@163.com
*************************************************************************************/

/************************************************************************************
 *
 * 代码与注释：卢科青
 * 日期：2019/3/17
 *
 * 该文件的作用是接收服务器通过LoRa无线模块发送过来的指令数据，然后解析这些指令。指令包括通用指令
 * （绑定SN，设置区域号，设置工作组号，查询本机状态等），私有指令（卷膜机重置行程，设置开度卷膜
 * 设置卷膜工作阈值等）。接收的指令都要校验CRC8，有些指令要校验区域号或工作组号本机才会执行相应
 * 功能。
 * 头文件中提供了各个类的公共接口。
 *
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "Command_Analysis.h"
#include "LoRa.h"
#include "User_CRC8.h"
#include "Memory.h"
#include "receipt.h"
#include "Private_RTC.h"
#include <libmaple/iwdg.h>

/*Create command analysis project*/
Command_Analysis LoRa_Command_Analysis;

/*Serial port related variable*/
static unsigned char g_Receive_cmd[128]; //Receive USARTx data.
unsigned char g_Receive_Length; //Received data length.
bool g_Access_Network_Flag = true; //Register to server flag.
bool g_Mass_Command_Flag = false;
bool g_Get_Para_Flag = false; //waiting to receive acquisition parameter.

/*
 @brief   : 从网关接收LoRa数据（网关 ---> 本机），接受的指令有通用指令和本设备私有指令。
			每条指令以0xFE为帧头，0x0D 0x0A 0x0D 0x0A 0x0D 0x0A，6个字节为帧尾。最大接受指令长度为128字节，超过将清空接收长度。
			LoRa data is received from the gateway, and commands received include general commands and private commands
			of the device. Each command starts with 0xFE as the frame head, 0x0D 0x0A 0x0D 0x0A 0x0D 0x0A, and 6 bytes as
			the frame tail. The maximum receiving command length is 128 bytes, which will clear the receiving length.
 @para    : None
 @return  : None
 */
void Command_Analysis::Receive_LoRa_Cmd(void)
{
	unsigned char End_num = 0;  //Frame tail number.
	bool End_num_flag = false;  //Number of endings received complete flag bits.
	bool Receive_end_flag = false;  //Correct flag at end of frame.
	g_Receive_Length = 0;

	iwdg_feed();

	while (LoRa_Serial.available() > 0) {
		iwdg_feed();
		g_Receive_cmd[g_Receive_Length++] = LoRa_Serial.read();
		delay(5);
		Serial.print(g_Receive_cmd[g_Receive_Length - 1], HEX);
		Serial.print(" ");

		if (g_Receive_Length >= 128)
			g_Receive_Length = 0;
		//Verify frame end: 0D 0A 0D 0A 0D 0A
		if (End_num_flag == false) {
			if (g_Receive_cmd[g_Receive_Length - 1] == 0x0D)
				End_num_flag = true;
		}
		if (End_num_flag == true) {
			switch (End_num) {
			case 0: g_Receive_cmd[g_Receive_Length - 1] == 0x0A ? End_num += 1 : End_num = 0; break;
			case 1: g_Receive_cmd[g_Receive_Length - 1] == 0x0D ? End_num += 1 : End_num = 0; break;
			case 2: g_Receive_cmd[g_Receive_Length - 1] == 0x0A ? End_num += 1 : End_num = 0; break;
			case 3: g_Receive_cmd[g_Receive_Length - 1] == 0x0D ? End_num += 1 : End_num = 0; break;
			case 4: g_Receive_cmd[g_Receive_Length - 1] == 0x0A ? End_num += 1 : End_num = 0; break;
			}
		}
		if (End_num == 5) {
			End_num = 0;
			End_num_flag = false;
			Receive_end_flag = true;
			Serial.println("Get frame end... <Receive_LoRa_Cmd>");
		}
	}
	if (Receive_end_flag == true) 
	{
		Serial.println("Parsing LoRa command... <Receive_LoRa_Cmd>");
		Receive_end_flag = false;
		Receive_Data_Analysis();//接收数据分析
		g_Receive_Length = 0;
	}
	else
	{
		g_Receive_Length = 0;
	}
}

/*
 @brief     : 根据验证接收的帧ID，决定返回对应的指令枚举
			  Based on the received frame ID verification, the decision to return the corresponding command enumeration.
 @para      : None
 @return    : frame id type(enum)
 */
Frame_ID Command_Analysis::FrameID_Analysis(void)
{
	unsigned int frame_id = ((g_Receive_cmd[1] << 8) | g_Receive_cmd[2]);
	iwdg_feed();
	switch (frame_id) {
	case 0xA011: return Work_Para;       break;
	case 0xA012: return Set_Group_Num;   break;
	case 0xA013: return SN_Area_Channel; break;
	case 0xA014: return Work_Status;     break;
	case 0xA022: return Work_Limit;      break;
	}
}

/*
 @brief     : 验证接收到的LoRa数据里的CRC8校验码
			  Verify CRC8 from received LoRa commomd.
 @para      : base verify_data_base_addr : Verify CRC8 data base address.
			  verify_data_len : Verify CRC8 data length.
 @return    : true or false.
 */
bool Command_Analysis::Verify_CRC8(unsigned char verify_data_base_addr, unsigned char verify_data_len)
{
	unsigned char Receive_CRC8 = GetCrc8(&g_Receive_cmd[verify_data_base_addr], verify_data_len);
	iwdg_feed();
	if (Receive_CRC8 == g_Receive_cmd[g_Receive_Length - 7])
		return true;
	else
		return false;
}

/*
 @brief   : 验证接收的设备ID与本机是否相同
			Verify that the received device ID is the same as the divice.
 @para    : None
 @return  : true or false
 */
bool Command_Analysis::Verify_Device_Type_ID(void)
{
	unsigned int Device_Type_ID = ((g_Receive_cmd[4] << 8) | g_Receive_cmd[5]);
	iwdg_feed();
	if (Device_Type_ID == DEVICE_TYPE_ID)
		return true;
	else
		return false;
}

/*
 @brief   : 验证接收的指令是否是群发指令
			Verify that the received commands is the mass commands.
 @para    : None
 @return  : None
 */
void Command_Analysis::Verify_Mass_Commands(void)
{
	g_Receive_cmd[6] == 0x55 ? g_Mass_Command_Flag = true : g_Mass_Command_Flag = false;
}

/*
 @brief   : 验证接收的区域号与本地是否相同
			Verify that received area number is the same the local area number.
 @para    : None
 @return  : true or false
 */
bool Command_Analysis::Verify_Area_Number(void)
{
	iwdg_feed();
	if (g_Receive_cmd[7] == 0x55) return true;  //Group control instructions.

	unsigned char Local_Area_Number = SN.Read_Area_Number();
	if (g_Receive_cmd[7] == Local_Area_Number || Local_Area_Number == 0)
		return true;
	else
		return false;
}

/*
 @brief   : 验证接收的工作组号是否在本机组控列表内。
			如果接收的组号是0x55，表明此指令忽略组控，发送给区域内所有的设备
			如果本设备还未申请注册过服务器，不用校验组号。
			Verify that the received workgroup number is in the unit control list.
			If the group number is 0x55, this command ignores the group control and is send to all devices in the area.
			If the device has not applied to register the server, do not check the group number.
 @para    : None
 @return  : true or false
 */
bool Command_Analysis::Verify_Work_Group(void)
{
	iwdg_feed();
	if (g_Receive_cmd[8] == 0x55) return true;  //Group control instructions.

	unsigned char Local_Group_Number[5], Receive_Group_Single_Number = g_Receive_cmd[8];
	unsigned char Undefined_Group_Num = 0;
	SN.Read_Group_Number(&Local_Group_Number[0]);

	for (unsigned char i = 0; i < 5; i++) {
		if (Receive_Group_Single_Number == Local_Group_Number[i]) return true;
		//Uninitialized group number.
		if (Local_Group_Number[i] == 0x00) {
			Undefined_Group_Num++;
			if (Undefined_Group_Num == 5)
				return true;
		}
	}
	return false;
}

/*
 @brief   : 验证接收的指令CRC8校验、设备类型码、区域号、工作组是否合法。
			可以通过形参决定是否屏蔽验证区域号和工作组。
 @para    : verify_data_base_addr : Verify CRC8 data base address.
			verify_data_len : Verify CRC8 data length.
			area_flag : Whether to mask the validation area number.
			group_flag: Whether to mask the validation workgroup number.
 @return  : true or false.
 */
bool Command_Analysis::Verify_Frame_Validity(unsigned char verify_data_base_addr, unsigned char verify_data_len, bool area_flag = true, bool group_flag = true)
{
	iwdg_feed();
	if (Verify_CRC8(verify_data_base_addr, verify_data_len) == true) {
		if (Verify_Device_Type_ID() == true) {
			Verify_Mass_Commands();

			if (Verify_Area_Number() == true || area_flag == false) {
				if (Verify_Work_Group() == true || group_flag == false) {
					return true;
				}
				else {
					Serial.println("Not this device group number... <Verify_Frame_Validity>");
				}
			}
			else {
				Serial.println("Not this device area number... <Verify_Frame_Validity>");
			}
		}
		else {
			Serial.println("Device type ID ERROR! <Verify_Frame_Validity>");
		}
	}
	else {
		Serial.println("CRC8 ERROR! <Verify_Frame_Validity>");
	}
	return false;
}

/*
 @brief   : 根据帧ID分析判断执行哪一个接收到的通用指令或私有指令
			The frame ID analysis determines which received general command or private command to execute.
 @para    : None
 @return  : None
 */
void Command_Analysis::Receive_Data_Analysis(void)
{
	iwdg_feed();
	switch (FrameID_Analysis()) {
		//General commamd.
	case Work_Para: Query_Current_Work_Para();  break;//查询当前工作段
	case Set_Group_Num: Set_Group_Number();         break;
	case SN_Area_Channel: Set_SN_Area_Channel();      break;
	case Work_Status: Detailed_Work_Status();     break;//详细的工作状态
		//Private command.
	case Work_Limit: Working_Limit_Command();    break;
	}
}

/*
 @brief   : 服务器查询当前群控相关参数，如所在区域、SN码、路数、工作组、等。（服务器 ---> 本设备）
		  : Server query current group control related parameters, such as the region, SN code, channel, working group, etc.
 @para    : None
 @return  : None
 */
void Command_Analysis::Query_Current_Work_Para(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号 | 申号标志 |  查询角色 | 采集时间间隔      |  时间   |  预留位     |  校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number | intent   |  channel | collect interval  |  RTC   |   allocate  |  CRC8   |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte       1 byte       1 byte      1 byte      2 byte           7 byte      8 byte     1 byte      6 byte
	iwdg_feed();
	if (g_Access_Network_Flag == false)  return;  //Unregistered to server, ignored.

	if (Verify_Frame_Validity(4, 23, true, false) == true) {
		Private_RTC.Update_RTC(&g_Receive_cmd[12]);

		unsigned int time_temp = (g_Receive_cmd[10] << 8 | g_Receive_cmd[11]);
		Serial.println(String("服务器设置的time_temp = ") + time_temp);
		if (WorkParameter_Info.Save_Collect_Time(time_temp))
			g_Get_Para_Flag = true;
		else
			Serial.println("Save collect time failed !!!保存收集时间失败!!");

		if (g_Receive_cmd[8] == 0X55)
			Message_Receipt.Report_General_Parameter();
		else
			Message_Receipt.General_Receipt(SetWorkParaOK, 1);
	}

	memset(g_Receive_cmd, 0x00, g_Receive_Length);
}

/*
 @brief   : 设置本设备的工作组号，不需要验证本设备原有工作组号（服务器 ---> 本设备）
			To set the wroking group number of the device, the original working group number of the device
			does not need to be verified.(server ---> the device)
 @para    : None
 @return  : None
 */
void Command_Analysis::Set_Group_Number(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   |所在执行区域号 | 工作组号   | 设备路数 |  校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag   |  Area number |  workgroup |  channel |   CRC8 |  |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte        5 byte       1 byte    1 byte      6 byte
	iwdg_feed();
	if (g_Access_Network_Flag == false)  return;  //Unregistered to server, ignored.

	if (Verify_Frame_Validity(4, 10, true, false) == true) {
		if (SN.Save_Group_Number(&g_Receive_cmd[8]) == true) {
			Serial.println("Save group number success... <Set_Group_Number>");
			Serial.println("保存组号成功…<Set_Group_Number>");
			Message_Receipt.General_Receipt(AssignGroupIdArrayOk, 1);
		}
		else {
			Serial.println("Save group number failed !!! <Set_Group_Number>");
			Serial.println("保存组号失败!!<Set_Group_Number>");
			Set_Motor_Status(STORE_EXCEPTION);
			Message_Receipt.General_Receipt(AssignGroupIdArrayErr, 1);
		}
	}
	memset(g_Receive_cmd, 0x00, g_Receive_Length);
}

/*
 @brief   : 设置本设备的SN码、区域号、设备路数等参数（服务器 ---> 本设备）
			Set the SN code, area number, channel and other parameters of the device.(server ---> the device)
 @para    : None
 @return  : None
 */
void Command_Analysis::Set_SN_Area_Channel(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位   | 所在执行区域号 |  设备路数      |  子设备总路数           |  SN码       | 校验码   |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag   |   Area number | Device channel |  subordinate channel   | SN code     |  CRC8   |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte          1 byte          1 byte           1 byte                9 byte       1 byte      6 byte
	iwdg_feed();
	if (Verify_Frame_Validity(4, 15, false, false) == true) {
		if (SN.Save_SN_Code(&g_Receive_cmd[10]) == true && SN.Save_BKP_SN_Code(&g_Receive_cmd[10]) == true) {
			Serial.println("Set SN code success... <Set_SN_Area_Channel>");

			if (SN.Save_Area_Number(g_Receive_cmd[7]) == true) {
				Serial.println("Save area number success... <Set_SN_Area_Channel>");

				Message_Receipt.General_Receipt(SetSnAndSlaverCountOk, 1);
				SN.Set_SN_Access_Network_Flag();
			}
			else {
				Serial.println("Save area number ERROR !!! <Set_SN_Area_Channel>");
				Message_Receipt.General_Receipt(SetSnAndSlaverCountErr, 1);
			}
		}
		else {
			Serial.println("Save SN code ERROR !!! <Set_SN_Area_Channel>");
			Message_Receipt.General_Receipt(SetSnAndSlaverCountErr, 1);
		}
	}
	memset(g_Receive_cmd, 0x00, g_Receive_Length);
}

/*
 @brief     : 查询本设备详细工作状态（服务器 ---> 本设备）
			  Inquire detailed working status of the device.(server ---> the device)
 @para      : None
 @return    : None
 */
void Command_Analysis::Detailed_Work_Status(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  |所在执行区域号 |  设备路数 |  校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number |   channel |   CRC8 |  |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte        1 byte         1 byte    1 byte      6 byte
	iwdg_feed();
	if (g_Access_Network_Flag == false)  return;  //Unregistered to server, ignored.

	if (Verify_Frame_Validity(4, 5, true, false) == true)
		Message_Receipt.Working_Parameter_Receipt();

	memset(g_Receive_cmd, 0x00, g_Receive_Length);
}

/*
 @brief     : 电机工作电压阈值、上报状态间隔值设置（网关 ---> 本机）
			: The motor Working valtage threshold and the report state interval value setting.
 @para      : None
 @return    : None
 */
void Command_Analysis::Working_Limit_Command(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位  | 所在执行区域号 |  工作组号   | 设备路数 | 低电压阈值       |   高电压阈值      | 状态上报间隔     |校验码 | 帧尾 
	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |   Area number |   workgroup | channel | LowVolThreshold | HighVolThreshold |  ReprotInterval | CRC8 |  Frame end
	//  1 byte       2 byte      1 byte          2 byte        1 byte         1 byte         1 byte      1 byte     2 byte             2 byte              1 byte        1 byte  6 byte
	// if (g_Access_Network_Flag == false)  return;  //Unregistered to server, ignored.

	// if (Verify_Frame_Validity(4, 11, true, true) == true){
	//   if(Control_Info.Save_Roll_Work_Voltage_and_Report_Interval(&g_Receive_cmd[10]) == true)
	//     Message_Receipt.General_Receipt(LimitRollerOk, 1);
	//   else
	//     Message_Receipt.General_Receipt(LimitRollerErr, 1);
	// }
	// memset(g_Receive_cmd, 0x00, g_Receive_Length);
}


