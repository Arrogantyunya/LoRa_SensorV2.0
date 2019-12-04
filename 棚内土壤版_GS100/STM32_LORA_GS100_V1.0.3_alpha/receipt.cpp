#include "receipt.h"
#include "User_CRC8.h"
#include "LoRa.h"
#include "Memory.h"
#include "Command_Analysis.h"
#include "public.h"
#include "fun_periph.h"
#include "private_sensor.h"
#include "BCD_CON.h"
#include "Private_RTC.h"

Receipt Message_Receipt;

#define CLEAR_BUFFER_FLAG   0 //Clear server LoRa data buffer.

#define SEND_END_DELAY      200 //Send frame end data delay.
#define SEND_DATA_DELAY     500 //Send frame data delay.

unsigned char g_Motor_Status = MotorFactoryMode;  //Default status.

/*
 @brief   : 清除服务器上一次接收的LoRa数据缓存
			Clear the LoRa data cache that the server received last time.
 @para    : None
 @return  : None
*/
void Receipt::Clear_Server_LoRa_Buffer(void)
{
	unsigned char Buffer[6] = { 0x0D, 0x0A, 0x0D, 0x0A, 0x0D, 0x0A }; //Frame end.
	LoRa_Serial.write(Buffer, 6);
	delay(SEND_END_DELAY);
}

/*
 @brief   : 随机生成回执消息的回执发送微秒延时
			A microsecond delay in the receipt delivery of a randomly generated receipt message.
 @para    : random_value ---> us delay.
 @return  : None
 */
void Receipt::Receipt_Random_Wait_Value(unsigned long int *random_value)
{
	unsigned char Random_Seed;
	SN.Read_Random_Seed(&Random_Seed);
	/*Random_Seed * 1ms, 2S + Random_Seed * 0.1ms, 200ms*/
	*random_value = random(Random_Seed * 1000, 2000000) + random(Random_Seed * 100, 200000);
}

/*
 @brief   : 上报本设备通用设置参数,包括区域号、SN码、子设备路数、工作组号、采集间隔等（本机 ---> 服务器）
			该回执帧也用作设备第一次申请注册服务器。
			Report general parameter of the device, include area number, SN code, subordinate channel, workgroup, collect Interval
			and so on.(The device ---> server)
			The receipt frame is also used as the device's first registration server.
 @para    : None
 @return  : None
 */
void Receipt::Report_General_Parameter(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   |群发标志位 | 所在执行区域号  | 工作组号   | SN码    | 查询角色 | 采集时间间隔      |  时间   |  预留位     |  校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag | Area number   | workgroup | SN code | channel | collect interval  |  RTC   |   allocate  |  CRC8   |  Frame end
	//  1 byte       2 byte      1 byte          2 byte       1 byte       1 byte          5 byte     9 byte    1 byte      2 byte           7 byte      8 byte     1 byte      6 byte
	unsigned char Report_Frame[64] = { 0 };
	unsigned char Frame_Length = 0;
	unsigned char Data_Temp[10];
	unsigned long int Random_Send_Interval = 0;

	Receipt_Random_Wait_Value(&Random_Send_Interval);
	delayMicroseconds(Random_Send_Interval);

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	Report_Frame[Frame_Length++] = 0xFE; //Frame head
	Report_Frame[Frame_Length++] = 0xE0; //Frame ID
	Report_Frame[Frame_Length++] = 0x11;
	Report_Frame[Frame_Length++] = 0x24; //Frame length
	//Device type
	Report_Frame[Frame_Length++] = highByte(DEVICE_TYPE_ID);
	Report_Frame[Frame_Length++] = lowByte(DEVICE_TYPE_ID);
	//Verify mass commands flag
	g_Mass_Command_Flag == true ? Report_Frame[Frame_Length++] = 0x55 : Report_Frame[Frame_Length++] = 0x00;
	//Area number
	Report_Frame[Frame_Length++] = Control_Info.Read_Area_Number();
	//Group number
	Control_Info.Read_Group_Number(&Data_Temp[0]);
	for (unsigned char i = 0; i < 5; i++)
		Report_Frame[Frame_Length++] = Data_Temp[i];
	//SN code
	SN.Read_SN_Code(&Data_Temp[0]);
	for (unsigned char i = 0; i < 9; i++)
		Report_Frame[Frame_Length++] = Data_Temp[i];
	//channel
	Report_Frame[Frame_Length++] = 0x01;
	//Interval
	Report_Frame[Frame_Length++] = 0x00;
	Report_Frame[Frame_Length++] = 0x00;
	//RTC 
	for (unsigned char i = 0; i < 7; i++)
		Report_Frame[Frame_Length++] = 0x00;
	//The reserved 8 bytes.
	for (unsigned char i = 0; i < 8; i++)
		Report_Frame[Frame_Length++] = 0x00;
	//CRC8
	Report_Frame[Frame_Length++] = GetCrc8(&Report_Frame[4], 0x24);
	//Frame end
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? Report_Frame[Frame_Length++] = 0x0D : Report_Frame[Frame_Length++] = 0x0A;

	Serial.println("Report general parameter...");
	Print_Debug(&Report_Frame[0], Frame_Length);

	Some_Peripheral.Stop_LED();
	LoRa_Serial.write(&Report_Frame[0], Frame_Length);
	delay(SEND_DATA_DELAY);
	Some_Peripheral.Start_LED();
}

/*
 @brief   : 当本地工作组号丢失，向服务器申请本机的工作组号（本设备 ---> 服务器）
			When the local workgroup number is lost, apply to the server for local workgroup number.(The device ---> server)
 @para    : None
 @return  : None
 */
void Receipt::Request_Set_Group_Number(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   |群发标志位 | 所在执行区域号 |  设备路数      | 校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag | Area number  | Device channel |  CRC8  |  Frame end
	//  1 byte        2 byte      1 byte          2 byte       1 byte       1 byte          1 byte       1 byte     6 byte
	unsigned char Request_Frame[20] = { 0 };
	unsigned char Frame_Length = 0;
	unsigned long int Random_Send_Interval = 0;

	Receipt_Random_Wait_Value(&Random_Send_Interval);
	delayMicroseconds(Random_Send_Interval);

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	Request_Frame[Frame_Length++] = 0xFE; //Frame head
	Request_Frame[Frame_Length++] = 0xE0; //Frame ID
	Request_Frame[Frame_Length++] = 0x12;
	Request_Frame[Frame_Length++] = 0x05; //Data length

	Request_Frame[Frame_Length++] = highByte(DEVICE_TYPE_ID); //Device type ID
	Request_Frame[Frame_Length++] = lowByte(DEVICE_TYPE_ID);
	//Verify mass commands flag
	g_Mass_Command_Flag == true ? Request_Frame[Frame_Length++] = 0x55 : Request_Frame[Frame_Length++] = 0x00;
	//Area number.
	Request_Frame[Frame_Length++] = Control_Info.Read_Area_Number();  //Aread number
	Request_Frame[Frame_Length++] = 0x01; //Master device
	Request_Frame[Frame_Length++] = GetCrc8(&Request_Frame[4], 0x05);
	//Frame end
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? Request_Frame[Frame_Length++] = 0x0D : Request_Frame[Frame_Length++] = 0x0A;

	Serial.println("Requeset SN code to access network...");
	Print_Debug(&Request_Frame[0], Frame_Length);

	Some_Peripheral.Stop_LED();
	LoRa_Serial.write(&Request_Frame[0], Frame_Length);
	delay(SEND_DATA_DELAY);
	Some_Peripheral.Start_LED();
}

/*
 @brief   : 当本地SN码丢失，向服务器申请本机的SN码（本设备 ---> 服务器）
			When the local SN code is lost, apply to the server for local SN code.(The device ---> server)
 @para    : None
 @return  : None
 */
void Receipt::Request_Device_SN_and_Channel(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位 | 所在执行区域号 |  设备路数      | 校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID |  mass flag |  Area number | Device channel |  CRC8  |  Frame end
	//  1 byte        2 byte      1 byte          2 byte        1 byte      1 byte          1 byte       1 byte     6 byte
	unsigned char Request_Frame[20] = { 0 };
	unsigned char Frame_Length = 0;
	unsigned long int Random_Send_Interval = 0;

	Receipt_Random_Wait_Value(&Random_Send_Interval);
	delayMicroseconds(Random_Send_Interval);

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	Request_Frame[Frame_Length++] = 0xFE; //Frame head
	Request_Frame[Frame_Length++] = 0xE0; //Frame ID
	Request_Frame[Frame_Length++] = 0x13;
	Request_Frame[Frame_Length++] = 0x05; //Data length

	Request_Frame[Frame_Length++] = highByte(DEVICE_TYPE_ID); //Device type ID
	Request_Frame[Frame_Length++] = lowByte(DEVICE_TYPE_ID);
	//Verify mass commands flag
	g_Mass_Command_Flag == true ? Request_Frame[Frame_Length++] = 0x55 : Request_Frame[Frame_Length++] = 0x00;
	//Area number.
	Request_Frame[Frame_Length++] = Control_Info.Read_Area_Number();  //Aread number
	Request_Frame[Frame_Length++] = 0x01; //Master device
	Request_Frame[Frame_Length++] = GetCrc8(&Request_Frame[4], 0x05);
	//Frame end
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? Request_Frame[Frame_Length++] = 0x0D : Request_Frame[Frame_Length++] = 0x0A;

	Serial.println("Requeset SN code to access network...");
	Print_Debug(&Request_Frame[0], Frame_Length);

	Some_Peripheral.Stop_LED();
	LoRa_Serial.write(&Request_Frame[0], Frame_Length);
	delay(SEND_DATA_DELAY);
	Some_Peripheral.Start_LED();
}

/*
 @brief   : 上报实时详细工作状态（本机 ---> 服务器）
			Report real-time detailed working status.(The device ---> server)
 @para    : None
 @return  : None
 */
void Receipt::Working_Parameter_Receipt(void)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位 | 所在执行区域号 | 设备路数       | 设备状态        | 电压     |  RSSI  |  CSQ    | 预留位  | 校验码  | 帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag | Area number   | Device channel |  device status |  voltage |                              CRC8   | Frame end
	//  1 byte        2 byte      1 byte          2 byte       1 byte      1 byte             1 byte       1 byte           2 byte     1 byte  1 byte    8 byte   1 byte     6 byte
	unsigned char Receipt_Frame[30] = { 0 };
	unsigned char Receipt_Length = 0;
	unsigned char LoRa_CSQ[2] = { 0 };
	unsigned long int Random_Send_Interval = 0;

	Receipt_Random_Wait_Value(&Random_Send_Interval);
	delayMicroseconds(Random_Send_Interval);

	//Read LoRa module's SNR and RSSI
	LoRa_MHL9LF.LoRa_AT(LoRa_CSQ, true, AT_CSQ_, 0);

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	Receipt_Frame[Receipt_Length++] = 0xFE; //Frame head
	Receipt_Frame[Receipt_Length++] = 0xE0; //Frame ID
	Receipt_Frame[Receipt_Length++] = 0x14;
	Receipt_Frame[Receipt_Length++] = 0x12; // Data length
	//Device type
	Receipt_Frame[Receipt_Length++] = highByte(DEVICE_TYPE_ID);
	Receipt_Frame[Receipt_Length++] = lowByte(DEVICE_TYPE_ID);
	//Verify mass commands flag
	g_Mass_Command_Flag == true ? Receipt_Frame[Receipt_Length++] = 0x55 : Receipt_Frame[Receipt_Length++] = 0x00;
	//Area number
	Receipt_Frame[Receipt_Length++] = Control_Info.Read_Area_Number();
	//channel
	Receipt_Frame[Receipt_Length++] = 0x01;
	//device private status
	Receipt_Frame[Receipt_Length++] = Read_Motor_Status();
	//Voltage value.
	Receipt_Frame[Receipt_Length++] = 0x00;
	Receipt_Frame[Receipt_Length++] = 0x00;
	//SNR and RSSI 
	Receipt_Frame[Receipt_Length++] = Type_Conv.Dec_To_Hex(LoRa_CSQ[0]);  //Send data intensity.
	Receipt_Frame[Receipt_Length++] = Type_Conv.Dec_To_Hex(LoRa_CSQ[1]);  //Receive data intensity
	//The reserved 8 bytes(some of them use to report motor status information)
	// Receipt_Frame[Receipt_Length++] = Type_Conv.Dec_To_Hex(Control_Info.Read_RealTime_Opening_Value()); //Current opening.
	// unsigned int Vol_Temp = Control_Info.Read_Roll_Low_Voltage_Limit_Value();  
	// Receipt_Frame[Receipt_Length++] = highByte(Vol_Temp); //Low voltage value.
	// Receipt_Frame[Receipt_Length++] = lowByte(Vol_Temp);
	// Vol_Temp = Control_Info.Read_Roll_High_Voltage_Limit_Value();
	// Receipt_Frame[Receipt_Length++] = highByte(Vol_Temp); //High voltage value.
	// Receipt_Frame[Receipt_Length++] = lowByte(Vol_Temp);
	// unsigned int Current_Temp = Motor_Operation.Current_Detection();  //Get motor current value.
	// Receipt_Frame[Receipt_Length++] = highByte(Current_Temp);
	// Receipt_Frame[Receipt_Length++] = lowByte(Current_Temp);
	// Receipt_Frame[Receipt_Length++] = Type_Conv.Dec_To_Hex(Control_Info.Read_Roll_Report_Status_Interval_Value()); //Interval value.
	//CRC8
	Receipt_Frame[Receipt_Length++] = GetCrc8(&Receipt_Frame[4], 0x12);
	//Frame end
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? Receipt_Frame[Receipt_Length++] = 0x0D : Receipt_Frame[Receipt_Length++] = 0x0A;

	Serial.println("LoRa parameter receipt...");
	Print_Debug(&Receipt_Frame[0], Receipt_Length);

	Some_Peripheral.Stop_LED();
	LoRa_Serial.write(&Receipt_Frame[0], Receipt_Length);
	delay(SEND_DATA_DELAY);
	Some_Peripheral.Start_LED();
}

/*
 @brief   : 发送通用回执信息给服务器。（本设备 ---> 服务器）
			在大多数情况下，当接受到服务器的指令后，需要发送本条通用回执
			Send general receipt of information to server.(The device ---> server)
			In most cases, this general receipt needs to be sent after the command is received from the server.
 @para    : receipt status(enum)
 @return  : None
 */
void Receipt::General_Receipt(unsigned char status, unsigned char send_times)
{
	//  帧头     |    帧ID   |  数据长度   |    设备类型ID   | 群发标志位 | 所在执行区域号 |  设备路数      | 回执状态       |  预留位    | 校验码  |     帧尾 
	//Frame head | Frame ID | Data Length | Device type ID | mass flag | Area number | Device channel | receipt status |  allocate | CRC8    |  Frame end
	//  1 byte        2 byte      1 byte          2 byte       1 byte        1 byte          1 byte       1 byte          8 byte      1 byte     6 byte
	unsigned char Receipt_Frame[25] = { 0 };
	unsigned char Receipt_Length = 0;
	unsigned long int Random_Send_Interval = 0;

	Receipt_Random_Wait_Value(&Random_Send_Interval);
	delayMicroseconds(Random_Send_Interval);

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	Receipt_Frame[Receipt_Length++] = 0xFE; //Frame head
	Receipt_Frame[Receipt_Length++] = 0xE0; //Frame ID
	Receipt_Frame[Receipt_Length++] = 0x15;
	Receipt_Frame[Receipt_Length++] = 0x0E; // Data length
	//device type 
	Receipt_Frame[Receipt_Length++] = highByte(DEVICE_TYPE_ID);
	Receipt_Frame[Receipt_Length++] = lowByte(DEVICE_TYPE_ID);
	//Verify mass commands flag
	g_Mass_Command_Flag == true ? Receipt_Frame[Receipt_Length++] = 0x55 : Receipt_Frame[Receipt_Length++] = 0x00;
	//Area Number.
	Receipt_Frame[Receipt_Length++] = Control_Info.Read_Area_Number();
	Receipt_Frame[Receipt_Length++] = 0x01; //Master device
	Receipt_Frame[Receipt_Length++] = status;
	//The reserved 8 bytes
	for (unsigned char i = 0; i < 8; i++)
		Receipt_Frame[Receipt_Length++] = 0x00;
	//CRC8
	Receipt_Frame[Receipt_Length++] = GetCrc8(&Receipt_Frame[4], 0x0E);
	//Frame end
	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? Receipt_Frame[Receipt_Length++] = 0x0D : Receipt_Frame[Receipt_Length++] = 0x0A;

	Serial.println("Send General Receipt...");
	Print_Debug(&Receipt_Frame[0], Receipt_Length);

	Some_Peripheral.Stop_LED();
	for (unsigned char i = 0; i < send_times; i++) {
		LoRa_Serial.write(Receipt_Frame, Receipt_Length);
		delay(SEND_DATA_DELAY);
	}
	Some_Peripheral.Start_LED();
}

/*
 *brief   : Send 11-factor sensor data to gateway
 *para    : None
 *return  : None
 */
void Receipt::Send_Sensor_Data(void)
{
	unsigned char NumOfDot = 0;
	unsigned char Data_BCD[4] = { 0 };//把转好的BCD数据存入这个数组
	char weathertr[20] = { 0 };
	unsigned char Sensor_Buffer[128] = { 0 };
	unsigned char Receipt_Length = 0;
	bool Temperature_Change_Flag = false;
	float SoliTemp_Value = 0.0;
	unsigned long int Random_Send_Interval = 0;

	Receipt_Random_Wait_Value(&Random_Send_Interval);//接收随机等待值
	delayMicroseconds(Random_Send_Interval);

#if CLEAR_BUFFER_FLAG
	Clear_Server_LoRa_Buffer();
#endif

	private_sensor.Get_All_Sensor_Data();

	Sensor_Buffer[Receipt_Length++] = 0xFE; //Frame head.
	Sensor_Buffer[Receipt_Length++] = 0xD0; //Frame ID
	Sensor_Buffer[Receipt_Length++] = 0x01;
	Sensor_Buffer[Receipt_Length++] = 0x30; //Data length
	//Sensor type ID
	Sensor_Buffer[Receipt_Length++] = highByte(DEVICE_TYPE_ID); //Sensor type ID
	Sensor_Buffer[Receipt_Length++] = lowByte(DEVICE_TYPE_ID);

	unsigned int BatVol = Some_Peripheral.Get_Voltage();//得到电压值
	Sensor_Buffer[Receipt_Length++] = highByte(BatVol);
	Sensor_Buffer[Receipt_Length++] = lowByte(BatVol);

	//Air Temperature空气温度
	NumOfDot = 2;
	if (Sensor_Data.g_Temp > 150) {
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;

	}
	else {
		if (Sensor_Data.g_Temp < 0) {
			Sensor_Data.g_Temp *= -1;
			Temperature_Change_Flag = true;
		}

		PackBCD((char *)Data_BCD, Sensor_Data.g_Temp, 4, NumOfDot);
		Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
	}

	if (Temperature_Change_Flag == true)
		Sensor_Buffer[Receipt_Length++] = 0xF0 | NumOfDot;//最高位是1，表示负的数值

	else if (Temperature_Change_Flag == false)
		Sensor_Buffer[Receipt_Length++] = 0xE0 | NumOfDot;//最高位是0，表示正的数值//10

	  //Air Humidity空气湿度
	memset(Data_BCD, 0x00, sizeof(Data_BCD));//清零Data_BCD数组
	NumOfDot = 2;
	if ((int)Sensor_Data.g_Humi > 100 || (int)Sensor_Data.g_Humi <= 0) {
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;

	}
	else {
		PackBCD((char *)Data_BCD, Sensor_Data.g_Humi, 4, NumOfDot);//把大气湿度转换成BCD码
		Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
	}
	Sensor_Buffer[Receipt_Length++] = 0xE0 | NumOfDot;

	//Air light
	NumOfDot = 0;
	memset(Data_BCD, 0x00, sizeof(Data_BCD));
	memset(weathertr, 0x00, sizeof(weathertr));

	if (Sensor_Data.g_Lux > 200000) {
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;

	}
	else {
		sprintf(weathertr, "%08ld", Sensor_Data.g_Lux);
		//该函数是将一个ASCII码字符串转换成BCD码
		ASC2BCD(Data_BCD, weathertr, strlen(weathertr));//读取BCD数据数组、ASCII码字符串、该字符串的长度
		Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[2];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[3];
	}
	Sensor_Buffer[Receipt_Length++] = 0xE0 | NumOfDot;

	//-------------------------------------------------------
	//Air Pressure气压
	NumOfDot = 0;
	memset(Data_BCD, 0x00, sizeof(Data_BCD));
	PackBCD((char *)Data_BCD, Sensor_Data.g_Solid_PH /10, 4, NumOfDot);


	Sensor_Buffer[Receipt_Length++] = 0x00;
	Sensor_Buffer[Receipt_Length++] = 0x00;
	Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
	Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
	Sensor_Buffer[Receipt_Length++] = 0xE1 | NumOfDot;

	////Solid Cond土壤电导率（土壤EC）
	//NumOfDot = 0;
	//memset(Data_BCD, 0x00, sizeof(Data_BCD));

	//if (Sensor_Data.g_Cond >= 65535)
	//{
	//	Sensor_Buffer[Receipt_Length++] = 0xFF;
	//	Sensor_Buffer[Receipt_Length++] = 0xFF;

	//}
	//else 
	//{
	//	PackBCD((char *)Data_BCD, Sensor_Data.g_Cond, 4, NumOfDot);

	//	Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
	//	Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
	//}
	//Sensor_Buffer[Receipt_Length++] = 0xE0 | NumOfDot;
	//---------------------------------------------------------

	//UV紫外线
	NumOfDot = 0;
	memset(Data_BCD, 0x00, sizeof(Data_BCD));

	if (Sensor_Data.g_UV > 100) {
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;

	}
	else {
		PackBCD((char *)Data_BCD, Sensor_Data.g_UV, 4, NumOfDot);
		Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
	}
	Sensor_Buffer[Receipt_Length++] = 0xE0 | NumOfDot;

	//CO2
	Sensor_Buffer[Receipt_Length++] = 0xFF;
	Sensor_Buffer[Receipt_Length++] = 0xFF;
	Sensor_Buffer[Receipt_Length++] = 0xE0;

	//TVOC总挥发性有机化合物(Total Volatile Organic Compounds)
	Sensor_Buffer[Receipt_Length++] = 0xFF;
	Sensor_Buffer[Receipt_Length++] = 0xFF;
	Sensor_Buffer[Receipt_Length++] = 0xE0;

	//Solid temperature土壤温度
	NumOfDot = 2;
	if (Sensor_Data.g_Solid_Temp >= 65535) {
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;

	}
	else {
		if (Sensor_Data.g_Solid_Temp_Flag == 1) {
#if PR_3000_ECTH_N01
			SoliTemp_Value = (float)(65536 - Sensor_Data.g_Solid_Temp) / 100;
#else
			SoliTemp_Value = (float)(65536 - Sensor_Data.g_Solid_Temp) / 10;
#endif

		}
		else {
#if PR_3000_ECTH_N01
			SoliTemp_Value = (float)Sensor_Data.g_Solid_Temp / 100;
#else
			SoliTemp_Value = (float)Sensor_Data.g_Solid_Temp / 10;
#endif
		}

		PackBCD((char *)Data_BCD, SoliTemp_Value, 4, NumOfDot);
		Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
	}

	if (Sensor_Data.g_Solid_Temp_Flag == 1)
		Sensor_Buffer[Receipt_Length++] = 0xF0 | NumOfDot;//最高位是1，表示负的数值
	else
		Sensor_Buffer[Receipt_Length++] = 0xE0 | NumOfDot;//最高位是0，表示正的数值//10

	  //Solid humidity土壤湿度
	memset(Data_BCD, 0x00, sizeof(Data_BCD));//清零Data_BCD数组
	NumOfDot = 2;

	if ((int)(Sensor_Data.g_Solid_Humi) > 100) {
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;

	}
	else {
		PackBCD((char *)Data_BCD, Sensor_Data.g_Solid_Humi, 4, NumOfDot);//把大气湿度转换成BCD码
		Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
	}
	Sensor_Buffer[Receipt_Length++] = 0xE0 | NumOfDot;

	//Solid Cond土壤电导率（土壤EC）
	NumOfDot = 0;
	memset(Data_BCD, 0x00, sizeof(Data_BCD));

	if (Sensor_Data.g_Cond >= 65535) {
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;

	}
	else {
		PackBCD((char *)Data_BCD, Sensor_Data.g_Cond, 4, NumOfDot);

		Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
	}
	Sensor_Buffer[Receipt_Length++] = 0xE0 | NumOfDot;

	//Solid Salt土壤盐度
	NumOfDot = 0;
	memset(Data_BCD, 0x00, sizeof(Data_BCD));

	if (Sensor_Data.g_Salt >= 65535) {
		Sensor_Buffer[Receipt_Length++] = 0xFF;
		Sensor_Buffer[Receipt_Length++] = 0xFF;

	}
	else {
		PackBCD((char *)Data_BCD, Sensor_Data.g_Salt, 4, NumOfDot);
		Sensor_Buffer[Receipt_Length++] = Data_BCD[0];
		Sensor_Buffer[Receipt_Length++] = Data_BCD[1];
	}
	Sensor_Buffer[Receipt_Length++] = 0xE0 | NumOfDot;

	unsigned char date_temp[7];
	Private_RTC.Get_RTC(&date_temp[0]);
	for (unsigned char i = 0; i < 7; i++)
		Sensor_Buffer[Receipt_Length++] = date_temp[i];

	unsigned char crc8 = GetCrc8(&Sensor_Buffer[4], 0x30);
	Sensor_Buffer[Receipt_Length++] = crc8;

	for (unsigned char i = 0; i < 6; i++)
		i % 2 == 0 ? Sensor_Buffer[Receipt_Length++] = 0x0D : Sensor_Buffer[Receipt_Length++] = 0x0A;

	Serial.println("LoRa sensor data receipt...");
	Print_Debug(&Sensor_Buffer[0], Receipt_Length);

	Some_Peripheral.Stop_LED();
	LoRa_Serial.write(&Sensor_Buffer[0], Receipt_Length);
	delay(SEND_DATA_DELAY);
	Some_Peripheral.Start_LED();
}

/*
 @brief   : 串口打印16进制回执信息
			Serial port prints hex receipt information.
 @para    : base_addr ---> Frame's start address.
			len ---> Frame length.
 @return  : None
 */
void Receipt::Print_Debug(unsigned char *base_addr, unsigned char len)
{
	for (unsigned char i = 0; i < len; i++) {
		Serial.print(base_addr[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
}
