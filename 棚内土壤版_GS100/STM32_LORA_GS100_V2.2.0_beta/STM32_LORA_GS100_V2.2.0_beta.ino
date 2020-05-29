// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       STM32_LORA_GS100_V2.1.4_beta.ino
    Created:	2020/05/02 星期六 11:14:05
    Author:     刘家辉
*/

// Define User Types below here or use a .h file
//
#include <Arduino.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <RTClock.h>
#include "RS485.h"
#include "LoRa.h"
#include "Memory.h"
#include "Command_Analysis.h"
#include "fun_periph.h"
#include "Private_RTC.h"
#include "private_sensor.h"
#include "receipt.h"


// Define Function Prototypes that use User Types below here or use a .h file
//
#define test_time 				2000 
#define Collect_battery_num		2
#define SOFT_HARD_VERSION 		true
#define Software_version_high	0x02
#define Software_version_low	0x20
#define Hardware_version_high	0x07
#define Hardware_version_low	0x10

// Define Functions below here or use other .ino or cpp files
//


void Request_Access_Network(void);
void Data_Communication_with_Gateway(void);
void Sleep(void);
void Key_Reset_LoRa_Parameter(void);
void Serial_Port_Configuration(void);
void Debug_Project(void);

/* 变量 */
String comdata = "";//串口接收的字符串
unsigned char g_SN_Code[9] = { 0x00 }; //
bool Debug = false;


// The setup() function runs once each time the micro-controller starts
void setup()
{
	afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);


	Some_Peripheral.Peripheral_GPIO_Config();

	Serial.begin(115200);         //DEBUG Serial baud
	LoRa_MHL9LF.BaudRate(9600);    //LoRa SoftwareSerial baud
	RS485_Serial.begin(9600);  //ModBus SoftwareSerial baud
	bkp_init(); //Initialize backup register.


	LoRa_MHL9LF.LoRa_GPIO_Config();
	RS485_GPIO_Config();  //RS485 GPIO configuration
	EEPROM_Operation.EEPROM_GPIO_Config();
	LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
	delay(1000);
	LoRa_Command_Analysis.Receive_LoRa_Cmd();//接收LORA收到的数据

	USB_ON; //Turn on the USB enable
	delay(10);

#if SOFT_HARD_VERSION
	Serial.println("");

	//软件版本存储程序
	if (Software_version_high == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR) &&
		Software_version_low == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR + 1))
	{
		Serial.println(String("Software_version is V") + String(Software_version_high, HEX) + "." + String(Software_version_low, HEX));
	}
	else
	{
		Vertion.Save_Software_version(Software_version_high, Software_version_low);
		Serial.println(String("Successfully store the software version, the current software version is V") + String(Software_version_high, HEX) + "." + String(Software_version_low, HEX));
	}
	//硬件版本存储程序
	if (Hardware_version_high == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR) &&
		Hardware_version_low == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR + 1))
	{
		Serial.println(String("Hardware_version is V") + Hardware_version_high + "." + Hardware_version_low);
	}
	else
	{
		Vertion.Save_hardware_version(Hardware_version_high, Hardware_version_low);
		Serial.println(String("Successfully store the hardware version, the current hardware version is V") + Hardware_version_high + "." + Hardware_version_low);
	}
#endif


	Key_Reset_LoRa_Parameter();//按键2重置LORA参数

	//Initialize LoRa parameter.
	if (LoRa_Para_Config.Verify_LoRa_Config_Flag() == false)
	{
		if (!LoRa_Para_Config.Verify_LoRa_TRMode_Flag())
		{
			LoRa_Para_Config.Save_LoRa_TRMode(0x01);//保存为默认状态，也就是模式1
		}
		Lora_TRMode = LoRa_Para_Config.Read_LoRa_TRMode();
		Serial.println(String("Lora_TRMode = ") + String(Lora_TRMode,HEX));

		if (!LoRa_Para_Config.Verify_LoRa_SYNC_Flag())
		{
			LoRa_Para_Config.Save_LoRa_SYNC(0x34);//同步字默认设置为0x34
		}
		LORA_SYNC = LoRa_Para_Config.Read_LoRa_SYNC();
		Serial.println(String("LORA_SYNC = ") + String(LORA_SYNC));
	
		LoRa_MHL9LF.Parameter_Init(All_parameter);//设置所有的参数
		LoRa_MHL9LF.IsReset(true);
		LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
		LoRa_Command_Analysis.Receive_LoRa_Cmd();
		LoRa_Para_Config.Save_LoRa_Config_Flag();
	}

	SN.Clear_SN_Access_Network_Flag();
	Request_Access_Network();//按下按键1

	while (SN.Self_Check(g_SN_Code) == false)
	{
		LED_SELF_CHECK_ERROR;
		Serial.println("Verify SN code ERROR, try to Retrieving SN code...");
		Message_Receipt.Request_Device_SN_and_Channel();//向服务器申请本机的SN码
		delay(1000);
		LoRa_Command_Analysis.Receive_LoRa_Cmd();//等待接收指令
		delay(3000);
	}
	Serial.println("SN self_check success. ");
	Serial.print("My SN is:");
	for (size_t i = 0; i < 9; i++)
	{
		Serial.print(g_SN_Code[i],HEX);
		Serial.print(" ");
	}
	Serial.println();
	
	LED_RUNNING;

	unsigned int Battery_Voltage = 0;
	unsigned long X = 0;
	for (size_t i = 0; i < Collect_battery_num; i++)
	{
		X +=  Some_Peripheral.Get_Voltage();
	}
	Battery_Voltage = X/Collect_battery_num;
	
	Serial.println(String("Battery_Voltage = ") + Battery_Voltage + "mv");

	//-----极低电压不发送数据
	if (Battery_Voltage <= 3000)
	{
		delay(100);
		if (Battery_Voltage <= 3000)
		{
			Private_RTC.Set_onehour_Alarm();

			Sleep();
		}
	}
	//-----极低电压不发送数据


	Data_Communication_with_Gateway();//发送数据至网关


	if (Battery_Voltage >= 3700)
	{
		LowBalFlag = Normal;
	}
	else
	{
		LowBalFlag = Low;  //如果电压小于3700mV大于3300mV

		if (Battery_Voltage <= 3300)
		{
			LowBalFlag = Extremely_Low;//如果电压小于3000mV
		}
	}

	Private_RTC.Set_Alarm();//设置RTC闹钟

	Serial_Port_Configuration();//串口设置参数
}

void loop()
{
	Debug_Project();//现场调试模式

	Sleep();//进入停机模式
}

/*
 @brief   : 检测是否已经注册到服务器成功，如果没有注册，则配置相关参数为默认参数，然后注册到服务器。
			没有注册成功，红灯1每隔500ms闪烁。
			Checks whether registration with the server was successful, and if not,
			configures the relevant parameters as default parameters and registers with the server.
			Failing registration, red light flashes every 500ms.
 @para    : None
 @return  : None
 */
void Request_Access_Network(void)
{
	if (SN.Verify_SN_Access_Network_Flag() == false) {
		g_Access_Network_Flag = false;

		if (SN.Save_SN_Code(g_SN_Code) && SN.Save_BKP_SN_Code(g_SN_Code))
			Serial.println("Write SN success...");

		if (Control_Info.Clear_Area_Number() && Control_Info.Clear_Group_Number())
			Serial.println("Already Clear area number and grouop number...");

		unsigned char Default_WorkGroup[5] = { 0x01, 0x00, 0x00, 0x00, 0x00 };
		if (Control_Info.Save_Group_Number(Default_WorkGroup))
			Serial.println("Save gourp number success...");

		LED_NO_REGISTER;//未注册到服务器，红灯闪烁
	}
	Serial.println("Please press key 1");
	while (SN.Verify_SN_Access_Network_Flag() == false) {

		if (digitalRead(K1) == LOW) {
			delay(5000);
			if (digitalRead(K1) == LOW) {
				Message_Receipt.Report_General_Parameter();

				while (digitalRead(K1) == LOW);
			}
		}
		LoRa_Command_Analysis.Receive_LoRa_Cmd();
	}
	g_Access_Network_Flag = true;
}

/*
 *brief   : Send sensor data to gateway and then received some parameters from gateway.
 *para    : None
 *return  : None
 */
void Data_Communication_with_Gateway(void)
{
	unsigned char Get_Para_num = 0;
	unsigned long wait_time = millis();

	do {
		if (g_Get_Para_Flag == false) { //If it don't receive a message from the gateway.
			Message_Receipt.Send_Sensor_Data(); //Send sensor data to gateway again.
			Get_Para_num++;
		}
		else
			break; //If receive acquisition parameter, break loop.

		while ((millis() < (wait_time + 10000)) && g_Get_Para_Flag == false)  //waiting to receive acquisition parameter.
			LoRa_Command_Analysis.Receive_LoRa_Cmd(); //waiting to receive acquisition parameter.从网关接收LoRa数据（网关 ---> 本机）

		wait_time = millis();

	} while (Get_Para_num < 3);

	if (Get_Para_num == 3) //If it don't receive message three times.
		Serial.println("No parameter were received !!!");
}

/*
 *brief   : This device goes sleep
 *para    : None
 *library : <Enerlib.h> <Arduino.h>
 */
void Sleep(void)
{
	Serial.println("Enter Sleep>>>>>>");
	Serial.println("进入停机模式");
	Serial.flush();
	Some_Peripheral.Stop_LED();
	// PWR_485_OFF;
	LORA_PWR_OFF;
	USB_OFF;

	PWR_WakeUpPinCmd(ENABLE);//使能唤醒引脚，默认PA0
	PWR_ClearFlag(PWR_FLAG_WU);
	PWR_EnterSTANDBYMode();//进入待机
}

void Key_Reset_LoRa_Parameter(void)
{
	if (digitalRead(K2) == LOW) {
		delay(100);
		if (digitalRead(K2) == LOW) {
			//Some_Peripheral.Key_Buzz(600);
			delay(2000);
			Serial.println("K1释放，K2按下");
			delay(2000);
			if (digitalRead(K2) == LOW) {
				delay(100);
				if (digitalRead(K2) == LOW) {
					//Some_Peripheral.Key_Buzz(600);
					LoRa_Para_Config.Clear_LoRa_Config_Flag();
					Serial.println("Clear LoRa configuration flag SUCCESS... <Key_Reset_LoRa_Parameter>");
				}
			}
		}
	}
}


/*
 @brief   : 串口设置
 @para    : 无
 @return  : 无
 */
void Serial_Port_Configuration(void)
{
	while (Serial.available() > 0)
	{
		comdata += char(Serial.read());  //每次读一个char字符，并相加
		delay(2);
	}

	if (comdata.length() > 0)
	{
		comdata.toUpperCase();
		Serial.println(String("comdata = ") + comdata);

		if (comdata == String("GET_CSQ"))
		{
			//在这里通过AT指令查询信号质量

			Serial.println("开始查询信号质量...");
			LoRa_MHL9LF.LoRa_AT(gLoRaCSQ, true, AT_CSQ_, 0);
			// comdata = "";
		}
		else if (comdata == String("DEBUG"))
		{
			Debug = !Debug;
			if (Debug)
			{
				Serial.println("DEBUG模式已开启");
			}
			else
			{
				Serial.println("DEBUG模式已关闭");
			}
		}
		else if (comdata == String("GET_SENSOR"))
		{
			Serial.println("开始发送传感器数据");
			Message_Receipt.Send_Sensor_Data();
		}
		else if (comdata.startsWith("SET_LORA_TRMODE="))//SET_LORA_TRMODE=1 
		{
			Serial.println("设置LoRa的TRMode");
			String Str_x = comdata.substring(comdata.indexOf("=")+1,comdata.length());//可以配合indexOf使用
			unsigned char x = (unsigned char) Str_x.toInt();//使用long来接收,所以强转为unsigned char
			LoRa_Para_Config.Save_LoRa_TRMode(x);
			Lora_TRMode = LoRa_Para_Config.Read_LoRa_TRMode();
		}
		else if (comdata.startsWith("SET_LORA_SYNC="))//SET_LORA_SYNC=12
		{
			Serial.println("设置LoRa的SYNC");
			String Str_x = comdata.substring(comdata.indexOf("=")+1,comdata.length());//可以配合indexOf使用
			unsigned char x = (unsigned char) Str_x.toInt();//使用long来接收,所以强转为unsigned char
			LoRa_Para_Config.Save_LoRa_SYNC(x);
			LORA_SYNC = LoRa_Para_Config.Read_LoRa_SYNC();
		}
		else if(comdata == String("LORA_INIT"))
		{
			LoRa_MHL9LF.Parameter_Init(All_parameter);//LORA参数设置
		}
		else
		{
			Serial.println("输入错误 <Serial_Port_Configuration>");
		}
		comdata = "";
	}
}

/*
 @brief   : 现场调试模式
 @para    : 无
 @return  : 无
 */
void Debug_Project(void)
{
	while (Debug)
	{
		Serial_Port_Configuration();
	}
}