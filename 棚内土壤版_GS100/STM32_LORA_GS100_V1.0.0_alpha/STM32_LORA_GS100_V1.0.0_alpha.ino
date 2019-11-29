// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       STM32_LORA_GS100_V1.0.0_alpha.ino
    Created:	2019/11/26 星期二 16:00:08
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
#define test_time 2000 
#define SOFT_HARD_VERSION 1
#define Software_version_high	0x01
#define Software_version_low	0x00
#define Hardware_version_high	0x07
#define Hardware_version_low	0x00

// Define Functions below here or use other .ino or cpp files
//


void Request_Access_Network(void);
void Data_Communication_with_Gateway(void);
void Sleep(void);
void Key_Reset_LoRa_Parameter(void);

unsigned char g_SN_Code[9] = { 0x00 }; //


/*----------Function statement----------*/
//void Sleep(void);


// The setup() function runs once each time the micro-controller starts
void setup()
{
	//delay(5000);
	
	afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

	Serial.begin(9600);         //DEBUG Serial baud
	LoRa_MHL9LF.BaudRate(9600);    //LoRa SoftwareSerial baud
	RS485_Serial.begin(9600);  //ModBus SoftwareSerial baud
	bkp_init(); //Initialize backup register.

	Serial.println("开始执行初始化");
	Some_Peripheral.Peripheral_GPIO_Config();
	LoRa_MHL9LF.LoRa_GPIO_Config();
	RS485_GPIO_Config();  //RS485 GPIO configuration
	EEPROM_Operation.EEPROM_GPIO_Config();
	LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);//
	delay(1000);
	LoRa_Command_Analysis.Receive_LoRa_Cmd();

	//USB_ON; //Turn on the USB enable
	//delay(10);

#if SOFT_HARD_VERSION
	Serial.println("");
	//软件版本存储程序
	if (Software_version_high == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR) &&
		Software_version_low == Vertion.Read_Software_version(SOFT_VERSION_BASE_ADDR + 1))
	{
		Serial.println(String("当前的软件版本为V") + Software_version_high + "." + Software_version_low);
	}
	else
	{
		Vertion.Save_Software_version(Software_version_high, Software_version_low);
		Serial.println(String("成功存储软件版本，当前的软件版本为V") + Software_version_high + "." + Software_version_low);
	}
	//硬件版本存储程序
	if (Hardware_version_high == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR) &&
		Hardware_version_low == Vertion.Read_hardware_version(HARD_VERSION_BASE_ADDR + 1))
	{
		Serial.println(String("当前的硬件版本为V") + Hardware_version_high + "." + Hardware_version_low);
	}
	else
	{
		Vertion.Save_hardware_version(Hardware_version_high, Hardware_version_low);
		Serial.println(String("成功存储硬件版本，当前的硬件版本为V") + Hardware_version_high + "." + Hardware_version_low);
	}
#endif

	Key_Reset_LoRa_Parameter();

	//Initialize LoRa parameter.
	if (LoRa_Para_Config.Verify_LoRa_Config_Flag() == false)
	{
		LoRa_MHL9LF.Parameter_Init();
		LoRa_MHL9LF.IsReset(true);
		LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
		LoRa_Command_Analysis.Receive_LoRa_Cmd();
		LoRa_Para_Config.Save_LoRa_Config_Flag();
	}

	SN.Clear_SN_Access_Network_Flag();

	Request_Access_Network();

	while (SN.Self_Check(g_SN_Code) == false)
	{
		LED_SELF_CHECK_ERROR;
		Serial.println("Verify SN code ERROR, try to Retrieving SN code...");
		Message_Receipt.Request_Device_SN_and_Channel();
		delay(1000);
		LoRa_Command_Analysis.Receive_LoRa_Cmd();
		delay(3000);
	}
	Serial.println("SN self_check success...");
	LED_RUNNING;


	//-----极低电压不发送数据
	if (Some_Peripheral.Get_Voltage() <= 2800)
	{
		delay(100);
		if (Some_Peripheral.Get_Voltage() <= 2800)
		{
			Private_RTC.Set_onehour_Alarm();

			Sleep();
		}
	}
	//-----极低电压不发送数据


	Serial.println("12344556");
	Data_Communication_with_Gateway();

	if (Some_Peripheral.Get_Voltage() >= 3300)
	{
		LowBalFlag = 0;
		//Serial.println("LowBalFlag = 0");
	}
	else
	{
		LowBalFlag = 1;  //如果电压小于3200mV大于2900mV

		if (Some_Peripheral.Get_Voltage() <= 3000)
		{
			LowBalFlag = 2;//如果电压小于2900mV
			//Serial.println("LowBalFlag = 0");
		}
	}

	Private_RTC.Set_Alarm();
	Serial.println("~~~~~~~~");
}

// Add the main program code into the continuous loop() function
void loop()
{
	//nvic_sys_reset();
	Sleep();
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

		LED_NO_REGISTER;
	}
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
			LoRa_Command_Analysis.Receive_LoRa_Cmd(); //waiting to receive acquisition parameter.

		wait_time = millis();

	} while (Get_Para_num < 1);

	if (Get_Para_num == 1) //If it don't receive message three times.
	{
		Serial.println("No parameter were received!!!");
		Serial.println("//////////////////");
	}
}

/*
 *brief   : This device goes sleep
 *para    : None
 *library : <Enerlib.h> <Arduino.h>
 */
void Sleep(void)
{
	Serial.println("Enter Sleep>>>>>>");
	delay(1000);
	//Serial.flush();
	Some_Peripheral.Stop_LED();
	PWR_485_OFF;
	LORA_PWR_OFF;
	USB_OFF;

	PWR_WakeUpPinCmd(ENABLE);//使能唤醒引脚，默认PA0
	PWR_ClearFlag(PWR_FLAG_WU);
	PWR_EnterSTANDBYMode();//进入待机
}

void Key_Reset_LoRa_Parameter(void)
{
	if (digitalRead(K1) == LOW) 
	{
		delay(100);
		if (digitalRead(K1) == LOW) 
		{
			//Some_Peripheral.Key_Buzz(600);
			delay(5000);
			if (digitalRead(K2) == LOW) 
			{
				delay(100);
				if (digitalRead(K2) == LOW) 
				{
					//Some_Peripheral.Key_Buzz(600);
					LoRa_Para_Config.Clear_LoRa_Config_Flag();
					Serial.println("Clear LoRa configuration flag SUCCESS... <Key_Reset_LoRa_Parameter>");
				}
			}
		}
	}
}
