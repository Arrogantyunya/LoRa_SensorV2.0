// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       STM32_LORA_GS100_V1.0.2_alpha.ino
    Created:	2019/12/3 星期二 10:19:58
    Author:     刘家辉
*/

// Define User Types below here or use a .h file
//
#include <Arduino.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <libmaple/rcc.h>
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
#define SOFT_HARD_VERSION 0
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

unsigned char g_SN_Code[9] = { 0x00 }; //Defualt SN code is 0.

// The setup() function runs once each time the micro-controller starts
void setup()
{
	afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

	Serial.begin(9600);         //DEBUG Serial baud
	LoRa_MHL9LF.BaudRate(9600);    //LoRa SoftwareSerial baud
	RS485_Serial.begin(9600);  //ModBus SoftwareSerial baud
	bkp_init(); //Initialize backup register.
    // rtc_init(RTCSEL_HSE);

    Serial.println("开始初始化");
	Some_Peripheral.Peripheral_GPIO_Config();
	LoRa_MHL9LF.LoRa_GPIO_Config();
	RS485_GPIO_Config();  //RS485 GPIO configuration
	EEPROM_Operation.EEPROM_GPIO_Config();
	LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
	delay(1000);
	LoRa_Command_Analysis.Receive_LoRa_Cmd();

	USB_ON; //Turn on the USB enable
	delay(10);

#if SOFT_HARD_VERSION
	Vertion.Save_Software_version(0x00, 0x01);
	Vertion.Save_hardware_version(0x00, 0x01);
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


	Data_Communication_with_Gateway();


	if (Some_Peripheral.Get_Voltage() >= 3300)
	{
		LowBalFlag = 0;
	}
	else
	{
		LowBalFlag = 1;  //如果电压小于3200mV大于2900mV

		if (Some_Peripheral.Get_Voltage() <= 3000)
		{
			LowBalFlag = 2;//如果电压小于2900mV
		}
	}

	Private_RTC.Set_Alarm();
}

// Add the main program code into the continuous loop() function
void loop()
{
    Serial.println("睡觉");
    // nvic_sys_reset();
    delay(1000);
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
		Serial.println("No parameter were received !!!没有接收到信息");
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
	Some_Peripheral.Stop_LED();
	PWR_485_OFF;
	LORA_PWR_OFF;
	USB_OFF;

	Serial.println("-------");
	if (0 == PWR_GetFlagStatus(PWR_FLAG_WU))
	{
		Serial.println("SET == PWR_GetFlagStatus(PWR_FLAG_WU)");
	}
	else
	{
		Serial.println("SET != PWR_GetFlagStatus(PWR_FLAG_WU)");
	}

	if (0 == PWR_GetFlagStatus(PWR_FLAG_SB))
	{
		Serial.println("SET == PWR_GetFlagStatus(PWR_FLAG_SB)");
	}
	else
	{
		Serial.println("SET != PWR_GetFlagStatus(PWR_FLAG_SB)");
	}

	//rcc_clk_enable(RCC_PWR); /*在操作这几句睡眠语句前，加上这句。*/

	//PWR_WakeUpPinCmd(ENABLE);//使能唤醒引脚，默认PA0

	//rtc_detach_interrupt(RTC_ALARM_SPECIFIC_INTERRUPT);//a禁止RTC闹钟中断
	PWR_ClearFlag(PWR_FLAG_WU);//将WUF标志清零
	PWR_ClearFlag(PWR_FLAG_SB);

	if (SET == PWR_GetFlagStatus(PWR_FLAG_WU))
	{
		Serial.println("SET == PWR_GetFlagStatus(PWR_FLAG_WU)");
	}
	else
	{
		Serial.println("SET != PWR_GetFlagStatus(PWR_FLAG_WU)");
	}

	if (SET == PWR_GetFlagStatus(PWR_FLAG_SB))
	{
		Serial.println("SET == PWR_GetFlagStatus(PWR_FLAG_SB)");
	}
	else
	{
		Serial.println("SET != PWR_GetFlagStatus(PWR_FLAG_SB)");
	}

	delay(1000);

	PWR_EnterSTANDBYMode();//进入待机
	//PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);
}

void Key_Reset_LoRa_Parameter(void)
{
	if (digitalRead(K1) == LOW) {
		delay(100);
		if (digitalRead(K1) == LOW) {
			//Some_Peripheral.Key_Buzz(600);
			delay(5000);
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
