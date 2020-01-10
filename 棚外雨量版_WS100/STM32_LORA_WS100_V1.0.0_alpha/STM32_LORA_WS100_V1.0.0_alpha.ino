// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       STM32_LORA_WS100_V1.0.0_alpha.ino
    Created:	2020/01/06 星期一 09:54:33
    Author:     刘家辉
*/


#include <Arduino.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include <libmaple/iwdg.h>
#include <RTClock.h>
#include "RS485.h"
#include "LoRa.h"
#include "Memory.h"
#include "Command_Analysis.h"
#include "fun_periph.h"
//#include "Private_RTC.h"
#include "Private_Timer.h"
#include "private_sensor.h"
#include "receipt.h"
#include "Security.h"


/* 宏定义 */
//使能宏
#define SOFT_HARD_VERSION   1

//测试宏
#define Text    0

//替换宏
#define Software_version_high	0x01
#define Software_version_low	0x00
#define Hardware_version_high	0x02
#define Hardware_version_low	0x00
/*Timer timing time*/
#define TIMER_NUM             	1000000L * 1 //1S
#define CHECK_TIMER_NUM       	1000000L

/* 函数声明 */
void Request_Access_Network(void);
void Data_Communication_with_Gateway();
void Sleep(void);
void Key_Reset_LoRa_Parameter(void);
void Timer2_Interrupt(void);

/* 变量 */
unsigned char g_SN_Code[9] = { 0x00 }; //Defualt SN code is 0.
unsigned long Cyc_Send_Data_Num = 0;


// The setup() function runs once each time the micro-controller starts
void setup()
{
    afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

	iwdg_init(IWDG_PRE_256, 1000);  //6.5ms * 1000 = 6500ms.

	Serial.begin(9600);            //DEBUG Serial baud
	LoRa_MHL9LF.BaudRate(9600);     //LoRa SoftwareSerial baud
	RS485_Serial.begin(9600);       //ModBus SoftwareSerial baud
	bkp_init(); //Initialize backup register.

	iwdg_feed();

	Some_Peripheral.Peripheral_GPIO_Config();
	LoRa_MHL9LF.LoRa_GPIO_Config();
	RS485_GPIO_Config();  //RS485 GPIO configuration
	EEPROM_Operation.EEPROM_GPIO_Config();
	LoRa_MHL9LF.Mode(PASS_THROUGH_MODE);
	delay(1000);
	LoRa_Command_Analysis.Receive_LoRa_Cmd();

	iwdg_feed();

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

	Key_Reset_LoRa_Parameter();//按住按键2重置LORA参数
	if (LoRa_Para_Config.Verify_LoRa_Config_Flag() == false)
	{
		digitalWrite(RED_PIN, HIGH);
		LoRa_MHL9LF.Parameter_Init(false);
		digitalWrite(RED_PIN, LOW);
	}

	SN.Clear_SN_Access_Network_Flag(); //按键1清除注册到服务器标志位

	/*Request access network(request gateway to save the device's SN code and channel)*/
	Request_Access_Network(); //检查是否注册到服务器,按键1申号

	while (SN.Self_check(g_SN_Code) == false)
	{
		LED_SELF_CHECK_ERROR;
		Serial.println("Verify SN code failed, try to Retrieving SN code...");
		Serial.println("验证SN代码失败，尝试找回SN代码…");
		Message_Receipt.Request_Device_SN_and_Channel(); //当本地SN码丢失，向服务器申请本机的SN码
		LoRa_Command_Analysis.Receive_LoRa_Cmd();		 //接收LORA参数
		MyDelayMs(3000);
		iwdg_feed();
	}
	Serial.println("SN self_check success...SN自检成功");
	LED_RUNNING;
	iwdg_feed();

	Self_Check_Parameter_Timer_Init(); //使用定时器3初始化自检参数功能自检周期
	Serial.println("定时自检机制初始化完成");

	//Private_RTC.Set_Alarm();
	Timer_Init();
}

// Add the main program code into the continuous loop() function
void loop()
{
    iwdg_feed();

	//Data_Communication_with_Gateway();

	private_sensor.Detect_Rain();

	//这里是得到电平的反转（雨量传感器的检测到晴天或雨天），然后开始发送新的实时数据
	if (g_Current_Rain_Status_Flag != g_Last_Rain_Status_Flag)
	{
		Serial.println("Send real-time data of sensor...");//发送实时传感器数据
		Data_Communication_with_Gateway();
	}

	if (g_Send_Air_SensorData_Flag == true)
	{
		//Serial.println("==========");
		g_Send_Air_SensorData_Flag = false;
		Serial.println("Send cyclic data of sensor...");//发送传感器的周期数据
		Data_Communication_with_Gateway();
		//Private_RTC.Set_Alarm();
		Timer2.setCount(0);//设置当前计时器计数
		Timer2.resume();//在不影响其配置的情况下恢复暂停的计时器
	}

	Check_Store_Param_And_LoRa(); //检查存储参数以及LORA参数
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
	if (SN.Verify_SN_Access_Network_Flag() == false)
	{
		g_Access_Network_Flag = false;
		iwdg_feed();
		if (SN.Save_SN_Code(g_SN_Code) && SN.Save_BKP_SN_Code(g_SN_Code))
			Serial.println("Init SN success...");

		if (SN.Clear_Area_Number() && SN.Clear_Group_Number())
			Serial.println("Already Clear area number and grouop number...");

		unsigned char Default_WorkGroup[5] = { 0x01, 0x00, 0x00, 0x00, 0x00 };
		if (SN.Save_Group_Number(Default_WorkGroup))
			Serial.println("Save gourp number success...");

		LED_NO_REGISTER;
	}
	Serial.println("Please press button 1 to register...");
	while (SN.Verify_SN_Access_Network_Flag() == false)
	{
		iwdg_feed();
		if (digitalRead(K1) == LOW) {
			iwdg_feed();
			delay(5000);
			iwdg_feed();
			if (digitalRead(K1) == LOW) {
				Message_Receipt.Report_General_Parameter();

				while (digitalRead(K1) == LOW)
					iwdg_feed();
			}
		}
		LoRa_Command_Analysis.Receive_LoRa_Cmd();
		iwdg_feed();
	}
	g_Access_Network_Flag = true;
}

/*
 *brief   : Send sensor data to gateway and then received some parameters from gateway.
			向网关发送传感器数据，然后从网关接收一些参数。
 *para    : None
 *return  : None
 */
void Data_Communication_with_Gateway(void)//与网关的数据通信
{
	unsigned char Get_Para_num = 0;
	unsigned long wait_time = millis();

	//If it don't receive a message from the gateway.
	//如果它没有收到来自网关的消息。
	do {
		iwdg_feed();
		if (g_Get_Para_Flag == false)
		{
			Message_Receipt.Send_Sensor_Data(); //Send sensor data to gateway again.再次发送传感器数据到网关。
			Get_Para_num++;
		}
		else
		{
			g_Last_Rain_Status_Flag = g_Current_Rain_Status_Flag;//最后一场雨状态标志 = 现时雨量状况标志
			g_Get_Para_Flag = false;
			break; //If receive acquisition parameter, break loop.如果接收到采集参数，断开循环。
		}
		while ((millis() < (wait_time + 10000)) && g_Get_Para_Flag == false)  //等待接收采集参数。
			LoRa_Command_Analysis.Receive_LoRa_Cmd(); //waiting to receive acquisition parameter.等待接收采集参数。

		wait_time = millis();

	} while (Get_Para_num < 3);

	if (Get_Para_num == 3)
	{	//If it don't receive message three times.如果3次没有收到信息。
		Serial.println("No parameter were received !!!");

		//在这里进行LORA的切换，切换为组播对所有的卷膜机发送关棚指令

	}
}

/*
 *brief     :This device goes sleep
 *para      :None
 *library   :<Enerlib.h> <Arduino.h>
 *return    :None
 */
void Sleep(void)
{
	Serial.println("Enter Sleep>>>>>>");
	Some_Peripheral.Stop_LED();
	PWR_485_OFF;
	LORA_PWR_OFF;
	USB_OFF;

	PWR_WakeUpPinCmd(ENABLE);//使能唤醒引脚，默认PA0
	PWR_ClearFlag(PWR_FLAG_WU);
	PWR_EnterSTANDBYMode();//进入待机
}

/*
 *brief     :Key_Reset_LoRa_Parameter
 *para      :None
 *library   :<Arduino.h>
 *return    :None
 */
void Key_Reset_LoRa_Parameter(void)
{
	if (digitalRead(K2) == LOW) {
		delay(100);
		if (digitalRead(K2) == LOW) {
			//Some_Peripheral.Key_Buzz(600);
			Serial.println("Please press key 2");
			iwdg_feed();
			delay(5000);
			iwdg_feed();
			if (digitalRead(K2) == LOW) {
				delay(100);
				if (digitalRead(K2) == LOW) 
				{
					//Some_Peripheral.Key_Buzz(600);
					LoRa_Para_Config.Clear_LoRa_Config_Flag();
					Serial.println("Clear LoRa configuration flag SUCCESS... <Key_Reset_LoRa_Parameter>");
					iwdg_feed();
				}
			}
		}
	}
}

void Timer_Init(void)
{
	Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
	Timer2.setPeriod(1000000L); // in microseconds，1S
	Timer2.setCompare1(1);   // overflow might be small
	Timer2.attachCompare1Interrupt(Timer2_Interrupt);
	Timer2.setCount(0);
	//Timer2.pause(); 
}

void Timer2_Interrupt(void)
{
	Cyc_Send_Data_Num++;
	if (Cyc_Send_Data_Num >= WorkParameter_Info.Read_Collect_Time())
	{
		Timer2.pause();
		Cyc_Send_Data_Num = 0;
		g_Send_Air_SensorData_Flag = true;
	}
}

void MyDelayMs(unsigned int d)
{
	unsigned long WaitNum = millis();

	while (millis() <= (WaitNum + d))
	{
		/*
		  *millis()函数自程序运行，开始计时，大约50天后溢出到0，再计时。
		  *为了防止发生溢出后，延时函数出错，当检测到毫秒计时小于赋值的毫秒数，
		  *再次重新赋值，重新判断。
		 */
		if (millis() < WaitNum)
			WaitNum = millis();
	}
}
