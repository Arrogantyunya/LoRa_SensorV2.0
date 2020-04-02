/************************************************************************************
 * Code and comments : KeeganLu
 * Date：2019/3/17
 *
 * Configure some peripheral functions.Such as buzzer, LED lights, buttons and so on.
 * These peripherals are used for program debugging and human-computer interaction.
 * A common interface for each class is provided in the header file.
 *
 * If you have any questions, please send an email to me： idlukeqing@163.com
*************************************************************************************/

/************************************************************************************
 *
 * 代码与注释：卢科青
 * 日期：2019/3/17
 *
 * 配置一些外设功能。如蜂鸣器、LED灯、按钮等。这些外设用来作为程序调试和人机交互。
 * 头文件中提供了各个类的公共接口。
 *
 * 如有任何疑问，请发送邮件到： idlukeqing@163.com
*************************************************************************************/

#include "fun_periph.h"

/*Create peripheral object*/
Some_Peripherals Some_Peripheral;

LED Which_LED;
unsigned int LED_Freq = 0;  //LED_Freq * 100ms

/*
 @brief   : 设置蜂鸣器，LED，功能按钮等引脚
			Set up the buzzer, LED, function button and other pins.
 @para    : None
 @return  : None
 */
void Some_Peripherals::Peripheral_GPIO_Config(void)
{
	pinMode(USBEN_PIN, OUTPUT);
	pinMode(GREEN_PIN, OUTPUT);
	pinMode(RED_PIN, OUTPUT);
	pinMode(K1, INPUT);
	pinMode(K2, INPUT);
	pinMode(BAT_V_PIN, INPUT_ANALOG);
	pinMode(AD_V_PIN, INPUT_ANALOG);
	//Default initialize turn off LED
	delay(10);
	GREEN_OFF;
	RED_OFF;
}

/*
 @brief   : 选择要显示的LED灯（颜色），和闪烁的频率（100ms一次）
			Select the LED light (color) to display, and the flashing frequency (100ms once)
 @para    : which_led(enum)
			freq (100ms once)
 @return  : None
 */
void Some_Peripherals::LED_Display(LED which_led, unsigned char freq)
{
	Which_LED = which_led;
	LED_Freq = freq;
	Timer4.setChannel1Mode(TIMER_OUTPUTCOMPARE);
	Timer4.setPeriod(100000); // in microseconds, 100ms
	Timer4.setCompare1(1);
	Timer4.attachCompare1Interrupt(LED_Interrupt);
	Timer4.setCount(0);
}

/*
 @brief   : 暂停所有LED闪烁，同时暂停定时器4
			pause all LED flashing while pause timer 4.
 @para    : None
 @return  : None
*/
void Some_Peripherals::Stop_LED(void)
{
	Timer4.pause();
	GREEN_OFF;
	RED_OFF;
}

/*
 @brief   : 恢复定时器4计数，LED开始闪烁
			resume timer 4 to timing while LED flashing.
 @para    : None
 @return  : None
 */
void Some_Peripherals::Start_LED(void)
{
	Timer4.resume();
}

/*
 *brief   : Collect battery Voltage
 *para    : None
 *return  : 电压值
 */
unsigned int Some_Peripherals::Get_Voltage(void)
{
	unsigned int Bat_V_Buffer[9] = { 0 };
	unsigned int Bat_Temp;

	for (unsigned char i = 0; i < 9; i++) {
		Bat_V_Buffer[i] = analogRead(BAT_V_PIN) * ADC_RATE * VBAT_DIVIDER_RATIO;
		delayMicroseconds(1000);
	}

	for (unsigned char i = 0; i < 9; i++) {
		for (unsigned char j = 0; j < 8; j++) {
			if (Bat_V_Buffer[j] > Bat_V_Buffer[j + 1]) {
				Bat_Temp = Bat_V_Buffer[j + 1];
				Bat_V_Buffer[j + 1] = Bat_V_Buffer[j];
				Bat_V_Buffer[j] = Bat_Temp;
			}
		}
	}
	return (Bat_V_Buffer[4]);
}

unsigned int Some_Peripherals::Get_ADC_V(void)
{
	unsigned int V_Buffer[9] = { 0 };
	unsigned int V_Temp;

	for (unsigned char i = 0; i < 9; i++) {
		V_Buffer[i] = analogRead(AD_V_PIN) * ADC_RATE * 11.95;//11.95为经验系数，就是频繁测试大致得出的值
		delayMicroseconds(1000);
	}

	for (unsigned char i = 0; i < 9; i++) {
		for (unsigned char j = 0; j < 8; j++) {
			if (V_Buffer[j] > V_Buffer[j + 1]) {
				V_Temp = V_Buffer[j + 1];
				V_Buffer[j + 1] = V_Buffer[j];
				V_Buffer[j] = V_Temp;
			}
		}
	}
	return (V_Buffer[4]);
}

/*
 @brief   : LED闪烁中断函数
			LED flashing interrupt function
 @para    : None
 @return  : None
 */
void LED_Interrupt(void)
{
	static unsigned char LED_Status_Flag = 0; //LED on or off status flag.
	static unsigned int LED_Num = 0;
	LED_Num++;
	if (LED_Num >= LED_Freq)
	{
		LED_Num = 0;
		LED_Status_Flag = ~LED_Status_Flag;
		switch (Which_LED)
		{
		case GREEN: LED_Status_Flag == 0 ? digitalWrite(GREEN_PIN, HIGH) : digitalWrite(GREEN_PIN, LOW);
			digitalWrite(RED_PIN, LOW);   break;

		case RED: LED_Status_Flag == 0 ? digitalWrite(RED_PIN, HIGH) : digitalWrite(RED_PIN, LOW);
			digitalWrite(GREEN_PIN, LOW); break;
		}
	}
}

