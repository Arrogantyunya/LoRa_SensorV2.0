#ifndef _FUN_PERIPH_H
#define _FUN_PERIPH_H

#include <Arduino.h>


/*一些外设的引脚*/
/*Some peripheral GPIO configuration*/
#define GREEN_PIN           PB8
#define RED_PIN             PB9

#define K1                  PB0 //Button1
#define K2                  PB4 //Button2

#define USBEN_PIN           PB3

#define BAT_V_PIN           PA7
#define AD_V_PIN            PB1


#define ADC_RATE            0.8056
#define VBAT_DIVIDER_RATIO  6

enum LED{
    RED, GREEN
};

#define USB_ON                      (digitalWrite(USBEN_PIN, LOW))
#define USB_OFF                     (digitalWrite(USBEN_PIN, HIGH))

/*LED GPIO switch*/
#define GREEN_OFF                   (digitalWrite(GREEN_PIN, LOW))
#define GREEN_ON                    (digitalWrite(GREEN_PIN, HIGH))
#define RED_OFF                     (digitalWrite(RED_PIN,  LOW))
#define RED_ON                      (digitalWrite(RED_PIN,  HIGH))

#define LED_NO_REGISTER             (Some_Peripheral.LED_Display(RED, 5))
#define LED_RUNNING                 (Some_Peripheral.LED_Display(GREEN, 10))
#define LED_SELF_CHECK_ERROR        (Some_Peripheral.LED_Display(RED, 10))
#define LED_SET_LORA_PARA_ERROR     (Some_Peripheral.LED_Display(RED, 30))

class Some_Peripherals{
public:
    /*Configurate some functional pins*/
    void Peripheral_GPIO_Config(void);
    void LED_Display(LED which_led, unsigned char freq);
    void Stop_LED(void);
    void Start_LED(void);
    unsigned int Get_Voltage(void);
    unsigned int Get_ADC_V(void);
};

void LED_Interrupt(void);

/*Create peripheral object*/
extern Some_Peripherals Some_Peripheral;

#endif