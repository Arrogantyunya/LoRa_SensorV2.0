#ifndef _RS485_H
#define _RS485_H

#include <Arduino.h>

#define RS485_Serial          Serial2

#define RS485_PWR_PIN       PB4 //power

#define PWR_485_ON      (digitalWrite(RS485_PWR_PIN, HIGH))
#define PWR_485_OFF     (digitalWrite(RS485_PWR_PIN, LOW))

inline void RS485_GPIO_Config(void)  {pinMode(RS485_PWR_PIN, OUTPUT); PWR_485_OFF;}

#endif
