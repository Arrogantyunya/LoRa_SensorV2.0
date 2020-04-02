#ifndef _PRIVATE_RTC_H
#define _PRIVATE_RTC_H

#include "User_Clock.h"
#include <RTClock.h>

#define Normal			0	//正常电量
#define Low				1	//低电量
#define Extremely_Low	2	//极低电量

class date{
public:
    void Update_RTC(unsigned char *buffer);
    void Get_RTC(unsigned char *buffer);
    void Init_Set_Alarm(void);
    void Set_Alarm(void);
	void Set_onehour_Alarm(void);
};

extern date Private_RTC;

//extern bool LowBalFlag;
extern unsigned char LowBalFlag;

#endif