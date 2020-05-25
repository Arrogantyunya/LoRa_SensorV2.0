#include "public.h"

TypeConv Type_Conv;

unsigned char TypeConv::Hex_To_Dec(unsigned char hex)
{
    unsigned char Temp1, Temp2;
    Temp1 = hex / 10;
    Temp2 = hex % 10;
    Temp2 = Temp1 * 16 + Temp2;
    return Temp2;
}

void TypeConv::Hex_To_Dec(unsigned char *hex)
{
    unsigned char Temp1, Temp2;
    Temp1 = *hex / 10;
    Temp2 = *hex % 10;
    *hex = Temp1 * 16 + Temp2;
}

unsigned char TypeConv::Dec_To_Hex(unsigned char dec)
{
    unsigned char Temp1, Temp2;
    Temp1 = dec / 16;
    Temp2 = dec % 16;
    Temp2 = Temp1 * 10 + Temp2;
    return Temp2;
}

void TypeConv::Dec_To_Hex(unsigned char *dec)
{
    unsigned char Temp1, Temp2;
    Temp1 = *dec / 16;
    Temp2 = *dec % 16;
    *dec = Temp1 * 10 + Temp2;
}