#ifndef DEF_H_
#define DEF_H_


#include <inttypes.h>

typedef struct StrucTime
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day_of_week;
    uint8_t day_of_month;
    uint8_t month;
    uint8_t year;
} StruTime;

typedef struct StructBoitier{

//StruRTC			*RTC;							//Store the current RTC value
float			temp;							//Store the Temperature
}StruBoitier;

StruBoitier		Boitier;					// Déclaration de la structure Boitier
#endif
