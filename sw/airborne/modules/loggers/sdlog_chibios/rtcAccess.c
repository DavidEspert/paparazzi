#include "rtcAccess.h"
#include <ch.h>
#include <hal.h>
#include <rtc.h>
#include <time.h>


static RTCDateTime utime;
static uint32_t weekDay (void);


void setHour (uint32_t val)
{
  rtcGetTime (&RTCD1, &utime);
  uint32_t rem = utime.millisecond % (3600 * 1000);
  utime.millisecond = (3600 * 1000 * val) + rem;
  utime.dstflag = 0;
  rtcSetTime (&RTCD1, &utime);
}

void setMinute (uint32_t val)
{
  rtcGetTime (&RTCD1, &utime);
  uint32_t rem = utime.millisecond % (60 * 1000);
  utime.millisecond = utime.millisecond - rem + (val * 60 * 1000);
  rtcSetTime (&RTCD1, &utime);
}

void setSecond (uint32_t val)
{
  rtcGetTime (&RTCD1, &utime);
  uint32_t rem = utime.millisecond % (1000);
  utime.millisecond = utime.millisecond - rem + (val * 1000);
  rtcSetTime (&RTCD1, &utime);
}

void setYear (uint32_t val)
{
  rtcGetTime (&RTCD1, &utime);
  utime.year = val - 1980;
  rtcSetTime (&RTCD1, &utime);
  setWeekDay (weekDay());
}

void setMonth (uint32_t val)
{
  rtcGetTime (&RTCD1, &utime);
  utime.month = val;
  rtcSetTime (&RTCD1, &utime);
  setWeekDay (weekDay());
}

void setMonthDay (uint32_t val)
{
  rtcGetTime (&RTCD1, &utime);
  utime.day = val;
  rtcSetTime (&RTCD1, &utime);
  setWeekDay (weekDay());
}

void setWeekDay (uint32_t val)
{
  rtcGetTime (&RTCD1, &utime);
  utime.dayofweek = val;
  rtcSetTime (&RTCD1, &utime);
}

uint32_t getHour (void)
{
  rtcGetTime (&RTCD1, &utime);
  return (utime.millisecond / 3600 / 1000);
}
uint32_t getMinute (void)
{
  rtcGetTime (&RTCD1, &utime);
  return ((utime.millisecond / 1000) % 3600) / 60;
}
uint32_t getSecond (void)
{
  rtcGetTime (&RTCD1, &utime);
  return ((utime.millisecond / 1000) % 3600) % 60;
}
uint32_t getYear (void)
{
  rtcGetTime (&RTCD1, &utime);
  return utime.year + 1980;
}
uint32_t getMonth (void)
{
  rtcGetTime (&RTCD1, &utime);
  return utime.month;
}
uint32_t getMonthDay (void)
{
  rtcGetTime (&RTCD1, &utime);
  return utime.day;
}
uint32_t getWeekDay (void)
{
  rtcGetTime (&RTCD1, &utime);
  return utime.dayofweek;
}

const char *getWeekDayAscii (void)
{
  static const char* weekDays[] = {
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday"
  };

  return weekDays[(getWeekDay())];
}



void setRtcFromGps (int16_t week, uint32_t tow)
{
  // Unix timestamp of the GPS epoch 1980-01-06 00:00:00 UTC
  const uint32_t unixToGpsEpoch = 315964800;
  struct tm time_tm;

  time_t univTime = ((week * 7 * 24 * 3600) + (tow/1000)) + unixToGpsEpoch;
  gmtime_r(&univTime, &time_tm);

  RTCDateTime date;
  rtcConvertDateTimeToStructTm(&date, &time_tm, NULL);
  rtcSetTime (&RTCD1, &date);
}



static uint32_t weekDay (void)
{
  const uint32_t day = getMonthDay();
  const uint32_t month = getMonth();
  const uint32_t year = getYear();

  return  (day
    + ((153 * (month + 12 * ((14 - month) / 12) - 3) + 2) / 5)
    + (365 * (year + 4800 - ((14 - month) / 12)))
    + ((year + 4800 - ((14 - month) / 12)) / 4)
    - ((year + 4800 - ((14 - month) / 12)) / 100)
    + ((year + 4800 - ((14 - month) / 12)) / 400)
    - 32045) % 7;
}
