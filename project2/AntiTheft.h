/*
 * AntiTheft.h
 * Nicholas DeMarinis
 * 21 April 2012
 */

#ifndef ANTITHEFT_H
#define ANTITHEFT_H

#define IS_RED (ANTITHEFT_MOTE_TYPE == MOTE_RED)


typedef nx_struct theft
{
  nx_uint16_t who;
} theft_t;

typedef nx_struct theft_serial
{
  nx_uint16_t who;
  nx_uint8_t color;
} theft_serial_t;

enum 
  {
    // Need to define which mote are to define our actions
    MOTE_RED = 0,
    MOTE_GREEN = 1,

    RED_SAMPLE_TIME_MS = 1000,
    GREEN_SAMPLE_TIME_MS = 2000,
    BLUE_SAMPLE_TIME_MS = 500,

    AM_TYPE = 6,
    AM_TEST_SERIAL_MSG = 0x89,

    DARK_INTERVAL = 256,
    DARK_THRESHOLD = 30
  };



#endif
