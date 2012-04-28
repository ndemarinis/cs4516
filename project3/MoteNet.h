/*
 * MoteNet.h
 * Nicholas DeMarinis
 * 27 April 2012
 */

#ifndef MOTENET_H
#define MOTENET_H

typedef nx_struct theft
{
  nx_uint16_t who;
  nx_uint8_t state;
} theft_t;

enum 
  {
    // Need to define which mote are to define our actions
    MOTE_RED = 0,
    MOTE_GREEN = 1,

    // Timer periods
    TX_PERIOD_MS = 1000,
    CS_PERIOD_MS = 5000,

    // Define near/far mote
    MOTE_NEAR = 0,
    MOTE_FAR = 1,

    // 802.4.15 Channels
    CHANNEL_BROADCAST = 26,
    CHANNEL_LOCAL = 15, // 10 + our team number, as specified

    // Active Message types
    AM_TYPE_BROADCAST = 6,
    AM_TYPE_LOCAL = 51 // (Our team number*10) + some number, as specified
  };



#endif
