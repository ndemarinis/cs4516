/*
 * MoteNet.h
 * Nicholas DeMarinis
 * 27 April 2012
 */

#ifndef MOTENET_H
#define MOTENET_H

typedef struct BeaconMsg {
  nx_uint8_t msg_type;
  nx_uint64_t base_clock;
  nx_uint8_t subnet_id;  // 0 if no request, >0 specifies a request to a subnet ID
} BeaconMsg_t;


typedef struct TargetMsg {
  nx_uint8_t msg_type;
} TargetMsg_t;

typedef struct ReportMsg {
  nx_uint8_t msg_type;
  nx_uint8_t node_id;
  nx_uint8_t subnet_id;
  nx_uint64_t report_time;
} ReportMsg_t;


typedef nx_struct localMsg
{
  nx_uint8_t who;
  nx_uint8_t state;
  nx_int8_t rssi;
} LocalMsg_t;


enum 
  {
    SUBNET_ID = 5,

    BEACON_PERIOD_MS = 1000,
    TARGET_PERIOD_MS = 500,
    WAIT_TIME = 100,
    
    // Base station/target message types
    BEACON_MSG_TYPE = 0,
    TARGET_MSG_TYPE = 1,
    REPORT_MSG_TYPE = 2,

    // Timer periods
    TX_PERIOD_MS = 1000,
    TRANSMIT_PERIOD_MS = 250,
    CS_PERIOD_MS = (2*BEACON_PERIOD_MS),
    BEACON_RANGE_PERIOD_MS = (5*BEACON_PERIOD_MS),

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
