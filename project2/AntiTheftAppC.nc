/*
 * AntiTheftAppC.nc
 * Nicholas DeMarinis
 * 21 April 2012
 */ 

#include "AntiTheft.h"

configuration AntiTheftAppC 
{
}

implementation
{
  components DarkC, MainC, LedsC; 
  components new HamamatsuS1087ParC() as ParLight;

  components new TimerMilliC() as RTimer;
  components new TimerMilliC() as GTimer;
  components new TimerMilliC() as BTimer;

  components ActiveMessageC;
  components new AMSenderC(AM_TYPE);
  components new AMReceiverC(AM_TYPE);

  components SerialActiveMessageC as SerialAM;

  DarkC.Boot -> MainC;
  DarkC.Leds -> LedsC;

  DarkC.RedTimer -> RTimer;
  DarkC.GreenTimer -> GTimer;
  DarkC.BlueTimer -> BTimer;

  DarkC.Light -> ParLight;

  DarkC.Packet    -> AMSenderC;
  DarkC.AMPacket  -> AMSenderC;
  DarkC.AMSend    -> AMSenderC;
  DarkC.AMControl -> ActiveMessageC;
  DarkC.Receive   -> AMReceiverC;

  DarkC.SerialControl -> SerialAM;
  DarkC.SerialReceive -> SerialAM.Receive[AM_TEST_SERIAL_MSG];
  DarkC.SerialSend    -> SerialAM.AMSend[AM_TEST_SERIAL_MSG];
  DarkC.SerialPacket    -> SerialAM;
}


