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
  components AntiTheftC, MainC, LedsC; 
  components new HamamatsuS10871TsrC() as TsrLight;

  components new TimerMilliC() as RTimer;
  components new TimerMilliC() as GTimer;
  components new TimerMilliC() as BTimer;

  components ActiveMessageC;
  components new AMSenderC(AM_TYPE);
  components new AMReceiverC(AM_TYPE);

  components SerialActiveMessageC as SerialAM;

  AntiTheftC.Boot -> MainC;
  AntiTheftC.Leds -> LedsC;

  AntiTheftC.RedTimer -> RTimer;
  AntiTheftC.GreenTimer -> GTimer;
  AntiTheftC.BlueTimer -> BTimer;

  AntiTheftC.Light -> TsrLight;

  AntiTheftC.Packet    -> AMSenderC;
  AntiTheftC.AMPacket  -> AMSenderC;
  AntiTheftC.AMSend    -> AMSenderC;
  AntiTheftC.AMControl -> ActiveMessageC;
  AntiTheftC.Receive   -> AMReceiverC;

  AntiTheftC.SerialControl -> SerialAM;
  AntiTheftC.SerialReceive -> SerialAM.Receive[AM_THEFT_SERIAL];
  AntiTheftC.SerialSend    -> SerialAM.AMSend[AM_THEFT_SERIAL];
  AntiTheftC.SerialPacket    -> SerialAM;
}


