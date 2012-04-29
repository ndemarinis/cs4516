/**
 * @file MoteNetAppC.nc
 *
 * @author Nicholas DeMarinis
 *
 * @date 27 April 2012
 */ 

#include "MoteNet.h"

configuration MoteNetAppC 
{
}

implementation
{
  components MoteNetC, MainC, LedsC; 
  components new HamamatsuS10871TsrC() as TsrLight;

  // Timers for periodically transmitting messages and channel switching
  components new TimerMilliC() as BTimer;

  components new TimerMilliC() as BMTimer;
  components new TimerMilliC() as BWTimer;
  components new TimerMilliC() as TMTimer;
  components new TimerMilliC() as TWTimer;

  // Message control
  components ActiveMessageC;

  // Components for sending/receiving messages on both the broadcast and local channels
  components new AMSenderC(AM_TYPE_BROADCAST) as BroadcastSend;
  components new AMReceiverC(AM_TYPE_BROADCAST) as BroadcastReceive;

  components new AMSenderC(AM_TYPE_LOCAL) as LocalSend;
  components new AMReceiverC(AM_TYPE_LOCAL) as LocalReceive;

  // Components for managing the radio channel/duty cycle
  components CC2420ControlC;
  components CC2420ActiveMessageC;

  MoteNetC.Boot -> MainC;
  MoteNetC.Leds -> LedsC;

  // Connect the timers
  MoteNetC.BeaconInRangeTimer -> BTimer;

  MoteNetC.B_Main -> BMTimer;
  MoteNetC.B_Wait -> BWTimer;
  MoteNetC.T_Main -> TMTimer;
  MoteNetC.T_Wait -> TWTimer;

  // Connect the radio components
  MoteNetC.RadioConfig -> CC2420ControlC;
  MoteNetC.RadioPacket -> CC2420ActiveMessageC;
  MoteNetC.LPLConfig   -> CC2420ActiveMessageC;

  // Connect the broadcast send/recv components
  MoteNetC.BroadcastPacket    -> BroadcastSend;
  MoteNetC.BroadcastAMPacket  -> BroadcastSend;
  MoteNetC.BroadcastAMSend    -> BroadcastSend;
  MoteNetC.BroadcastReceive   -> BroadcastReceive;

  MoteNetC.AMControl -> ActiveMessageC; // TODO:  We may need more than one of these

  // Connect the local send/recv components
  MoteNetC.LocalPacket   -> LocalSend;
  MoteNetC.LocalAMPacket -> LocalSend;
  MoteNetC.LocalAMSend   -> LocalSend;
  MoteNetC.LocalReceive  -> LocalReceive;


}


