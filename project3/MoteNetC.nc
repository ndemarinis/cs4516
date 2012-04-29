/*
 * MoteNetC.nc
 * Nicholas DeMarinis
 * 27 April 2012
 */

#include "MoteNet.h"

module MoteNetC
{
  uses interface Boot;
  uses interface Leds;

  uses interface Timer<TMilli> as TransmitTimer;
  uses interface Timer<TMilli> as ChannelSelectTimer;
  uses interface Timer<TMilli> as BeaconInRangeTimer;

  uses interface SplitControl as AMControl;

  // Broadcast message interfaces
  uses interface Packet as BroadcastPacket;
  uses interface AMPacket as BroadcastAMPacket;
  uses interface AMSend as BroadcastAMSend;
  uses interface Receive as BroadcastReceive;

  // Local message interfaces
  uses interface Packet as LocalPacket;
  uses interface AMPacket as LocalAMPacket;
  uses interface AMSend as LocalAMSend;
  uses interface Receive as LocalReceive;

  // Radio control interfaces
  uses interface CC2420Config as RadioConfig;
  uses interface CC2420Packet as RadioPacket;
  uses interface LowPowerListening as LPLConfig;
}

implementation
{

  // Globals
  message_t broadcastMsg;
  message_t localMsg;

  // Flags to enable/disable local and broadcast sends
  bool broadcast_sending = FALSE, local_sending = FALSE; 

  bool radio_enabled = TRUE; // Flag for disabling the entire radio when we're syncing
  bool wait_until_beacon = TRUE; // Don't switch until we get a beacon message

  // Our current channel, initialize to channel in Makefile
  uint8_t curr_channel = CHANNEL_BROADCAST; 

  // Whether we're the near or far mode
  uint8_t mote_state = MOTE_NEAR; // Start off as the near

  // Prototypes
  void sendBroadcast();
  void sendLocal();

  event void Boot.booted()
  {
    call AMControl.start();
  }


  /****************** TIMER EVENT HANDLERS **************************/  

  // This is just an example to transmit a message on whatever channel we're on
  // We probably only need to periodically transmit on the local network in the real mode
  event void TransmitTimer.fired()
  {
    call Leds.led2Off(); // Clear the LEDs saying we got a message
    call Leds.led0Off();

    while(!radio_enabled); // Wait for the radio to be enabled

    // Send a message over the current channel
    if(curr_channel == CHANNEL_BROADCAST)
      sendBroadcast();
    else
      sendLocal();
  }

  event void ChannelSelectTimer.fired()
  {
    // Wait for any sends to complete
    while(local_sending || broadcast_sending);

    // Disable the radio while we're switching
    radio_enabled = FALSE;

    // Switch to the other channel
    curr_channel = (curr_channel == CHANNEL_BROADCAST) ? CHANNEL_LOCAL : CHANNEL_BROADCAST;
    call RadioConfig.setChannel(curr_channel);
    
#if 0     // As we don't know the period, we don't need to modify this yet
    // Set the sample time for the radio based on the period
    call LPLConfig.setLocalSleepInterval((curr_channel == CHANNEL_BROADCAST) ? 
					 BROADCAST_SAMPLE_TIME_MS : LOCAL_SAMPLE_TIME_MS);
#endif

    // Just turn on an LED to show our state right now.  This is NOT the specified behavior.  
    if(curr_channel == CHANNEL_BROADCAST) 
      call Leds.led1On();
    else
      call Leds.led1Off();

    call RadioConfig.sync(); // Send our changes to the radio
  }

  event void BeaconInRangeTimer.fired()
  {
    call Leds.led2Off(); // Shut off the blue LED saying we're no longer in range of the beacon
  }


  /************* RADIO CONTROL EVENT HANDLERS **************************/  

  // Notify that we've finished syncing
  event void RadioConfig.syncDone(error_t error)
  {
    if(error == SUCCESS)
      radio_enabled = TRUE;
  }

  /************* MESSAGE CONTROL EVENT HANDLERS **************************/  
  event void AMControl.startDone(error_t error)
  {
    if(error == SUCCESS)
      {
	// Initialize all of our timers
	// Basic startup tasks for when the radio is enabled go here
	call TransmitTimer.startPeriodic(TX_PERIOD_MS);
      }
    else
      call AMControl.start();
  }


  // Just implement the interface.  Nothing to see, here.  
  event void AMControl.stopDone(error_t error)
  {

  }


  /************* BROADCAST MESSAGE EVENT HANDLERS **************************/  

  // When we're done, signal as such.  
  event void BroadcastAMSend.sendDone(message_t *msg, error_t error)
  {
    broadcast_sending = FALSE;
  }

  // When we receive a Broadcast message
  event message_t *BroadcastReceive.receive(message_t *msg, void *payload, uint8_t len)
  {
    TargetMsg_t *t_msg;
    BeaconMsg_t *b_msg;

    if((*((uint8_t *)payload)) == TARGET_MSG_TYPE)
      {
	t_msg = (TargetMsg_t *)payload;
	call Leds.led2On(); // Just blink an LED for now.  
      }
    else
      {
	b_msg = (BeaconMsg_t *)payload;

	// As specified, turn on the blue LED.  
	//call Leds.led2On();

	// (Re)set a timer to turn off the blue LED if we don't hear from the beacon after 5s.  
	call BeaconInRangeTimer.startOneShot(BEACON_RANGE_PERIOD_MS);
	
	// If this was a request and we're the near node, send a response
	if(mote_state == MOTE_NEAR && b_msg->subnet_id == SUBNET_ID) 
	  sendBroadcast();

	// Start the CS timer after we get the first beacon, this should keep us sync'd
	if(wait_until_beacon) 
	  {
	    call ChannelSelectTimer.startPeriodic(CS_PERIOD_MS);
	    wait_until_beacon = FALSE;
	  }
      }

    return msg;
  }

  /***************** LOCAL MESSAGE EVENT HANDLERS **************************/  

  // Notify we're done sending
  event void LocalAMSend.sendDone(message_t *msg, error_t error)
  {
    local_sending = FALSE;
  }

  event message_t *LocalReceive.receive(message_t *msg, void *payload, uint8_t len)
  {
#if 0 // We'd get the RSSI of the packet we just received locally as follows
    int8_t rssi = call RadioPacket.getRssi(msg); // Get the RSSI of this packet
#endif

    call Leds.led0On(); // Just blink an LED for now.  

    return msg;
  }

  /*************************** FUNCTIONS **************************************/  

  // Send a message over the broadcast channel
  void sendBroadcast()
  {
    ReportMsg_t *payload = 
      (ReportMsg_t *)(call BroadcastPacket.getPayload(&broadcastMsg, sizeof(ReportMsg_t)));

    // Send only if we're not already sending and the radio is enabled
    if(payload && radio_enabled && !broadcast_sending)
      {
	// Populate the struct here
	payload->msg_type = REPORT_MSG_TYPE;
	payload->node_id = TOS_NODE_ID;
	payload->subnet_id = SUBNET_ID;
	payload->report_time = 0; // As specified, we don't care about this.
	
	// Send the message
	if(call BroadcastAMSend.send(AM_BROADCAST_ADDR, &broadcastMsg, sizeof(theft_t)) 
	   == SUCCESS)
	  broadcast_sending = TRUE;
      }
  }

  // Send a message over the local channel
  void sendLocal()
  {
    theft_t *payload = 
      (theft_t *)(call LocalPacket.getPayload(&localMsg, sizeof(theft_t)));
    
    if(payload && radio_enabled && !local_sending)
      {
	// Populate the struct here
	payload -> who = TOS_NODE_ID;
	
	// Send the message (as a broadcast to the current channel)
	if(call LocalAMSend.send(AM_BROADCAST_ADDR, &localMsg, sizeof(theft_t)) 
	   == SUCCESS)
	  local_sending = TRUE;
      }
  }



}
