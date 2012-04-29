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

  // Flags for whether we're waiting to send a message when we switch channels
  bool broadcast_send_ready = FALSE, local_send_ready = FALSE;

  bool radio_enabled = TRUE; // Flag for disabling the entire radio when we're syncing
  bool wait_until_beacon = TRUE; // Don't switch until we get a beacon message

  // Our current channel, initialize to channel in Makefile
  uint8_t curr_channel = CHANNEL_BROADCAST; 

  // Whether we're the near or far mode
  uint8_t mote_state = MOTE_NEAR; // Start off as the near

  // RSSI of the last target packet we received
  int8_t last_target_RSSI;

  // Prototypes
  void sendBroadcast(uint8_t type);
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
    call Leds.led0Off();
    call Leds.led2Off();

    if(curr_channel == CHANNEL_BROADCAST && broadcast_send_ready)
      {
	sendBroadcast(TARGET_MSG_TYPE);
	broadcast_send_ready = FALSE;
      }
    else if(curr_channel == CHANNEL_LOCAL && local_send_ready)
      {
	sendLocal();
	local_send_ready = FALSE;
      }
  }

  event void ChannelSelectTimer.fired()
  {
    // Disable the radio while we're switching
    radio_enabled = FALSE;

    // Switch to the other channel
    curr_channel = (curr_channel == CHANNEL_BROADCAST) ? CHANNEL_LOCAL : CHANNEL_BROADCAST;
    call RadioConfig.setChannel(curr_channel);
    
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
    uint32_t i;
    if(error == SUCCESS)
      {
	radio_enabled = TRUE;
	for(i = 0; i < 32768; i++);
	if(TOS_NODE_ID == 1 && curr_channel == CHANNEL_BROADCAST) // CHEAT
	  sendBroadcast(TARGET_MSG_TYPE);
      }
  }
  
  /************* MESSAGE CONTROL EVENT HANDLERS **************************/  
  event void AMControl.startDone(error_t error)
  {
    if(error == SUCCESS)
      {
	// Initialize all of our timers
	// Basic startup tasks for when the radio is enabled go here
	call TransmitTimer.startPeriodic(TRANSMIT_PERIOD_MS);

	if(TOS_NODE_ID == 1) // CHEAT
	  {
	    sendBroadcast(BEACON_MSG_TYPE);
	    call ChannelSelectTimer.startPeriodic(CS_PERIOD_MS);
	    wait_until_beacon = FALSE;
	  }
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
    uint8_t type = *((uint8_t *)payload);
    TargetMsg_t *t_msg;
    BeaconMsg_t *b_msg;

    if(type == TARGET_MSG_TYPE)
      {
	t_msg = (TargetMsg_t *)payload;
	//call Leds.led2On(); // Just blink an LED for now.  

	// Get the RSSI of this packet	
	last_target_RSSI = (call RadioPacket.getRssi(msg)); 
	local_send_ready = TRUE;
      }
    else if(type == BEACON_MSG_TYPE) // We received a beacon
      {
	b_msg = (BeaconMsg_t *)payload;

	// As specified, turn on the blue LED.  
	//call Leds.led2On();

	// (Re)set a timer to turn off the blue LED if we don't hear from the beacon after 5s.  
	// If the timer is already running, this will reset it.  
	//call BeaconInRangeTimer.startOneShot(BEACON_RANGE_PERIOD_MS);

	// If this was a request and we're the near node, send a response
	broadcast_send_ready = TRUE;
      }

    // Start the CS timer after we get the first beacon, this should keep us sync'd
    if(wait_until_beacon) 
      {
	call ChannelSelectTimer.startPeriodic(CS_PERIOD_MS);
	wait_until_beacon = FALSE;
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
    LocalMsg_t *recvd_msg = (LocalMsg_t *)payload;

    call Leds.led2On();

    // If the RSSI value from the other node is less than our last RSSI
    // or the RSSIs are the same and our node ID is lower
    if(recvd_msg->rssi < last_target_RSSI ||
       (recvd_msg->rssi == last_target_RSSI && recvd_msg->who > TOS_NODE_ID)) 
      {
	mote_state = MOTE_NEAR; // We're now the near node, so tell the base station
	broadcast_send_ready = TRUE;

	//call Leds.led0On(); // Show we're the near node by turning on the red LED
	//call Leds.led1Off();
      }
    else
      {
	mote_state = MOTE_FAR;  // Otherwise, we're the far node.  
	//call Leds.led1On(); // Show we're the far node by turning on the green LED.  
	//call Leds.led0Off();
      }
      
    return msg;
  }

  /*************************** FUNCTIONS **************************************/  

  // Send a message over the broadcast channel
  void sendBroadcast(uint8_t type)
  {
    ReportMsg_t *payload = 
      (ReportMsg_t *)(call BroadcastPacket.getPayload(&broadcastMsg, sizeof(ReportMsg_t)));

    broadcast_send_ready = FALSE;

    while(curr_channel != CHANNEL_BROADCAST);

    // Send only if we're not already sending and the radio is enabled
    if(payload && !broadcast_sending && radio_enabled)
      {
	// Populate the struct here
	payload->msg_type = type;
	payload->node_id = TOS_NODE_ID;
	payload->subnet_id = SUBNET_ID;
	payload->report_time = 0; // As specified, we don't care about this.
	
	// Send the message
	if(call BroadcastAMSend.send(AM_BROADCAST_ADDR, &broadcastMsg, sizeof(ReportMsg_t)) 
	   == SUCCESS)
	  broadcast_sending = TRUE;
      }
  }

  // Send a message over the local channel
  void sendLocal()
  {
    LocalMsg_t *payload = 
      (LocalMsg_t *)(call LocalPacket.getPayload(&localMsg, sizeof(LocalMsg_t)));

    call Leds.led0On();
    local_send_ready = FALSE;

    while(curr_channel != CHANNEL_LOCAL);
    

    if(payload && radio_enabled && !local_sending)
      {
	// Populate the struct here
	payload->who = TOS_NODE_ID;
	payload->state = mote_state;
	payload->rssi = last_target_RSSI;
	
	// Send the message (as a broadcast to the current channel)
	if(call LocalAMSend.send(AM_BROADCAST_ADDR, &localMsg, sizeof(ReportMsg_t)) 
	   == SUCCESS)
	  local_sending = TRUE;
      }
  }



}
