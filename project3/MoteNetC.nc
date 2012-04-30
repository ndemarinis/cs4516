/**
 * @file MoteNetC.nc
 *
 * Handles communication between motes on a subnet as specified in Project 3.
 *
 * @author Nicholas DeMarinis
 * @author Eric Prouty
 * @author Ian Lonergan
 *
 * @date 27 April 2012
 */

#include "MoteNet.h"

module MoteNetC
{
	uses interface Boot;
	uses interface Leds;

	uses interface Timer<TMilli> as BeaconInRangeTimer;

	uses interface Timer<TMilli> as B_Main;
	uses interface Timer<TMilli> as B_Wait;
	uses interface Timer<TMilli> as T_Main;
	uses interface Timer<TMilli> as T_Wait;

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

	//Booleans to inform of when we are in the broadcast channel for each type of message
	bool B_broadcast = TRUE, T_broadcast = TRUE;

	//Boolean for if we have set up our timers and are ready to actually accept messages
	bool B_ready = FALSE, T_ready = FALSE;

	// Our current channel, initialize to channel in Makefile
	uint8_t curr_channel = CHANNEL_BROADCAST; 

	// Whether we're the near or far mode
	uint8_t mote_state = MOTE_NEAR; // Start off as the near

	// RSSI of the last target packet we received
	int8_t last_target_RSSI;

	// Prototypes
	void sendBroadcast(uint8_t type);
	void sendLocal();
	void switchChannel(uint8_t channel);

	event void Boot.booted()
	{
		call AMControl.start();
	}


	/****************** TIMER EVENT HANDLERS **************************/  

	/**
	* Main function that handles switching to the broadcast channel at the appropriate timing.
	*
	* @author eprouty
	*/
	event void B_Main.fired(){
		if(!B_ready){
			//Adding original offset so that the wait is surrounding when we should get a message
			call B_Wait.startOneShot(WAIT_TIME / 2);
		} else {
			//wait for the message
			B_broadcast = TRUE;
			call B_Wait.startOneShot(WAIT_TIME);
			//If we are not already on the broadcase channel due to the Target timer switch to it
			if(!T_broadcast){
				switchChannel(CHANNEL_BROADCAST);
			}
		}
	}

	/**
	* Main function that handles switching to the local channel at the appropriate timing.
	*
	* @author eprouty
	*/
	event void B_Wait.fired(){
		if(!B_ready){
			//we are done setting up!
			B_ready = TRUE;
			//first main timer!
			call B_Main.startOneShot(BEACON_PERIOD_MS - WAIT_TIME);
		} else {
			B_broadcast = FALSE;
			call B_Main.startOneShot(BEACON_PERIOD_MS - WAIT_TIME);
			//If T is not waiting for a broadcast then switch back to the local channel for sending...
			if(!T_broadcast){
				switchChannel(CHANNEL_LOCAL);
			}
		}
	}

	/**
	* Main function that handles switching to the broadcast channel at the appropriate timing.
	*
	* @author eprouty
	*/
	event void T_Main.fired(){
		if(!T_ready){
			call T_Wait.startOneShot(WAIT_TIME / 2);
		} else {
			T_broadcast = TRUE;
			call T_Wait.startOneShot(WAIT_TIME);
			//if we are not waiting on a Beacon message than switch back to the local channel
			if(!B_broadcast){
				switchChannel(CHANNEL_BROADCAST);
			}
		}
	}

	/**
	* Main function that handles switching to the local channel at the appropriate timing.
	*
	* @author eprouty
	*/
	event void T_Wait.fired(){
		if(!T_ready){
			//We are now ready for the main target loop!
			T_ready = TRUE;
			call T_Main.startOneShot(TARGET_PERIOD_MS - WAIT_TIME);
		} else {
			T_broadcast = FALSE;
			call T_Main.startOneShot(TARGET_PERIOD_MS - WAIT_TIME);
			//if we are not waiting for a beacon message than we need to switch to the broadcast channel
			if(!B_broadcast){
				switchChannel(CHANNEL_LOCAL);
			}
		}
	}

	event void BeaconInRangeTimer.fired()
	{
		call Leds.led2Off(); // Shut off the blue LED saying we're no longer in range of the beacon
	}


	/************* RADIO CONTROL EVENT HANDLERS **************************/  

	/**
	* Enables the radio when schronization is completed and sends any waiting broadcast.
	*
	* @author ndemarinis
	*/
	event void RadioConfig.syncDone(error_t error)
	{
		if(error == SUCCESS)
		{
			//re-enable the radio
			radio_enabled = TRUE;

			if(curr_channel == CHANNEL_BROADCAST && broadcast_send_ready)
			{
				//if we are on broadcast channel and able to send, do so
				sendBroadcast(TARGET_MSG_TYPE);
			}
			else if(curr_channel == CHANNEL_LOCAL && local_send_ready)
			{
				//if we are on local channel and able to send, do so
				sendLocal();
			}
		}
	}
  
	/************* MESSAGE CONTROL EVENT HANDLERS **************************/  
	event void AMControl.startDone(error_t error)
	{
		if(error != SUCCESS)
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
	/**
	* Handles receiving a message on the broadcast channel.
	*
	* @author ndemarinis
	*
	* @param msg Message that gets returned by the function
	* @param payload The payload 
	* @param len Length of the received packet
	*
	* @return Returns the msg parameter
	*/
	event message_t *BroadcastReceive.receive(message_t *msg, void *payload, uint8_t len)
	{
		uint8_t type = *((uint8_t *)payload);
		TargetMsg_t *t_msg;
		BeaconMsg_t *b_msg;


		if(type == TARGET_MSG_TYPE)
		{
			if(!T_ready){
				call T_Main.startOneShot(TARGET_PERIOD_MS);
			}
			t_msg = (TargetMsg_t *)payload;

			// Get the RSSI of this packet	
			last_target_RSSI = (call RadioPacket.getRssi(msg)); 
			local_send_ready = TRUE; // Send a message to our other local nodes saying we updated
		}
		else if(type == BEACON_MSG_TYPE) // We received a beacon
		{
			if(!B_ready){
			call B_Main.startOneShot(BEACON_PERIOD_MS);
		}

		b_msg = (BeaconMsg_t *)payload;

		// As specified, turn on the blue LED.  
		call Leds.led2On();

		// (Re)set a timer to turn off the blue LED if we don't hear from the beacon after 5s.  
		// If the timer is already running, this will reset it.  
		call BeaconInRangeTimer.startOneShot(BEACON_RANGE_PERIOD_MS);

		// If this was a request and we're the near node, send a response
		if(mote_state == MOTE_NEAR && b_msg->subnet_id == SUBNET_ID)
			broadcast_send_ready = TRUE;
		}

		return msg;
	}

	/***************** LOCAL MESSAGE EVENT HANDLERS **************************/  

	// Notify we're done sending
	event void LocalAMSend.sendDone(message_t *msg, error_t error)
	{
		local_sending = FALSE;
	}

	/**
	* Handles receiving a message sent over the local channel.
	*
	* @author ndemarinis
	*
	* @param msg Message that gets returned by the function
	* @param payload The payload 
	* @param len Length of the received packet
	*
	* @return Returns the msg parameter
	*/
	event message_t *LocalReceive.receive(message_t *msg, void *payload, uint8_t len)
	{
	LocalMsg_t *recvd_msg = (LocalMsg_t *)payload;

	// If the RSSI value from the other node is less than our last RSSI
	// or the RSSIs are the same and our node ID is lower
	if(recvd_msg->rssi < last_target_RSSI ||
		(recvd_msg->rssi == last_target_RSSI && recvd_msg->who > TOS_NODE_ID)) 
	{
		mote_state = MOTE_NEAR; // We're now the near node, so tell the base station
		broadcast_send_ready = TRUE;
		call Leds.led0On(); // Show we're the near node by turning on the red LED
		call Leds.led1Off();
	}
	else
	{
		mote_state = MOTE_FAR;  // Otherwise, we're the far node.  
		call Leds.led1On(); // Show we're the far node by turning on the green LED
		call Leds.led0Off();
	}

	return msg;
	}

	/*************************** FUNCTIONS **************************************/  

	/**
	* Switches communication to the specified channel.
	*
	* @author ndemarinis
	*
	* @param channel The new channel to broadcast on, as specified in the MoteNet header file
	*/
	void switchChannel(uint8_t channel)
	{
		// Disable the radio while we're switching
		radio_enabled = FALSE;

		curr_channel = channel;
		call RadioConfig.setChannel(curr_channel);

		call RadioConfig.sync(); // Send our changes to the radio
	}

	/**
	* Send a message over the broadcast channel.
	*
	* @author ndemarinis
	*
	* @param type The type of message being sent out
	*/
	void sendBroadcast(uint8_t type)
	{
		ReportMsg_t *payload = 
		(ReportMsg_t *)(call BroadcastPacket.getPayload(&broadcastMsg, sizeof(ReportMsg_t)));

		//Mark that we are not ready to send another broadcast message yet
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

	/**
	* Sends a message over the local channel.
	*
	* @author ndemarinis
	*/
	void sendLocal()
	{
		//gets the payload from the current message
		LocalMsg_t *payload = 
		(LocalMsg_t *)(call LocalPacket.getPayload(&localMsg, sizeof(LocalMsg_t)));

		//Warn that we can no longer send locally for now
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
