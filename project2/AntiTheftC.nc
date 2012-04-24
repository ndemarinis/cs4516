/*
 * AntiTheftC.nc
 * Nicholas DeMarinis
 * 21 April 2012
 */

#include "AntiTheft.h"

module AntiTheftC
{
  uses interface Boot;
  uses interface Leds;

  uses interface Timer<TMilli> as RedTimer;
  uses interface Timer<TMilli> as GreenTimer;
  uses interface Timer<TMilli> as BlueTimer;

  uses interface Read<uint16_t> as Light;

  uses interface Packet;
  uses interface AMPacket;
  uses interface AMSend;
  uses interface SplitControl as AMControl;
  uses interface Receive;

  uses interface SplitControl as SerialControl;
  uses interface Receive as SerialReceive;
  uses interface AMSend as SerialSend;
  uses interface Packet as SerialPacket;
}

implementation
{

  // Globals
  message_t reportMsg;
  message_t serialMsg;

  bool sending;
  bool serial_sending;
  
  // Last state of our sensor, so we can detect a change
  // Initialize to something neither 1 nor 0 we always see a change at startup
  uint8_t my_last_state = 2;

  // Prototypes
  void reportTheft(uint8_t state);
  void notifyChange(uint16_t who, uint8_t color, uint8_t state);

  event void Boot.booted()
  {
    call AMControl.start();

    if(IS_RED) // If we're the red sensor, start up the serial port
      call SerialControl.start();
  }

  event void RedTimer.fired()
  {
    // If we're the red sensor, sample our light sensor 
    if(IS_RED)
      call Light.read();
  }

  event void GreenTimer.fired()
  {
    // If we're the red sensor, sample our light sensor 
    if(!IS_RED)
      call Light.read();
  }

  event void BlueTimer.fired()
  {
    // Turn on the blue LED if we see both the red and green LEDs on
    if((((call Leds.get()) & (LEDS_LED0|LEDS_LED1)) == (LEDS_LED0|LEDS_LED1)))
      call Leds.led2On();
    else
      call Leds.led2Off();
  }

  event void Light.readDone(error_t ok, uint16_t val)
  {
    uint8_t curr_state;

    if(ok == SUCCESS && val < DARK_THRESHOLD) // If we detected darkness
	curr_state = STATE_DARK;
    else // Otherwise, we saw light, so turn off the LED belonging to us
	curr_state = STATE_LIGHT;

    // Update our LED states based on the color
    if(IS_RED)
      {
      if(curr_state == STATE_DARK)
	call Leds.led0On();
      else
	call Leds.led0Off();
      }
    else
      {
	if(curr_state == STATE_DARK)
	  call Leds.led1On();
	else
	  call Leds.led1Off();
      }
    
    if(my_last_state != curr_state) // If our state changed
      {
	reportTheft(curr_state); // Notify the other sensors

	if(IS_RED) // Notify the serial port, if we're red
	  notifyChange(TOS_NODE_ID, MOTE_RED, curr_state);
      }
    
    // Update our last state so we can track satte changes.  
    my_last_state = curr_state;
  }

  event void AMControl.startDone(error_t error)
  {
    if(error == SUCCESS)
      {
	// Initialize all of our timers
	call RedTimer.startPeriodic(RED_SAMPLE_TIME_MS);
	call GreenTimer.startPeriodic(GREEN_SAMPLE_TIME_MS);
	call BlueTimer.startPeriodic(RED_SAMPLE_TIME_MS);
      }
    else
      call AMControl.start();
  }


  // Just implement the interface.  Nothing to see, here.  
  event void AMControl.stopDone(error_t error)
  {
  }

  
  // When we're done, signal as such.  
  event void AMSend.sendDone(message_t *msg, error_t error)
  {
    sending = FALSE;
  }

  // When we receive a message
  event message_t *Receive.receive(message_t *msg, void *payload, uint8_t len)
  {
    theft_t *pkt;

    if(len == sizeof(theft_t))
      {
	pkt = (theft_t *)payload;

	// If we get a message, it belongs to the other sensor
	if(IS_RED)
	  {
	  if(pkt->state == STATE_DARK)
	    call Leds.led1On();
	  else
	    call Leds.led1Off();
	  }
	else
	  {
	  if(pkt->state == STATE_DARK)
	    call Leds.led0On();
	  else
	    call Leds.led0Off();
	  }

	// If we're red, notify the serial port we got a message from green
	if(IS_RED)
	  notifyChange(pkt->who, MOTE_GREEN, pkt->state);
      }

    return msg;
  }

  // No need to do anything here...
  event void SerialControl.startDone(error_t error)
  {
  }


  event void SerialControl.stopDone(error_t error)
  {
  }
  

  event void SerialSend.sendDone(message_t *msg, error_t error)
  {
    serial_sending = FALSE;
  }


  event message_t *SerialReceive.receive(message_t *msg, void *payload, uint8_t len)
  {
    // We don't care if the serial port sends us anything
    return NULL;
  }


  // Send a broadcast message saying we're in darkness
  void reportTheft(uint8_t state)
  {
    theft_t *payload = (theft_t *)(call Packet.getPayload(&reportMsg, sizeof(theft_t)));

    if(payload && !sending)
      {
	// Populate the packet by sending our ID and our current state
	payload->who = TOS_NODE_ID;
	payload->state = state;

	// When send it.  
	if(call AMSend.send(AM_BROADCAST_ADDR, &reportMsg, sizeof(theft_t)) == SUCCESS)
	  sending = TRUE;
      }
  }


  void notifyChange(uint16_t who, uint8_t color, uint8_t state)
  {
    theft_serial_t *payload = (theft_serial_t *)(call SerialPacket.getPayload(&serialMsg, sizeof(theft_serial_t)));

    if(payload && !serial_sending)
      {
	// Send who changed, what color they were, and their new state
	payload->who = who;
	payload->color = color;
	payload->state = state;
	
	if((call SerialSend.send(AM_BROADCAST_ADDR, &serialMsg, sizeof(theft_serial_t)) == SUCCESS))
	  serial_sending = TRUE;
      }
  }
}
