/*
 * DarkC.nc
 * Nicholas DeMarinis
 * 21 April 2012
 */

#include "AntiTheft.h"

module DarkC
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
  
  bool red_on = FALSE, green_on = FALSE;

  // Prototypes
  void reportTheft();
  void notifyChange(uint16_t who, uint8_t color);


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

    if(red_on) // If some handler told us we should turn on the LED for this period, do so
      {
	call Leds.led0On();
	red_on = FALSE;
      }
    else
      call Leds.led0Off();
  }

  event void GreenTimer.fired()
  {
    // If we're the red sensor, sample our light sensor 
    if(!IS_RED)
      call Light.read();

    if(green_on) // Turn on the LED if a handler signalled for it
      {
	call Leds.led1On();
	green_on = FALSE;
      }
    else
      call Leds.led1Off();
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
    if(ok == SUCCESS && val < DARK_THRESHOLD) // If we detected darkness
      {

	// If we're red, send a packet to the serial port if our state changed, too.  
	if(IS_RED)
	  notifyChange(TOS_NODE_ID, IS_RED ? MOTE_RED : MOTE_GREEN);

	if(IS_RED) // Turn on our respective LED
	  red_on = TRUE;
	else
	  green_on = TRUE;
	
	// Then signal the others
	reportTheft();
      }
    else // Otherwise, turn off the LED belonging to us
      {
	if(IS_RED)
	  red_on = FALSE;
	else
	  green_on = FALSE;
      }
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
	  green_on = TRUE; // So turn on the LED that doesn't belong to us
	else
	  red_on = TRUE;

	// If we're red, notify the serial port we got a message
	if(IS_RED)
	  notifyChange(pkt->who, IS_RED ? MOTE_RED : MOTE_GREEN);
      }

    return msg;
  }

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
  void reportTheft()
  {
    theft_t *payload = (theft_t *)(call Packet.getPayload(&reportMsg, sizeof(theft_t)));

    if(payload && !sending)
      {
	payload->who = TOS_NODE_ID;
	if(call AMSend.send(AM_BROADCAST_ADDR, &reportMsg, sizeof(theft_t)) == SUCCESS)
	  sending = TRUE;
      }
  }


  void notifyChange(uint16_t who, uint8_t color)
  {
    theft_serial_t *payload = (theft_serial_t *)(call SerialPacket.getPayload(&serialMsg, sizeof(theft_serial_t)));

    call Leds.led2On();

    if(payload && !serial_sending)
      {
	payload->who = who;
	payload->color = color;
	
	if((call SerialSend.send(AM_BROADCAST_ADDR, &serialMsg, sizeof(theft_serial_t)) == SUCCESS))
	  serial_sending = TRUE;
      }

    call Leds.led2Off();
  }
}
