/* 
 * AntiTheftC.nc
 * Nicholas DeMarinis
 * 21 April 2012
 */

module AntiTheftC
{
  uses interface Boot;
  uses interface Leds;
  uses interface Timer <Tmilli> as WarningTimer;
}


implementation
{
  enum {
    WARN_INTERVAL = 4096,
    WARN_DURATION = 64
  };

  event void WarningTimer.fired()
  {
    if(call Leds.get() & LEDS_LED0)
      {
	call Leds.led0Off();
	call WarningTimer.startOneShot(WARN_INTERVAL - WARN_DURATION);
      }
    else
      {
	call Leds.led0On();
	call WarningTimer.startOneShot(WARN_DURATION);
      }
  }

  event void Boot.booted()
  {
    signal WarningTimer.fired();
  }

}
