README
Nicholas DeMarinis
Ian Lonergan
Eric Prouty
29 April 2012

Compilation Instructions

This archive contains source files for the TinyOS demonstration
network MoteNet.  THis application demonstrates TinyOS programming
techniques as well as a basic timing analysis to handle communication
over two separate channels.
To compile the program, first run

   make telosb

After it has been compiled, it can be installed on the first mote with
      
   make telosb reinstall,0

and on the second mote with

   make telosb reinstall,1

Implementation Details

All functionality specified by the project writeup has been
implemented.  The LEDs on the motes light up as specified, with the
Blue LED showing that a node is connected and the Red and Green LEDs
showing the identify of the Near and Far nodes, which change as the
locations of the nodes change.

Design Decisions

 - Channel switching is handled with a series of one shot timers to
   ensure that each mote is listening on the broadcast channel when it
   expects to receive a beacon or target message.  These timers are
   set based on the first receipt of a beacon and target message.
   When the timer for one expires, it sets another 100ms timer to
   create a window for accepting the message.  After this timer
   expires for both the beacon and target, the mote can switch back to
   the local channel to send and receive messages on the local subnet.
   Note that these timers are set based on the first target and beacon
   message times, which means this implementation is independent of
   the delay between the target and beacon.  However, it assumes that
   the delay between the two messages does not change, which is a
   reasonable assumption given that time synchronization protocols are
   beyond the scope of the assignment.  This should not become an
   issue so long as the neither base station nor local motes are reset
   during the demonstration.

 - To ensure that messages are always sent on the correct channel, the
   call to send() on the respective channel is only invoked when the
   radio has completed.  Instead of actually sending messages, the
   logic that would send them (which, incidentally, occurs when the
   mote is listening on the opposite channel) simply sets a flag
   denoting that a transmission on that channel should occur when it
   is available.  This way, when the mote switches channels and the
   radio is ready, the message is sent.  While this implementation
   only sends one off-channel message per channel switch, a more
   extensive implementation was unnecessary given the specification.
   If such an implementation were necessary, this could be handled
   with a timer and an outgoing message queue on each channel.
