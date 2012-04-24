/*									tab:4
 * "Copyright (c) 2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and
 * its documentation for any purpose, without fee, and without written
 * agreement is hereby granted, provided that the above copyright
 * notice, the following two paragraphs and the author appear in all
 * copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE UNIVERSITY OF CALIFORNIA HAS BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 */

/**
 * Java-side application for testing serial port communication.
 * 
 *
 * @author Phil Levis <pal@cs.berkeley.edu>
 * @date August 12 2005
 *
 * Modified by Ian Lonergan to including queueing of messages
 */

import java.io.IOException;

import net.tinyos.message.*;
import net.tinyos.packet.*;
import net.tinyos.util.*;

public class TheftSerial implements MessageListener {

    private MoteIF moteIF;
  
    private final int STATE_DARK = 0, STATE_LIGHT = 1;
    private final int MOTE_RED = 0, MOTE_GREEN = 1;
	
	Queue<TheftSerialMsg> messageQ;

    public TheftSerial(MoteIF moteIF) {
	this.moteIF = moteIF;
	this.moteIF.registerListener(new TheftSerialMsg(), this);
    }

  public void sendPackets() {
      // We aren't sending anything, so just implement the interface.  
  }

  public void handleQueue(Queue<TheftSerialMsg> messages)
  {
	int red_to_dark = 0, red_to_light = 0, green_to_dark = 0, green_to_light = 0;
  
	while (!messages.isEmpty())
	{
		TheftSerialMsg msg = messages.remove();

		// Update our counters depending on who signalled
		if(msg.get_color() == MOTE_RED)
		{
		 if (msg.get_state() == STATE_DARK)
		 {
			red_to_dark++;
		 }
		 else
		 {
			red_to_light++;
		 }
		}
		else
		{
		if (msg.get_state() == STATE_DARK)
		 {
			green_to_dark++;
		 }
		 else
		 {
			green_to_light++;
		 }
		}
	}
	
	// Print out a nice happy message notifying us of the state change
		System.out.println("Total Red to Dark changes: " + red_to_dark);
		System.out.println("Total Red to Light changes: " + red_to_light);
		System.out.println("Total Green to Dark changes: " + green_to_dark);
		System.out.println("Total Green to Light changes: " + green_to_light);
  }

  public void messageReceived(int to, Message message) {
	messageQ.add((TheftSerialMsg)message);
  }
  
  private static void usage() {
    System.err.println("usage: TheftSerial [-comm <source>]");
  }
  
  public static void main(String[] args) throws Exception {
    String source = null;
	
	final ScheduledExecutorService worker = Executors.newSingleThreadScheduledExecutor();
	messageQ = new LinkedList<TheftSerialMsg>();
	Runnable task = new Runnable() {
		public void run() {
			handleQueue(messageQ);
		}
	};
	worker.schedule(task, 1, TimeUnit.MINUTES);
	
    if (args.length == 2) {
      if (!args[0].equals("-comm")) {
	usage();
	System.exit(1);
      }
      source = args[1];
    }
    else if (args.length != 0) {
      usage();
      System.exit(1);
    }
    
    PhoenixSource phoenix;
    
    if (source == null) {
      phoenix = BuildSource.makePhoenix(PrintStreamMessenger.err);
    }
    else {
      phoenix = BuildSource.makePhoenix(source, PrintStreamMessenger.err);
    }

    MoteIF mif = new MoteIF(phoenix);
    TheftSerial serial = new TheftSerial(mif);
    serial.sendPackets();
  }


}
