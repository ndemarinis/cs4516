/*
 * TheftMsgWrapper.java
 * Nicholas DeMarinis
 * 23 April 2012
 */

import java.util.Date;

/* 
 * This is a simple class to store a message and its given timestamp,
 * just so we can keep them in a queue and determine if we need to
 * remove it.
 */

public class TheftMsgWrapper {

    private Date date;
    private TheftSerialMsg msg;

    public TheftMsgWrapper(TheftSerialMsg msg)
    {
	this.msg = msg;
	this.date = new Date();
    }

    public TheftSerialMsg getMsg()
    {
	return this.msg;
    }

    public Date getDate()
    {
	return this.date;
    }


}
