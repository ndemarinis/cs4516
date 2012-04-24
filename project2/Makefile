COMPONENT=AntiTheftAppC

CC420_CHANNEL=15
CFLAGS = -DCC2420_DEF_RFPower=3

BUILD_EXTRA_DEPS += TheftSerial.class
CLEAN_EXTRA = *.class TheftSerialMsg.java

CFLAGS += -I$(TOSDIR)/lib/T2Hack


TheftSerial.class: $(wildcard *.java) TheftSerialMsg.java
	javac -target 1.4 -source 1.4 *.java

TheftSerialMsg.java:
	mig java -target=null $(CFLAGS) -java-classname=TheftSerialMsg AntiTheft.h theft_serial -o $@


include $(MAKERULES)