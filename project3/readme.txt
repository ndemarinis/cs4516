To compile the program, first run

make telosb

After it has been compiled, it can be installed on the first mote with

make telosb reinstall,0

and on the second mote with

make telosb reinstall,1


All functionality specified by the project writeup has been impleted.  The LEDs on the motes light up as specified, with the Blue LED showing that a node is connected and the Red and Green LEDs showing the identify of the Near and Far nodes, which change as the locations of the nodes change.