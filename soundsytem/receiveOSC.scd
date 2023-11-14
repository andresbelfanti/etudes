(
OSCdef(\opencv, {|msg|
	msg[0].postln;
}, \cv, recvPort:12000)
)