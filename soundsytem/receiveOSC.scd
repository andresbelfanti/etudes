(
OSCdef(\opencv, {|msg|
	msg[0].postln;
}, \cv, recvPort:12000)
)

//[nº1 circle, x y size]//

//[x> 200] --> Agudo
//[x< 200] --> Grave

a = [[1, 100, 200, 20],[2, 200, 100, 20]]

//Referencias. Primera grabación de piano