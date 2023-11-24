/////Setup////
("Setup.scd").loadRelative

d.keys

(
OSCdef(\opencv, {|msg|
	msg.postln;
}, \cv, recvPort:12000)
)









//[nº1 circle, x y size]//

//[x> 200] --> Agudo
//[x< 200] --> Grave
//////Ejemplo fácil
(
SynthDef(\sinfb, {
	arg freq = 440, atk = 0.01, sus = 0, rel = 1, fb = 0, amp = 0.3, out = 0, pan=0;
	var sig, env;
	env = EnvGen.ar(Env.linen(atk,sus,rel),1,1,0,1,2);
	sig = SinOscFB.ar(freq,fb,1);
	sig = sig*env;
	Out.ar(out,Pan2.ar(sig,pan,amp));
}).add;
);

Synth(\sinfb)

a = [[1, 305, 10, 20],[100, 300, 5, 20]]


//Referencias. Primera grabación de piano

(
a.collect({
	|i|

	Synth(\sinfb, [\freq, i[1], \fb, i[3].linlin(0.1, 20, 0.1, 2), \sus, i[2]]);

})
)

////Ejemplo ordenando la data y una routine


(
~sinSeq = r{
	loop{
	~circulitos = rrand(10, 100);
	~data = Array.fill2D(~circulitos, 3, { arg r, c; rrand(100, 2000)});

	~minmax = [[], [], []];
	~min = [[], [], []];
	~max = [[], [], []];

	~data.collect{ |i, j|
		~minmax[0] = ~minmax[0].insert(j, i[0]);
	};

	~data.collect{ |i, j|
		~minmax[1] = ~minmax[1].insert(j, i[1]);
	};


	~data.collect{ |i, j|
		~minmax[2] = ~minmax[2].insert(j, i[2]);
	};

	~min[0] = ~minmax[0].sort[~minmax[0].minIndex];
	~max[0] = ~minmax[0].sort[~minmax[0].maxIndex];

	~min[1] = ~minmax[1].sort[~minmax[1].minIndex];
	~max[1] = ~minmax[1].sort[~minmax[1].maxIndex];

	~min[2] = ~minmax[2].sort[~minmax[2].minIndex];
	~max[2] = ~minmax[2].sort[~minmax[2].maxIndex];

	wait(0.1);

	~data.collect({
		|i, j|
			Synth(\sinfb, [\freq, i[0].linlin(~min[0], ~max[0], 30, 35).round().midicps, \fb, i[1].linlin(~min[1], ~max[1], 0.1, 0.8), \sus, i[2].linlin(~min[2], ~max[2], 0.1, 2), \amp, i[0].linexp(~min[0], ~max[0], 0.01, 0.1), \atk, 1, \rel, rrand(1/16, 1/2)]);
	});
	wait(1);
}}
)

b = ~sinSeq.play
x.stop

//Misma seq anterior con Sintesis granular --> Con SynthDef Tgrains

(
SynthDef(\tgrain, {
	var clk, pos, pan, sig, env, dur, rate;
	dur = 4 / \freq.kr(1, 1);
    rate = Dseq([10, 1, 1, 0.5, 0.5, 0.2, 0.1], inf);
	env = EnvGen.ar(Env.linen(\atk.kr(0),\sus.kr(0),\rel.kr(1)),1,1,0,1,2);
	dur = 12 / \grains.kr(1);
	clk = Impulse.ar(\grains.kr(1));
	pos = TRand.kr(0, BufDur.kr(\buf.kr(1)), clk);
	pan = WhiteNoise.kr(0.6);
	sig = TGrains.ar(2, clk, \buf.kr(1), rate, pos, dur, pan, \amp.kr(0.5));
	sig = sig * env;
	Out.ar(\out.kr(0), sig)
	}).add;
)

(
~grainSeq = r{
	loop{

	~circulitos = rrand(1, 100);
	~circulitos.postln;
		~data = Array.fill2D(~circulitos, 3, { arg r, c; rrand(100, 2000)});

	~minmax = [[], [], []];
	~min = [[], [], []];
	~max = [[], [], []];

	~data.collect{ |i, j|
		~minmax[0] = ~minmax[0].insert(j, i[0]);
	};

	~data.collect{ |i, j|
		~minmax[1] = ~minmax[1].insert(j, i[1]);
	};


	~data.collect{ |i, j|
		~minmax[2] = ~minmax[2].insert(j, i[2]);
	};

	~min[0] = ~minmax[0].sort[~minmax[0].minIndex];
	~max[0] = ~minmax[0].sort[~minmax[0].maxIndex];
	~min[1] = ~minmax[1].sort[~minmax[1].minIndex];
	~max[1] = ~minmax[1].sort[~minmax[1].maxIndex];

	~min[2] = ~minmax[2].sort[~minmax[2].minIndex];
	~max[2] = ~minmax[2].sort[~minmax[2].maxIndex];

	wait(0.1);

	~data.collect({
		|i, j|
			Synth(\tgrain, [
				\buf, d["pianola"][0],
				\freq, i[0].linlin(~min[0], ~max[0], 2, 200).round(),
				\grains, i[1].linlin(~min[1], ~max[1], 0.1, 10),
				\amp, i[0].linexp(~min[0],~max[0], 0.1, 0.3),
				\sus, i[2].linlin(~min[2], ~max[2], 0, 2),
				\atk, exprand(0,1.0),
				\rel, rrand(0.5,2)]);
	});
	wait(5);
}}
)

b = ~sinSeq.play
b.stop
x = ~grainSeq.play
x.stop
z = ~grainSeq.play
z.stop

/////Versión usando Demand Ugens
s.boot;
~path = PathName(thisProcess.nowExecutingPath).parentPath;
b = Buffer.readChannel(s, "/home/fede/Nextcloud/CCEBA 2023/CCEBA_codes y material/etudes/soundsytem/samples/pianola/PianolaRara.wav", channels: 0)

(
{
    var trate, dur, rate;
    trate = MouseY.kr(2,200,1);
    dur = 4 / trate;
    rate = Dseq([10, 1, 1, 0.5, 0.5, 0.2, 0.1], inf);
	TGrains.ar(2, Impulse.ar(trate), d["pianola"][0], rate, MouseX.kr(0,BufDur.kr(d["pianola"][1])), dur, Dseq([-1, 1], inf), 0.1, 2);
}.scope(zoom: 4);
)

(
{
    var trate, dur, clk, pos, pan;
    trate = MouseY.kr(8,120,1);
    dur = 12 / trate;
    clk = Impulse.kr(trate);
    pos = MouseX.kr(0,BufDur.kr(b)) + TRand.kr(0, 0.01, clk);
    pan = WhiteNoise.kr(0.6);
    TGrains.ar(2, clk, b, 1, pos, dur, pan, 0.1);
}.scope(zoom: 4);
)

// 4 channels
(
{
    var trate, dur, clk, pos, pan;
    trate = MouseY.kr(8,120,1);
    dur = 12 / trate;
    clk = Impulse.kr(trate);
    pos = MouseX.kr(0,BufDur.kr(b)) + TRand.kr(0, 0.01, clk);
    pan = WhiteNoise.kr(0.6);
    TGrains.ar(4, clk, b, 1, pos, dur, pan, 0.1);
}.scope(4, zoom: 4);
)

(
{
    var trate, dur, clk, pos, pan;
    trate = MouseY.kr(8,120,1);
    dur = 4 / trate;
    clk = Dust.kr(trate);
    pos = MouseX.kr(0,BufDur.kr(b)) + TRand.kr(0, 0.01, clk);
    pan = WhiteNoise.kr(0.6);
    TGrains.ar(2, clk, b, 1, pos, dur, pan, 0.1);
}.scope(zoom: 4);
)

(
{
    var trate, dur, clk, pos, pan;
    trate = LinExp.kr(LFTri.kr(MouseY.kr(0.1,2,1)),-1,1,8,120);
    dur = 12 / trate;
    clk = Impulse.ar(trate);
    pos = MouseX.kr(0,BufDur.kr(b));
    pan = WhiteNoise.kr(0.6);
    TGrains.ar(2, clk, b, 1, pos, dur, pan, 0.1);
}.scope(zoom: 4);
)

(
{
    var trate, dur, clk, pos, pan;
    trate = 12;
    dur = MouseY.kr(0.2,24,1) / trate;
    clk = Impulse.kr(trate);
    pos = MouseX.kr(0,BufDur.kr(b)) + TRand.kr(0, 0.01, clk);
    pan = WhiteNoise.kr(0.6);
    TGrains.ar(2, clk, b, 1, pos, dur, pan, 0.1);
}.scope(zoom: 4);
)

(
{
    var trate, dur, clk, pos, pan;
    trate = 100;
    dur = 8 / trate;
    clk = Impulse.kr(trate);
    pos = Integrator.kr(BrownNoise.kr(0.001));
    pan = WhiteNoise.kr(0.6);
    TGrains.ar(2, clk, b, 1, pos, dur, pan, 0.1);
}.scope(zoom: 4);
)

(
{
    var trate, dur, clk, pos, pan;
    trate = MouseY.kr(1,400,1);
    dur = 8 / trate;
    clk = Impulse.kr(trate);
    pos = MouseX.kr(0,BufDur.kr(b));
    pan = WhiteNoise.kr(0.8);
    TGrains.ar(2, clk, b, 2 ** WhiteNoise.kr(2), pos, dur, pan, 0.1);
}.scope(zoom: 4);
)

(
{
    var trate, dur;
    trate = MouseY.kr(2,120,1);
    dur = 1.2 / trate;
    TGrains.ar(2, Impulse.ar(trate), b, (1.2 ** WhiteNoise.kr(3).round(1)), MouseX.kr(0,BufDur.kr(b)), dur, WhiteNoise.kr(0.6), 0.1);
}.scope(zoom: 4);
)

// demand ugens as inputs
(
{
    var trate, dur, z, d;
    trate = MouseX.kr(1, 100, 1);
    d = { Dwhite(0.1, 0.2, 1) };
    z = {
        Drand([Dgeom(0.1, 1 + d.value, Diwhite(20, 40)), Dgeom(1, 1 - d.value, Diwhite(20, 40))])
    };
    TGrains.ar(2,
        Impulse.ar(trate),
        bufnum: 10,
        rate: Dseq([1, 1, z.value, 0.5, 0.5, 0.2, 0.1, 0.1, 0.1, 0.1], inf) * 2 + 1,
        centerPos: Dseq(z.dup(8), inf),
        dur: Dseq([1, d.value, 1, z.value, 0.5, 0.5, 0.1, z.value] * 2, inf) / trate,
        pan: Dseq([1, 1, 1, 0.5, 0.2, 0.1, 0, 0, 0], inf) * 2 - 1,
        amp: Dseq([1, 0, z.value, 0, 2, 1.0, 1, 0.1, 0.1], inf)
    );
}.scope(zoom: 4);
)

b.free





///Data recibida
(
OSCdef(\opencv, {|msg|
	~grains = msg[1].postln
}, \cv, recvPort:12000)
)

~grains



(
~grains.collect({
	|i|

	Synth(\tgrain,
	[
		\buf, d["pianola"][0],
		\freq, 1,
		\grains, ~grains * 10,
		\amp, rrand(0.1,0.8),
		\sus, rrand(0.1,1),
		\atk, 0,
		\rel, rrand(0.5,2)]
)}
)
)

(
SynthDef(\tgrain, {
	var clk, pos, pan, sig, env, dur, rate;
	dur = 4 / \freq.kr(1, 1);
    rate = Dseq([10, 1, 1, 0.5, 0.5, 0.2, 0.1], inf);
	env = EnvGen.ar(Env.linen(\atk.kr(0),\sus.kr(0),\rel.kr(1)),1,1,0,1,2);
	dur = 12 / \grains.kr(1);
	clk = Impulse.ar(\grains.kr(1));
	pos = TRand.kr(0, BufDur.kr(\buf.kr(1)), clk);
	pan = WhiteNoise.kr(0.6);
	sig = TGrains.ar(2, clk, \buf.kr(1), rate, pos, dur, pan, \amp.kr(0.5));
	sig = sig * env;
	Out.ar(\out.kr(0), sig)
	}).add;
)




///////GUI///////
~central = Window("Data", Rect(200, 200, 255, 100))






