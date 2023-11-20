//Boilerplate code for basic live coding functionality

(
//increase number of buffers the server has access to for loading samples
s.options.numBuffers = 1024 * 16;
//increase the memory available to the server
s.options.memSize = 8192 * 64;
//boot the server
s.boot;
//display the oscilloscope
s.scope;
Task({
	var path, subfolders;
	3.wait;
	d = Dictionary.new;
	path = PathName(thisProcess.nowExecutingPath).parentPath;
	d.add(\foldernames -> PathName(path ++ "/samples/").entries);
	for (0, d[\foldernames].size-1,
	{arg i; d.add(d[\foldernames][i].folderName -> d[\foldernames][i].entries.collect({
	arg sf;
	Buffer.readChannel(s,sf.fullPath, channels: 0);
	});
	)});

	s.sync;

	// ("SynthDefs.scd").loadRelative;
	// //loads snippets from setup folder
	// ("Snippets.scd").loadRelative;
	// //wait, because otherwise it won't work for some reason
	3.wait;
	//activate StageLimiter - Part of the BatLib quark
	StageLimiter.activate;
	"Setup done!".postln;
}).start;
)
