:Begin:
:Function:      sload
:Pattern:       sload[fn_String, i_List]
:Arguments:     {fn, i}
:ArgumentTypes: {String, IntegerList}
:ReturnType:    Manual
:End:


:Evaluate: sload::usage = "data = sload[filename, {ne,ng,ns}] load data sweeps into mathematica workspace.
 ne, ng, and ns are the number of the experiment, the number of the series from this experiment and
 the number of the sweep from this series sweep, respectivly. 0 can be used as wildcard to select all
 sweeps.\nExamples: data = sload(\"abc.dat\",{1,5,0}) selects all sweeps from 5th series of first experiment; {0,0,0} selects
 all sweeps from file \"abc.dat\".\n
 The output ist a list of three elements, data[[1]] contains the 2-dim array of data samples, 
 data[[2]] contains the time axis in seconds, and data[[3]] contains the header information in serialized JSON string. 
 It can be converted with ImportString[data[[3]], \"JSON\"]. \n
 \nNOTE: If sweeps were sampled with different sampling rates, all data is converted to the
 least common multiple of the various sampling rates. (e.g. loading a 20kHz and a 25kHz sweep simultaneously, both sweeps are converted to 100kHz).
 \n\nCompiled on __DATE__"

:Evaluate:      sload::failed="Failed to load file"
