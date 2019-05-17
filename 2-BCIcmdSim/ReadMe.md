# Compile and Execution
## Compile
To compile the code you must use make in the main folder:
```
$ make
```
## Execution
You can find the executable in the bin folder, to execute it from the main folder use:
```
$ ./bin/sim0.o
```
## Commands
When the code is executing press:
```
[ #n ] set p300 players probability, with #n integer from 0 to MAX_P300
[ s ] send p300 
[ d ] send motor imagery right command 
[ a ] send motor imagery left command 
[ m ] switch p300 setter mode
[ p ] robot moves 
[ h ] help 
```
# Elements of the code
## Parameters Modification
In the /include directory you can find a file that contains some fixed parameters for the problem indipendent from the code execution. In the /src directory the _init_*(...) methods initialize the run time parameters;

## Visual Elements
When you execute the code one window appears. The red circles represent the players that can be selected by p300, between square brackets there are the number to press for select the player and modify the probability, the nearest number, that will be send pressing [s] to generate the p300. The portion of the screen in grey represent the robots fild of view. The dark arrow represent the current direction of the robot while the grey one represent the target direction that the robot tries to reach. The green paint correspond to the probability of the direction.
