# Compile and Execution
## Compile
To compile the code you must use cmake in the main folder:
```
$ make
```
## Execution
You can find the executable in the bin folder, to execute it from the main folder use:
```
$ ./bin/sim0.o
```
# Elements of the code
## Parameters Modification
If you watch the code, you can modify the size of the map modifing the method _init_map(...), the size and the parameters of the robot modifing _init_bot(...) and the simulated commands modifing _init_command(...).

## Visual Elements
When you execute the code, two windows will be open, the first one represents the maps and robots start position (Green circle), goal position setted up by p300 command (Small red circle), the actual goal position moved by motor-imagery commands (Big empty red circle), the path of the bot (Big empty grey circles) and the current position of the bot (Small grey circle). The second windows represent the commands received by a simulated BCI, the first text represet time, arrows represent the motor-imagery command with the straight arrow means no motor-imagery commands and the last text represent the position of the goal received from p300.

## Description
The execution is simple, as first the best move is calculated basing the calculus on the distance beetween new possible positions and goal and the robot moves in the best position. Then commands are checked, as first the code check if p300 sets a new goal position, if it is the new goal is setted and repeat the cycle. If there aren't new goal setted by p300 the code calculates a vector perpendicular to the direction of the best moves toward left, then an accumulator is checked, the accumulator try to limit the error, if the user commands the same direction for more time the degree of goal shift is increased until a threshold, representing the max degrees, is reached. Finally the goal is shifted by the accumulator multiplied to the left vector calculated before according to the direction of the motor-imagery command. At the end of the cycle there are a control that limit the robot to not make a 90° degree shift.
