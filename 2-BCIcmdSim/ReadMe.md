# Compile and Execution
## Compile
To compile the code you must use make in the main folder:
```
 make
```
## Execution
You can find the executable in the bin folder, to execute it from the main folder use:
```
$ ./bin/sim.o
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
[ r ] restar the simulation
[ q ] quit the simulation
[ h ] help 
```
# Elements of the code
## Parameters Modification
In the /include directory you can find a file that contains some fixed parameters for the problem indipendent from the code execution. In the /src directory the _init_*(...) methods initialize the run time parameters;

## Visual Elements
When you execute the code one window appears. The red circles represent the players that can be selected by p300, between square brackets there are the number to press for select the player and modify the probability, the nearest number, that will be send pressing [s] to generate the p300. The portion of the screen in grey represent the robots fild of view. The dark arrow represent the current direction of the robot while the grey one represent the target direction that the robot tries to reach. The green paint correspond to the probability of the direction.

## Description
This is a simulation to command a robot with two tipe of commands, the first one derive from the P300 wave, this command is sent pressing [s] and send the probability and the position of the red circle, the probabilities are writed inside of the circles with numbers that represent the people, this numbers is used for setting the probability. The second type of commands derive from motor imagery, the command is setted pressind [d] for the right commands, [a] for the left commands. Once send the command the robot moves pressing [p]. 
The P300's commands would be relative to a flash on a person's face that tell us the probability that who uses the BCI want to go near the selected person. So using the P300's output, if we have N people in front of the robot, we obtain N probability each one relative to one person and their positions. The command produces a Gaussian with a goal-dependent standard deviation and centered in the direction of the goal, the entire distribution is weighted by the probability of the people when we press [s].

Let D<sub>t</sub>  the probability distribution derived from the past command, let P<sub>BCI</sub>(n) the probability returned by the n-th person selected, let Deg(n) the direction relative to the n-th person, let P<sub>G</sub>(n) the probability that the goal n-th person is selected, the next probability distribution D<sub>t+1</sub> is obtained using:

 D<sub>t+1</sub> = D<sub>t</sub> * E<sub>P300_PAST</sub> + ( 1 - E<sub>P300_PAST</sub> )*`Σ`<sub>n=1</sub><sup>N</sup> [ N( Deg( ( n ) , STDDEV<sub>P300</sub> * (1 - P<sub>G</sub>( n ) )) * P<sub>BCI</sub>( n )] 

Where STDDEV<sub>P300</sub> is a prefixed constant and E<sub>P300_PAST</sub> represent how much the past commands affect the distribution. The P<sub>G</sub>(n) is calculated by the following formula:

 P<sub>G</sub>(n) = { e<sup>-C<sub>G</sub>(`ε`<sub>S->U</sub>)</sup>) `∫`<sub>U->G</sub> e<sup>-C<sub>G</sub>(`ε`)<sub>U->G</sub>)</sup>} /{`∫`<sub>S->G</sub> e<sup>-C<sub>G</sub>(`ε`<sub>S->G</sub>)</sup> }
 
 Where `ε`<sub>S->U</sub> represent the trajectory from the robot's start configuration S to the next configuration U, `ε`<sub>S->G</sub> represent the trajectory from the robot's start configuration S to the goal configuration G, `ε`<sub>U->G</sub> represent the trajectory from the robot's next configuration U to the goal configuration G and C<sub>G</sub>(`ε`<sub>X->Y</sub>) a cost function based on the distance between two points using the trajectory `ε`<sub>X->Y</sub>. 
 
 The motor imagery commands tell us if the user want to go to left or to right otherwise the user send nothing. This commands generate two new Gaussian, the first one represent the relative direction of the command so it has fixed standard deviation STDDEV<sub>MI</sub> and mean centered 90 degrees to left or to right of the current direction. The second Gaussian is to maintain the direction that the robot is already following, this Gaussian has standard deviation STDDEV<sub>STAY</sub> and it is centered on the current direction dir<sub>current</sub> of the robot. So for a motor imagery command we have:
 
  D<sub>t+1</sub> = D<sub>t</sub> * E<sub>MI_PAST</sub> + ( 1- E<sub>MI_PAST</sub> - E<sub>STAY</sub>)N( dir<sub>current</sub> `±` DEGREE<sub>MI</sub> , STDDEV<sub>MI</sub>) + E<sub>STAY</sub>* N( dir<sub>target</sub> , STDDEV<sub>STAY</sub>) 
 
 Where E<sub>STAY</sub> is a costant represent how many the target direction is important, this Gaussian expands the bell of the previous one so we can see it as a uncertainty around the previous target direction. The DEGREE<sub>MI</sub> parameter represents how many degrees the user wants the robot turn to a given direction using a motor imagery command.
 
 In this simulation the robot can make a move when the user press [p] so it calculate the direction with maximum probability, it change is direction of one degree to the maximum and makes one move, if it already is in the maximum probability direction it goes straight of one move for each [p] pressed.

 When the user does not send command we can assume that the current target direction is the right direction so while the robot is moving, a new Gaussian is generated on the target direction. Then the D<sub>t+1</sub> becomes:

 D<sub>t+1</sub> = D<sub>t</sub> * E<sub>MOVE_PAST</sub> + (1-E<sub>MOVE_PAST</sub> )N( dir<sub>target</sub> , STDDEV<sub>MOVE</sub>)

Making this if we have motor imagery commands one after a lot of time to the next the result will be different then if the commands are sequential. If the user uses motor imagery commands nearly each others we have the certainty that he/she want change the direction while if they are distance, between two commands the robot make the current target more stronger so when the second comes, it has less strength. This work only if a single motor imagery command don't change the target direction and the motor imagery commands are to the same direction.