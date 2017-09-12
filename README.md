# WalkYTo
### :Walk Yourself, Toddler!

Hanyang University, Robotics, CPS & AI LAB

Undergraduate Researcher Program
>Junyeob Beak(wnsdlqjtm@naver.com), Hyeonwoo Park(ppaa1135@naver.com), Seunghwan Yu(sinanju06@naver.com)


#### Nessesary for user to launch this project.
- "gazebo_ros_pkgs" for graphic simulation -- ros package
- "neat-python" for genetic network model -- python module
- "tensorflow" for DQN implementation -- python module

---
### Gazebo Simulation System
![Image](https://github.com/CUN-bjy/WalkYTo/blob/master/system.jpg?raw=true)
*with bell_and_faraday launch file*



### Commend for launch
<pre>roslaunch walkyto bell_and_faraday_world.launch</pre>

if you want to run gazebo simulator, with the argment `GUI:=true`
<pre>roslaunch walkyto bell_and_faraday_world.launch GUI:=true</pre>

### Package Abstract
- `/launch` : has launch files to launch processes with certain parameters.
- `/models` : has our robot descriptions(bell & faraday).
- `/src` : has main sources.
	* `network_gen.py` : networks generator code. This process generate genes and save in the `/genes` directory. Then choose random 4 genes and give the genes to the simulator until all population is done.
	* `simulator.py` : When receive the genes from generator, simulate the network. During the simulating, this process communicate with gazebo. 
	* `/genes` : gene files generated from network generator.
	* `neat-checkpoint-*` : save generation files(pickle)
	*  Other files are for supporting above files. Please focus on the every first lines with 'import' code.
- `/srv` : has a service file for generator to communicate with simulators.
- `/worlds` : has world files. It includes informations to describe physics and simulation env. configuration.
