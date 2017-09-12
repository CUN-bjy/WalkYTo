# WalkYTo
### :Walk Yourself, Toddler!

Hanyang University, Robotics, CPS & AI LAB

Undergraduate Researcher Program
>Junyeob Beak(wnsdlqjtm@naver.com), Hyeonwoo Park(ppaa1135@naver.com), Seunghwan Yu(sinanju06@naver.com)


#### Nessesary for user to launch this project.
- "gazebo_ros_pkgs" for graphic simulation -- ros package
- "neat-python" for genetic network model -- python module
- "tensorflow" for DQN implementation -- python module


### gazebo simulation system
![Image](https://github.com/CUN-bjy/WalkYTo/blob/master/system.jpg?raw=true)
*with bell_and_faraday launch file*



#### Conmmend for launch
<pre>roslaunch walkyto bell_and_faraday_world.launch</pre>

if you want to run gazebo simulator, with the argment `GUI:=true`
<pre>roslaunch walkyto bell_and_faraday_world.launch GUI:=true</pre>

#### Code Abstract
