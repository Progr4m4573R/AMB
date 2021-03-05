# AMB
Autonomous Mobile Robotics repository for saving my work on the splashtop remove computers

Github weblink:

https://github.com/LCAS/teaching/wiki/CMP3103M#week-3-computer-vision-with-ros-and-opencv

                                <--------Instructions ------->

FIrstly, make sure to install all dependencies, possibly put these in a bash file to automate:

(For splashtop)Log in with user computing and password computing. In a terminal, run:

sudo apt update && sudo apt upgrade
sudo apt install ros-melodic-uol-cmp3103m ros-melodic-desktop

Then run to ensure that everything necessary for the workshop is installed:

sudo apt install ros-melodic-uol-cmp3103m


Launching the Turtlebot simulator:

roslaunch uol_turtlebot_simulator simple.launch


------->
(For splashtop computers)

Note: There are a few issues with the lab computers' NVIDIA drivers and the simulator. It may regularly fail initially. Just press [Ctrl-C] in the terminal and try again. This is fixed in more recent Ubuntu versions, but we have to live with the slight annoyance for now, we're afraid.

Keyboard teleop: roslaunch uol_turtlebot_simulator keyop.launch
------->
rqt graph

rqt_graph - used to dislay the graph that is built by the active publisher - subscriber architecture

Nodes are elipses and they are connected to each other by a topic
------->
