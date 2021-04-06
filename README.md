# AMR
Autonomous Mobile Robotics repository for saving my work on the splashtop remove computers

Github weblink:

https://github.com/LCAS/teaching/wiki/CMP3103M#week-3-computer-vision-with-ros-and-opencv

                                <--------Instructions for splashtop ------->

FIrstly, make sure to install all dependencies, possibly put these in a bash file to automate:

(For splashtop)Log in with user computing and password computing. In a terminal, run:

sudo apt update && sudo apt upgrade
sudo apt install ros-melodic-uol-cmp3103m ros-melodic-desktop

Then run to ensure that everything necessary for the workshop is installed:

sudo apt install ros-melodic-uol-cmp3103m

             <-------------change github remote url for ease of access --------------->
https://docs.github.com/en/github/authenticating-to-github/checking-for-existing-ssh-keys
https://docs.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
https://docs.github.com/en/github/getting-started-with-github/managing-remote-repositories

	                    <--Launch different simulations-->
Launching the Turtlebot simulator:

roslaunch uol_turtlebot_simulator simple.launch - launches the simplest version of the simulation

roslaunch uol_turtlebot_simulator turtlebot-rviz.launch - allows us to see what the robot sees using point clouds that subscribes to the laser sensor topic which is published to by the laser sensor.

Available at: https://github.com/LCAS/teaching/wiki/first-turtlebot-tutorial.commaps
			            <------------------->


             <-----------------------ROS Commandline commands and help----------------------->
                                    always use [Tab] and -h when working on the command line!
roscore
rosnode list – lists a number of nodes running on the system just by starting off the simulation
rostopic list -  lists the names of all active topics that are currently advertised in the system.
rosservice list
rqt_graph
rqt_image_view
Rostopic pub 
Rostopic info  
rosservice list  
Rosmsg list - displays a list of all predefined msg types
rossrv 

Try adding -h or --help to any command you want additonal information about(shows sub commands)

                                <-----------(For splashtop computers)--------> 


Note: There are a few issues with the lab computers' NVIDIA drivers and the simulator. It may regularly fail initially. Just press [Ctrl-C] in the terminal and try again. This is fixed in more recent Ubuntu versions, but we have to live with the slight annoyance for now, we're afraid.

Keyboard teleop: roslaunch uol_turtlebot_simulator keyop.launch
------->
rqt graph

rqt_graph - used to dislay the graph that is built by the active publisher - subscriber architecture

Nodes are elipses and they are connected to each other by a topic
------->

                              <---------------Week 2 notes---------------->
rostopic list - h: shows the optional commands for different things such as active publishers and sucscribers.

                              <------------Ros_book_line_follower from week 3 ------->

https://github.com/LCAS/teaching/wiki/CMP3103-Week-3

sudo apt update && sudo apt install ros-melodic-ros-book-line-follower
source /opt/ros/melodic/setup.bash
roslaunch ros_book_line_follower course.launch

Launch robot on yellow line simulation

-------------> changing pything scripts into executables or not
chmod a+x for giving a file permission to execute with ./"example.py"
chmod a-x for removing permission from a file for falling off a cliff
