[![Build Status](https://travis-ci.org/inani47/terrapinavigator.svg?branch=master)](https://travis-ci.org/inani47/terrapinavigator)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
# terrapinavigator - Navigation and Mapping with TurtleBot 

## Overview
This project aims to implement a simulated TurtleBot capable of navigating and mapping. The robotic system aims to be capable of:
1. Navigating Unknown Environments and simultaneously mapping it.
2. Navigating a known environment from point A to point B.
Additionally, the robotic system will have a service be capable of taking photographs at any desired time.
The project will be implemented using Gazebo, Rviz and ROS (and its various packages and libraries.

Few applications of the projects include:
1. Navigation of known environments like offices and factories.
2. Explorer bot to aid rescue efforts during natural disasters.
3. Remote Survaillence 


## About the Developer

I am Pranav Inani. I am currently pursing my masters in Robotics at University of Maryland - College Park. I hold a Bachelors degree in Mechatronics from Mahatma Gandhi Instite of Technology, India. I have worked for a small start-up geared towards solving the self-driving car problem in India (a completely different challenge altogether). I wish to pursue a career in Robotics with a focus in Artificial Intelligence.
Few modest projects I was a part of:
1. Lane Marker Detection Using DBScan Clustering Algorithm.
2. AStar Path Planning Algorithm.
3. Design and Fabrication of Quadcopter with Integrated End-Effector.
4. Home automation Using Raspberry Pi and Arduino.

## Disclaimer

This software is released under the BSD-3 License.
```
BSD 3-Clause License

Copyright (c) 2017, Pranav Inani
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
## Dependencies

To run this program you need to have the following installed on your system:
* Ubuntu (Wily/Xenial) or Debian (Jessie)
* ROS Kinetic
* Gazebo 7.x (part of ros-kinetic-desktop-full package)
* Turtlebot simulation stack
* Gmapping
* map_server
To install ROS, use this [link](http://wiki.ros.org/kinetic/Installation)

To install turtlebot simulation stack. In a terminal:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
source /opt/ros/kinetic/setup.bash
```
To install gmapping, In a terminal:
```
sudo apt-get install ros-kinetic-slam-gmapping
```
To install Map_server, In a terminal:
```
 sudo apt-get install ros-kinetic-map-server
```



## SIP & Sprint Logs

Link to SIP Planning: [SIP Logs](https://docs.google.com/spreadsheets/d/1yglRR3HuQ96tQThB4AsiW9a2gjR8kYN-1Wcj_BuyqH0/edit?usp=sharing)
Link to Sprint Planning Notes: [Sprint Notes](https://docs.google.com/document/d/1rXK6foPKe-qIE33yUQAk5DwvEb2_a-keuWt45c4f9LM/edit)



## Build Steps
If you do not have a catkin workspace, in a new terminal:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/inani47/terrapinavigator.git
cd ..
catkin_make
```
If you wish to run this code in an existing catkin workspace:
```
cd ~/catkin_ws/
source devel/setup.bash
cd src/
git clone --recursive https://github.com/inani47/terrapinavigator.git
cd ..
catkin_make
```
## Running the Demo Using Launch File
After following the build instructions:

To run the demo, in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch terrapinavigator terrapinavigator.launch 
```
This will load the turtlebot in the gazebo world and wait for 15 seconds. Now to run gmapping and Rviz, in a new terminal:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch terrapinavigator demo.launch 
```

This will start the gmapping package and load rviz for visualization.
Note: You may close the gazebo window if your rig can't handle the load and continue to use rviz for visualization.

## Saving the Map
Once you are happy with the map created. To save a map, in a new terminal:
```
rosrun map_server map_saver -f <map_name>
```
To view the saved map. In a new terminal
```
eog <map_name>.pgm
```
## Running Rostest
To run rostest, in a new terminal:
```
cd ~/catkin_ws/build
make run_tests
```
## Known Issues/Bugs 
NONE

## API and other developer documentation

### Generating Doxygen Documentation

To install doxygen run the following command: 
```
sudo apt-get install doxygen
```

Now from your cloned directory, run the following command:

```
doxygen terpDocs
```

Doxygen files will be generated to /docs folder

To view them in a browser:
```
cd docs
cd html
firefox index.html
```



