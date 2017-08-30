# RosRobotNavigation
This ROS project was an attempt to implement a number of control laws which were introduced in the following papers:<br>
  - [Semi-autonomous Navigation of a Robotic Wheelchair]
  - [Corridor following by Mobile Robots Equipped with Panoramic Cameras]
  - [Wall-following controllers for sonar-based mobile robots]
  
Authors attempted to find a stable solution for an ideal environment. 
It is clear that there are some errors and limitations based on the properties they defined in their environmental layout. 
For instance, these papers have not mentioned or proposed any conditional control law for situations like crashing into a wall, 
so I tried to add a behaviour to the system in situations like that. I tried to fix it by adding a condition the system. 
Now if we are headed to a wall and the system detect which way has the longest empty space and robot rotates towards that way.<br>

In this project, I tried to implement what were possible in ROS envronment. For testing these control laws under different circumstances,
I designed the following envrionment to see how the robot behaves. <br> <br>
![World](https://github.com/salaee/RosRobotNavigation/blob/master/comp6912_project_world/world/autolab.png "World") <br> <br>
### How to run
Operating System: Ubuntu 14.04.1 LTS
Copy the folders in your project files(i.e. ~/catkin_ws/src/), and then you might need to do the following
```sh
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscd comp6912_project_world/launch
```
If above steps are done successfully, then: 

```sh
$ roslaunch comp6912_project_world all.launch
```
Now you should see the world envrionment being launched and the last step is to run one of the three scripts available in `comp6912_project`.
For instance: (in a new terminal)
```sh
$ rosrun comp6912_project pass_corridor.py
```

Happy coding :)

[Semi-autonomous Navigation of a Robotic Wheelchair]: <https://link.springer.com/article/10.1023/A:1016371922451>
[Wall-following controllers for sonar-based mobile robots]: <http://ieeexplore.ieee.org/document/657920>
[Corridor following by Mobile Robots Equipped with Panoramic Cameras]: <https://www.semanticscholar.org/paper/Corridor-following-by-Mobile-Robots-Equipped-with-TSAKIRIS-ARGYROS/73fec0fa5d96de73a27124b956e8cca3a9b7c9b2>
