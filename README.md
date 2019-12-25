# Obstacle_Avoidance_ROS

## install gazebo & ROS
 * [installation tutorial](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)

 * [installation tutorial](http://wiki.ros.org/ROS/Installation)
 * Reconmend: gazebo7.0+ROS kinetic+ubuntu16.04

## install some gazebo models
```
cd ~/.gazebo/
mkdir  models
cd models/
wget http://file.ncnynl.com/ros/gazebo_models.txt
wget -i gazebo_models.txt
ls model.tar.g* | xargs -n1 tar xzvf   
```

## Cloning the project
   
   Create a new directory in ```<catkin_workspace>/src/testbot_description``` and clone all of project files to that folder. 
   
   sample commands:
   
   1. ```mkdir ~/catkin_ws/src/testbot_description```
   2. ```cd ~/catkin_ws/src/testbot_description```
   3. ```git clone https://github.com/Jeret-Ljt/Obstacle_Avoidance_ROS.git```
   4. ```checkout maze_branch```
   5. ```cd ~/catkin_ws```
   6. ```catkin_make```
   7. ```echo "~/catkin_ws/devel/setup.bash" >> ~/.bashrc```
   8. ```source ~/.bashrc```

## Step by Step ROS Command Explanation

1. First you need to Launch the ROS node.
   ```
   roslaunch testbot_description testbot_gazebo.launch
   ```

   testbot_description is the package name that I gave. And testbot_gazebo.launch file can be found at /launch folder
   After executing this command, Gazebo will open like below.
   ![alt text](https://github.com/vibhuthasak/Obstacle_Avoidance_ROS/blob/master/1.png)

**Everthing looking great so far right ?** 

2. Now you need to run the python script that is created to listen sensor data from our virtual robot and move the robot.
   
   ```rosrun testbot_description sensor_data_listener_bug.py```
   

### Python Script Explanation in brief:
   
   The python script is available at Obstacle_Avoidance_ROS/scripts/sensor_data_listener.py path.
   
   The script is simply subscribes to ```/scan``` topic's ```sensor_msgs.msg.LaserScan``` message. The message is modified using    the ```LaserScanProcess()``` function.
   
   ```rospy.Subscriber("scan", sensor_msgs.msg.LaserScan , LaserScanProcess)```
   
   
   And it is publishing commands to ```/cmd_vel``` topic and the message type is Twist.
   
   ```pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)```
   
   If you echo the ```/cmd_vel``` topic using following command
   
   ```rostopic echo cmd_vel```
   
   You will see it is printing __Linear__ and __Angular__ velocities.
   
**DONE**

