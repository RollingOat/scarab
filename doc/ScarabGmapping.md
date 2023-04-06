On your computer/laptop create a workspace with following packages and compile. 

```
mkdir -p ~/ws_scarab/src
cd ~/ws_scarab
catkin init
cd ~/ws_scarab/src
git clone https://github.com/KumarRobotics/scarab
catkin build
```

Run `roscore` and `rviz` on your laptop. Your laptop is assumed to be ROS_MASTER
```
tmux
roscore
roslaunch scarab rviz.launch agent:=scarab41
```

In a new terminal (Note: ROS_MASTER_URI and ROS_IP are automatically set to the SSH_CLIENT)
```
tmux
ssh scarab@192.168.131.41
roslaunch scarab navigation.launch
```

Publish a geometry_msgs/PoseStamped on /scarab41/move_base_simple/goal
If the robot does not move, your ROS_IP is not set. Use following

```
export ROS_IP=$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')
```
