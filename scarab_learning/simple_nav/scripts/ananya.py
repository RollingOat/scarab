#!/usr/bin/env python

import rospy	
import functools
import geometry_msgs.msg
import numpy as np
import tf.transformations as tft
import sensor_msgs.msg
from std_msgs.msg import Float64

class MoveToGoal(object):
    def __init__(self, name):
        rospy.Subscriber("move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.goal_cb)
        rospy.Subscriber(name+"/pose", geometry_msgs.msg.PoseStamped, self.pose_cb)
	rospy.Subscriber(name+"/scan_throttle",sensor_msgs.msg.LaserScan,self.laser_cb)
        self._cmd_pub = rospy.Publisher(name+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
	#self._pub=rospy.Publisher
        self._goal_available = False
	self._angle_b=0
	self._obstacle = False
	self._obstacle_p=0
	self._t=0.0
	self._first_o=True	
	self._time_now=0.0
	#self._temporary_goal=False
	#self._t_goal=0
	self._extreme=False

    def pose_cb(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
  	quat = tft.numpy.array([data.pose.orientation.x,
                                data.pose.orientation.y,
                                data.pose.orientation.z,
                                data.pose.orientation.w])
        eul = tft.euler_from_quaternion(quat)
        self._angle = eul[-1]
        if self._goal_available:
            cmd = geometry_msgs.msg.Twist()
	    dist=np.sqrt((self._goal_x-x)*(self._goal_x-x)+(self._goal_y-y)*(self._goal_y-y))
	    #rospy.loginfo("obtained distance: %s", dist)
	    if self.goal_reached(dist):
	    	self._goal_available=False
	    	cmd.linear.x=0
		cmd.angular.z=0
		rospy.loginfo("goal reached")
 		self._cmd_pub.publish(cmd)
	    else:
	      angle_a=np.arctan2((self._goal_y-y),(self._goal_x-x))
	      self._angle_b=angle_a-self._angle
	      cmd.angular.z=np.clip(0.5*self._angle_b,-0.5,0.5)
	      j=self._t+1
	      k=self._t+3
    	      #rospy.loginfo("time stamp %s", self._t)
	      #rospy.loginfo(self._first_o)
   	      #rospy.loginfo("time no %s", rospy.get_time())
	      if self._extreme:
		j=self._t+2  
	      if self._obstacle and dist>0.4:
		if cmd.linear.x>0 or abs(self._angle_b)<0.3:  			 
		  #j=self._t+1
		  #k=self._t+3
		  #rospy.loginfo("time stamp %s", self._t)
		  #rospy.loginfo(self._first_o)
		  #rospy.loginfo("time no %s", rospy.get_time())
		  while self._time_now<=j:
		    rospy.loginfo("avoiding obstacle")
		    #cmd.linear.x=0.2
		    cmd.linear.x=0.0
		    rospy.loginfo("time stamp %s", self._t)
		    rospy.loginfo("time now %s", self._time_now)
		    rospy.loginfo("linear %s", cmd.linear.x)
		    if self._obstacle_p==1: 
		 	 self._angle_b=0.5
		  	 rospy.loginfo("turning left")

	  	    else:	
		    	self._angle_b=-0.5
		     	rospy.loginfo("turning right")
		    cmd.angular.z=self._angle_b
		    rospy.loginfo("angular %s", cmd.angular.z)
		    self._cmd_pub.publish(cmd)
		    #rospy.loginfo("timestamp %s",self._t) 
		  while self._time_now>j and self._time_now<=k:
		    cmd.angular.z=0
		    rospy.loginfo("travelling forward")
		    cmd.linear.x=0.2
		    self._cmd_pub.publish(cmd)	
		else:
		  rospy.loginfo("turning only")
		  cmd.linear.x=0
	      else:
		rospy.loginfo("no obstacle")
	        if abs(self._angle_b)<0.2:
			rospy.loginfo("moving forward")
			#cmd.linear.x=np.clip(0.2*dist,-0.5,0.5)
			cmd.linear.x=0.2
			#cmd.angular.z=0
			#cmd.linear.x=0
	        else:
			cmd.linear.x=0
			rospy.loginfo("turning only")
			rospy.loginfo(self._angle_b)


	


            self._cmd_pub.publish(cmd)
            # rospy.loginfo("publishing vel_cmd: %s" % cmd)  	
    def goal_reached(self, dist):
	if dist<0.1:
		return True
	else: 
		return False

    def goal_cb(self, data):
        self._goal_x = data.pose.position.x
        self._goal_y = data.pose.position.y
        rospy.loginfo("obtained goal! (%s, %s)", self._goal_x, self._goal_y)
        self._goal_available = True

    def laser_cb(self, data1):
	ranges=np.array(data1.ranges)
	i=360
	self._time_now=data1.header.stamp.to_sec()
	self._obstacle=False
	self._extreme=False
	if self._obstacle:
	  self.first_o=False
	else:
	  self.first_o=True
	while i<=600:
	  if ranges[i]<=1 and ranges[i]>0.015:
		self._obstacle=True
		
		if i<=480:
			self._obstacle_p=1
		else:
			self._obstacle_p=0
		if ranges[i]<=0.5:
		  	self._extreme=True
		#rospy.sleep(2)
	  i+=1
	if self._obstacle:
	  rospy.loginfo("obstacle detected")
	  if self._first_o:
	  	self._t=data1.header.stamp.to_sec()
		rospy.loginfo("setting time stamp")
	else:
	  rospy.loginfo("obstacle not detected")
    
    ''' def callback(self):
	if self._obstacle:
	 timer=rospy.Timer(rospy.Duration(2), functools.partial(self.laser_cb,data, oneshot=True)'''
    
def run(name):
    rospy.init_node('vel_from_pose')
    mtg = MoveToGoal(name)
    rospy.spin()

if __name__ == '__main__':
    name = 'scarab45'
    run(name)
