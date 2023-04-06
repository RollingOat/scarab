#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import numpy as np
import tf.transformations as tft
import sensor_msgs.msg

class MoveToGoal(object):
    def __init__(self, name):
        rospy.Subscriber("move_base_simple/goal", geometry_msgs.msg.PoseStamped, self.goal_cb)
        rospy.Subscriber(name+"/pose", geometry_msgs.msg.PoseStamped, self.pose_cb)
        self._cmd_pub = rospy.Publisher(name+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self._goal_available = False


    def pose_cb(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        quat = tft.numpy.array([data.pose.orientation.x,
                                data.pose.orientation.y,
                                data.pose.orientation.z,
                                data.pose.orientation.w])
        eul = tft.euler_from_quaternion(quat)
        angle = eul[-1]  # angle around z from quaternion

        if self._goal_available:
            cmd = geometry_msgs.msg.Twist()
            ### Your code goes here ###


            self._cmd_pub.publish(cmd)
            # rospy.loginfo("publishing vel_cmd: %s" % cmd)


    def goal_cb(self, data):
        self._goal_x = data.pose.position.x
        self._goal_y = data.pose.position.y
        rospy.loginfo("obtained goal! (%s, %s)", self._goal_x, self._goal_y)
        self._goal_available = True



def run(name):
    rospy.init_node('vel_from_pose')
    mtg = MoveToGoal(name)
    rospy.spin()

if __name__ == '__main__':
    name = 'scarab45'
    run(name)
