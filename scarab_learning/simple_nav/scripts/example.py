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
        rospy.Subscriber(name+"/scan_throttle", sensor_msgs.msg.LaserScan, self.laser_cb)
        self._cmd_pub = rospy.Publisher(name+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self._goal_available = False
        self._obstacle_onroute = False


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
            dist = np.sqrt((self._goal_x-x)*(self._goal_x-x)+(self._goal_y-y)*(self._goal_y-y))
            rospy.loginfo("distance to goal: %s", dist)
            if self.reached_goal(dist):
                rospy.loginfo("Goal reached!")
                self._goal_available = False
                cmd.linear.x = 0
                cmd.angular.z = 0
                self._cmd_pub.publish(cmd)
            else:
                angle_g = np.arctan2(self._goal_y-y,self._goal_x-x)
                angle_diff = angle_g - angle
                # rospy.loginfo("angle_diff: %s", angle_diff)
                cmd.angular.z = np.clip(angle_diff, -1.0, 1.0)
                # rospy.loginfo("commanded angular z: %s", cmd.angular.z)
                if abs(angle_diff) > 0.05:  # stop and turn if angle greater than threshold
                    cmd.linear.x = 0
                else:
                    cmd.linear.x = 0.2
                self._cmd_pub.publish(cmd)
                # rospy.loginfo("publishing vel_cmd: %s" % cmd)

    def laser_cb(self, data):
        return



    def goal_cb(self, data):
        self._goal_x = data.pose.position.x
        self._goal_y = data.pose.position.y
        rospy.loginfo("obtained goal! (%s, %s)", self._goal_x, self._goal_y)
        self._goal_available = True

    def reached_goal(self, dist):
        if dist < 0.05:
            return True
        else:
            return False



def run(name):
    rospy.init_node('vel_from_pose')
    mtg = MoveToGoal(name)
    rospy.spin()

if __name__ == '__main__':
    name = 'scarab45'
    run(name)
