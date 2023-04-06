#!/usr/bin/env python
import rospy
import tf
import math

import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from nav_msgs.msg import Odometry

import numpy as np
import copy

class OdomToTF(object):
  def __init__(self):
    self.br = tf2_ros.TransformBroadcaster()
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.tfros = tf.TransformerROS()

    self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
    self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')

    self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)

  def odom_cb(self,msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = self.odom_frame_id
    t.child_frame_id = self.base_frame_id
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(t)

    #rospy.loginfo("Pub static transform with name %s and parent link: %s"%(t.child_frame_id,t.header.frame_id))

def main():
  rospy.init_node('odom_to_tf')

  ftb = OdomToTF()

  rospy.spin()
  return 0

if __name__ == '__main__':
  main()
