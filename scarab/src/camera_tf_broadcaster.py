#!/usr/bin/env python
import roslib
import rospy

import tf
from dynamixel_msgs.msg import JointState as JointStateDynamixel


def handle_camera_pose(msg,agent_name):
    br = tf.TransformBroadcaster()
    br.sendTransform((0,0,.1),
                    tf.transformations.quaternion_from_euler(0,msg.current_pos,0),
                    rospy.Time.now(),
                    "%s/camera_link" % agent_name,
                    "%s/base_link" % agent_name)


if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')
    agent_name = rospy.get_param('~agent')
    rospy.Subscriber('/%s/servo_tilt_controller/state' % agent_name,
                    JointStateDynamixel,
                    handle_camera_pose,
                    agent_name)
    rospy.spin()
