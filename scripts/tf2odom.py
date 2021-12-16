#!/usr/bin/env python

"""
MIT License

Copyright (c) 2021 Mohamed Abdelkader, mohamedashraf123@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

"""
This script implements some test cases for trajectory prediction
* Test 1:
*   Send an initial  

* Test 2:
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
import tf

class TF2Odom:
  def __init__(self):
    self._parent_frame = rospy.get_param("~parent_frame", "parent_odom")
    self._child_frame = rospy.get_param("~child_frame", "child_odom")
    self._child_odom_topic = rospy.get_param("~child_odom_topic", "mavros/local_position/odom")
    self._transformed_odom_topic = rospy.get_param("~transformed_odom_topic", "target/odom")

    self._listener = tf.TransformListener()

    # ----------------------- ROS Subscribers ----------------------#
    rospy.Subscriber(self._child_odom_topic, Odometry, self.chiledOdomCallback, queue_size=10)

    # ----------------------- ROS Publishers------------------------#
    self._odom_pub = rospy.Publisher(self._transformed_odom_topic, Odometry, queue_size=10)

  def chiledOdomCallback(self, msg):
    """
    Odom callback of the target, in the child frame
    """
    child_frame = msg.child_frame_id
    vel_child_msg = Vector3Stamped()
    vel_child_msg.header.frame_id = child_frame
    vel_child_msg.header.stamp = msg.header.stamp
    vel_child_msg.vector.x = msg.twist.twist.linear.x
    vel_child_msg.vector.y = msg.twist.twist.linear.y
    vel_child_msg.vector.z = msg.twist.twist.linear.z

    try:
      (trans,rot) = self._listener.lookupTransform(self._parent_frame, child_frame, rospy.Time(0))
      vel_parent_msg = self._listener.transformVector3(self._parent_frame, vel_child_msg)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      return

    odom_msg = Odometry()
    odom_msg.header.frame_id = self._parent_frame
    odom_msg.header.stamp = msg.header.stamp

    odom_msg.pose.pose.position.x = trans[0]
    odom_msg.pose.pose.position.y = trans[1]
    odom_msg.pose.pose.position.z = trans[2]

    odom_msg.pose.pose.orientation.x = rot[0]
    odom_msg.pose.pose.orientation.y = rot[1]
    odom_msg.pose.pose.orientation.z = rot[2]
    odom_msg.pose.pose.orientation.w = rot[3]

    odom_msg.twist.twist.linear.x = vel_parent_msg.vector.x
    odom_msg.twist.twist.linear.y = vel_parent_msg.vector.y
    odom_msg.twist.twist.linear.z = vel_parent_msg.vector.z

    self._odom_pub.publish(odom_msg)


  def loop(self):

    while not rospy.is_shutdown():
      try:
        (trans,rot) = self._listener.lookupTransform(self._parent_frame, self._child_frame, rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

      odom_msg = Odometry()
      odom_msg.header.frame_id = self._parent_frame
      odom_msg.header.stamp = rospy.Time.now()
      odom_msg.pose.pose.position.x = trans[0]
      odom_msg.pose.pose.position.y = trans[1]
      odom_msg.pose.pose.position.z = trans[2]
      odom_msg.pose.pose.orientation.x = rot[0]
      odom_msg.pose.pose.orientation.y = rot[1]
      odom_msg.pose.pose.orientation.z = rot[2]
      odom_msg.pose.pose.orientation.w = rot[3]

if __name__ == "__main__":
    rospy.init_node('tf2odom_node', anonymous=True)
    rospy.loginfo("tf2odom_node is started.")

    t2odom = TF2Odom()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down tf2odom_node")

