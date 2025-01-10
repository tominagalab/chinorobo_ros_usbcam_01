#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from chinorobo_ros_usbcam_01.cfg import ParametersConfig

bridge = CvBridge()
pub = rospy.Publisher("image_filtered", Image, queue_size=5)

h_min = 0
h_max = 0
s_min = 0
s_max = 0
v_min = 0
v_max = 0

def callback(config, level):
  # rospy.loginfo('{H_min}'.format(**config))
  global h_min
  global h_max
  global s_min
  global s_max
  global v_min
  global v_max
  
  h_min = config.H_min
  h_max = config.H_max
  s_min = config.S_min
  s_max = config.S_max
  v_min = config.V_min
  v_max = config.V_max
  return config

def callback_image(_msg):
  global bridge
  global pub
  
  src = bridge.imgmsg_to_cv2(_msg)
  hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
  # rospy.loginfo('H_min:{}'.format(h_min))
  mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
  dst = cv2.bitwise_and(src, src, mask=mask)
  
  msg = bridge.cv2_to_imgmsg(dst, 'bgr8')
  pub.publish(msg)
  
  return

if __name__=="__main__":
  rospy.init_node("hsv_filter", anonymous=False)
  srv = Server(ParametersConfig, callback)
  sub = rospy.Subscriber("/image_src", Image, callback_image)
  rospy.spin()