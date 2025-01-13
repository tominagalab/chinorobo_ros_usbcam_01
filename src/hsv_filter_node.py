#!/usr/bin/env python
# import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from dynamic_reconfigure.server import Server
from chinorobo_ros_usbcam_01.cfg import ParametersConfig

# Node name
NODE_NAME = 'hsv_filter_node'

# Subscribe topic names
INPUT_IMAGE_TOPIC = '/image'

# Publish topic names
OUTPUT_FILTERED_IMAGE_TOPIC = '/hsv_filter/image_filtered'
OUTPUT_MASK_IMAGE_TOPIC = '/hsv_filter/image_mask'

bridge = CvBridge()
pub_filtered = rospy.Publisher(OUTPUT_FILTERED_IMAGE_TOPIC, Image, queue_size=5)
pub_mask = rospy.Publisher(OUTPUT_MASK_IMAGE_TOPIC, Image, queue_size=5)

h_min = 0
h_max = 0
s_min = 0
s_max = 0
v_min = 0
v_max = 0

def callback_config(config, level):
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
  
  # rospy.loginfo("image encoding = {}".format(_msg.encoding))

  src = bridge.imgmsg_to_cv2(_msg)
  
  if _msg.encoding == 'bgr8':
    pass
  elif _msg.encoding == 'rgb8':
    src = cv2.cvtColor(src, cv2.COLOR_RGB2BGR)
  else:
    pass
    
  hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
  dst = cv2.bitwise_and(src, src, mask=mask)
  
  msg_dst = bridge.cv2_to_imgmsg(dst, 'bgr8')
  msg_mask = bridge.cv2_to_imgmsg(mask, 'mono8')
  
  pub_filtered.publish(msg_dst)
  pub_mask.publish(msg_mask)
  
  return

if __name__=="__main__":
  rospy.init_node(NODE_NAME, anonymous=False)
  srv = Server(ParametersConfig, callback_config)
  sub = rospy.Subscriber(INPUT_IMAGE_TOPIC, Image, callback_image)
  rospy.spin()