#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server
from chinorobo_ros_usbcam_01.cfg import ParametersConfig

def callback(config, level):
  return config

if __name__=="__main__":
  rospy.init_node("hsv_filter", anonymous=False)
  srv = Server(ParametersConfig, callback)
  rospy.spin()