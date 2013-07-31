#! /usr/bin/env python
import roslib
roslib.load_manifest('robodart_vision')
import rospy
import sys
from robodart_vision.srv import Point
from std_srvs.srv import Empty

def take_reference_picture():
  resp = call_service('/robodart_vision/take_reference_picture', Empty)
  return resp

def get_bullseye_center_offset():
  resp = call_service('/robodart_vision/get_bullseye_center_offset', Point)
  return [resp.x, resp.y]

def get_dart_center_offset():
  resp = call_service('/robodart_vision/get_dart_center_offset', Point)
  return [resp.x, resp.y]

def call_service(serviceName, srv):
  rospy.wait_for_service(serviceName)
  try:
      service_method = rospy.ServiceProxy(serviceName, srv)
      resp = service_method()
      return resp
  except rospy.ServiceException, e:
      print "Service call failed: %s"%e
      return None


#Main Part:

if __name__ == "__main__":
  print "main"
  print "take_reference_picture: ", take_reference_picture()
  print "get_bullseye_center_offset:\t", get_bullseye_center_offset()
  print "get_dart_center_offset:\t\t", get_dart_center_offset()
