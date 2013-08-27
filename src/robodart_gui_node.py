#! /usr/bin/env python

PACKAGE='robodart_control'

import roslib
roslib.load_manifest(PACKAGE)

from std_srvs.srv import Empty
from robodart_vision.srv import Point

import rospy

import sys


from Tkinter import Tk, Button, Frame, OptionMenu, StringVar


def throw_dart():
  resp = call_service('/robodart_control/throw_dart', Empty)
  return resp

def throw_dart_with_old_offset_callback():
  resp = call_service('/robodart_control/throw_dart_with_old_offset', Empty)
  return resp

def throw_dart_from_drop_position():
  resp = call_service('/robodart_control/throw_dart_from_drop_position', Empty)
  return resp

def open_gripper():
  resp = call_service('/robodart_control/open_gripper', Empty)
  return resp

def close_gripper():
  resp = call_service('/robodart_control/close_gripper', Empty)
  return resp

def move_to_drop_position():
  resp = call_service('/robodart_control/move_to_drop_position', Empty)
  return resp

def move_home():
  resp = call_service('/robodart_control/move_home', Empty)
  return resp

def reset_dart_camera_offset():
  resp = call_service('/robodart_control/reset_dart_camera_offset', Empty)
  return resp

def say_sentence_1():
  resp = call_service('/robodart_control/say_sentence_1', Empty)
  return resp

def say_sentence_2():
  resp = call_service('/robodart_control/say_sentence_2', Empty)
  return resp
  
def start_looking_around():
  resp = call_service('/robodart_control/start_looking_around', Empty)
  return resp

def stop_looking_around():
  resp = call_service('/robodart_control/stop_looking_around', Empty)
  return resp

def get_bullseye_center_offset():
  resp = call_service('/robodart_vision/get_bullseye_center_offset', Point)
  if resp is None:
    print "No bullseye center offset received!, no circles detected?"
    return resp
  return resp

def reset_to_arrow_1():
  resp = call_service('/robodart_control/reset_to_arrow_1', Empty)
  return resp

def call_service(serviceName, srv):
  rospy.wait_for_service(serviceName)
  try:
    service_method = rospy.ServiceProxy(serviceName, srv)
    resp = service_method()      
    return resp
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
    return None
  
    
def exit():

  sys.exit("Successfully shut down")

def stop():
  print 'Stop all movements'
  #TODO: stop alle movements

  
  
    
if __name__ == '__main__':
  # Initialize the node and name it.
  rospy.init_node('robodart_gui_node')


  gui = Tk()
  
  width = 50
  
  mm_btn = Button(gui, command = open_gripper, text = 'Open Gripper', height=1, width=width)
  mm_btn.grid(row=0, column=0)
  
  mm_btn = Button(gui, command = close_gripper, text = 'Close Gripper', height=1, width=width)
  mm_btn.grid(row=1, column=0)
  
  mm_btn = Button(gui, command = move_to_drop_position, text = 'Drop Position', height=1, width=width)
  mm_btn.grid(row=2, column=0)
  
  mm_btn = Button(gui, command = move_home, text = 'Home', height=1, width=width)
  mm_btn.grid(row=3, column=0)

  mm_btn = Button(gui, command = throw_dart, text = 'throw_dart', height=20, width=width)
  mm_btn.grid(row=6, column=0)
  
  mm_btn = Button(gui, command = throw_dart_with_old_offset_callback, text = 'throw_dart_with_old_offset_callback', height=1, width=width)
  mm_btn.grid(row=7, column=0)
  
  mm_btn = Button(gui, command = throw_dart_from_drop_position, text = 'throw_dart_from_drop_position', height=1, width=width)
  mm_btn.grid(row=8, column=0)


  #mm_btn = Button(gui, command = stop, text = 'Stop', height=1, width=width)
  #mm_btn.grid(row=8, column=0)
  
  mm_btn = Button(gui, command = reset_dart_camera_offset, text = 'Reset Dart-Camera-Offset', height=1, width=width)
  mm_btn.grid(row=9, column=0)
  
  mm_btn = Button(gui, command = say_sentence_1, text = 'Say sentence 1', height=1, width=width)
  mm_btn.grid(row=10, column=0)
  
  mm_btn = Button(gui, command = say_sentence_2, text = 'Say sentence 2', height=1, width=width)
  mm_btn.grid(row=11, column=0)
  
  mm_btn = Button(gui, command = start_looking_around, text = 'Start looking around', height=1, width=width)
  mm_btn.grid(row=12, column=0)
  
  mm_btn = Button(gui, command = stop_looking_around, text = 'Stop looking around', height=1, width=width)
  mm_btn.grid(row=13, column=0)
  
  mm_btn = Button(gui, command = get_bullseye_center_offset, text = 'get bullseye center offset', height=1, width=width)
  mm_btn.grid(row=14, column=0)
  
  mm_btn = Button(gui, command = reset_to_arrow_1, text = 'Reset to arrow 1', height=1, width=width)
  mm_btn.grid(row=15, column=0)

  mm_btn = Button(gui, command = exit, text = 'Exit', height=1, width=width)
  mm_btn.grid(row=16, column=0)

  gui.mainloop()