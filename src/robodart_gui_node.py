#! /usr/bin/env python

PACKAGE='robodart_control'

import roslib
roslib.load_manifest(PACKAGE)

from std_srvs.srv import Empty

import rospy

import sys


from Tkinter import Tk, Button, Frame, OptionMenu, StringVar


def throw_dart():
  resp = call_service('/robodart_control/throw_dart', Empty)
  return resp

def throw_dart_without_adjusting_offset():
  resp = call_service('/robodart_control/throw_dart_without_adjusting_offset', Empty)
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
  
  mm_btn = Button(gui, command = throw_dart_without_adjusting_offset, text = 'throw_dart_without_adjusting_offset', height=1, width=width)
  mm_btn.grid(row=7, column=0)
  
  mm_btn = Button(gui, command = throw_dart_from_drop_position, text = 'throw_dart_from_drop_position', height=1, width=width)
  mm_btn.grid(row=8, column=0)


  #mm_btn = Button(gui, command = stop, text = 'Stop', height=1, width=width)
  #mm_btn.grid(row=8, column=0)
  
  mm_btn = Button(gui, command = reset_dart_camera_offset, text = 'Reset Dart-Camera-Offset', height=1, width=width)
  mm_btn.grid(row=9, column=0)

  mm_btn = Button(gui, command = exit, text = 'Exit', height=1, width=width)
  mm_btn.grid(row=10, column=0)

  gui.mainloop()