#! /usr/bin/env python

PACKAGE='robodart_control'

import roslib
roslib.load_manifest('robodart_control')

import rospy

import sys
import pickle
import time
import threading

from Tkinter import Tk, Button, Frame, OptionMenu, StringVar

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from math import pi
from math import atan2
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf import TransformListener

import actionlib

from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from geometry_msgs.msg import PoseStamped

from robodart_vision.srv import Point
from std_srvs.srv import Empty




class Robodart_control():
  
  # import ipdb; ipdb.set_trace()
  
  REFERENCE_FRAME = 'katana_base_link' # All positions are defined relative to this frame, 
                     # a new frame should be created in the urdf to define a aim frame
  GRIPPER_FRAME = 'katana_gripper_tool_frame' #Name of the gripper frame
  
  CAMERA_OFFSET = [0,0] #This tupel describes the constant offset between the camera and the actual dart drop position.
  
  AIMING_CENTER_POSITION = (0.478,0) #the aiming is done relative to this position, AIMING_CENTER_POSITION is defined in REFERENCE_FRAME
  
  last_position = [0,0]
  
  dart_center_offset = [0,0]
  
  is_first_throw = True
  
  client = None
  
  group = None
  
  tf_listener = None
  
  my_robodart_vision = None
  
  saved_positions = {'dart1_position':[0,0]}
   
  #tk dropdown var
  var = None

  scene = None
  

 
  def __init__(self):

    #init action client
    self.client = actionlib.ActionClient('gripper_action_controller/gripper_command', GripperCommandAction)
    
    print 'Waiting for grippercommand action server'
    self.client.wait_for_server()
    
    #init move_group
    self.group = MoveGroupCommander('arm')
    self.group.set_pose_reference_frame(self.REFERENCE_FRAME)
    self.group.set_goal_tolerance(0.0001)
    
    #init tf listener
    self.tf_listener = TransformListener()
    
    self.scene = PlanningSceneInterface()

  def throw_dart(self):  
    #self.move_to_drop_position()
    self.pickup_dart()
    self.move_to_drop_position()
    #Wait until dart is ready
    time.sleep(3)
    self.open_gripper()
    #Sleep just because of Timeouts
    time.sleep(1)
    self.move_home()

    """
    if self.is_first_throw:
      self.center_dart_board()
    else: 
      self.move_relative_to_last_position(self.dart_center_offset)
    
    print "Take reference picture"  
    self.take_reference_picture()
    self.open_gripper()
    
    print "Sleep 2 seconds"
    time.sleep(2) #drop time #TODO measure
    
    print "Get dart to center offset"
    self.dart_center_offset = self.get_dart_center_offset()
    """
    
  def center_dart_board(self):
    print "Center Dart Board"
    self.move_to_drop_position()

    print "Sleep 4 seconds"
    time.sleep(4) #drop time #TODO measure
    
    self.move_relative_to_last_position(self.get_bullseye_center_offset())

  """
  def calibrate(self):
    print 'Calibrating'
    self.center_dart_board()
    self.open_gripper()
  """
  def pickup_dart(self):
    print 'Starting Pickup dart'


    print 'Move to pre pickup position'
    p = PoseStamped()
    p.header.frame_id = self.REFERENCE_FRAME
    p.pose.position.x = -0.36246
    p.pose.position.y = -0.2487
    
    p.pose.position.z = 0.492327
    
    #p.pose.orientation.w = 1
    
    q = quaternion_from_euler(-0.032, -0.0376, 0)
    
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    

    #this is only needed for 5Dof Katana Robot
    self.replace_yaw_angle_with_reachable_value(p)

    print "Planning frame: " ,self.group.get_planning_frame()
    print "Pose reference frame: ",self.group.get_pose_reference_frame()

    self.group.set_pose_target(p, self.GRIPPER_FRAME)
    self.group.go()
    
    wait_time = 2

    time.sleep(wait_time)

    self.open_gripper()
    time.sleep(wait_time)
    print 'Move to pickup position'

    p.pose.position.z = 0.425327

    self.group.set_pose_target(p, self.GRIPPER_FRAME)
    self.group.go()

    time.sleep(wait_time)
    self.close_gripper()
    time.sleep(wait_time)
    print 'Move to post pickup position'
    p.pose.position.z = 0.492327

    self.group.set_pose_target(p, self.GRIPPER_FRAME)
    self.group.go()

    time.sleep(wait_time)

    """try in joint space
    self.group.set_named_target('dart_1')
    self.group.go()
    #self.move_to_position_in_gripper_frame([0.4,0], 0.38)

    """

    """ first try with pick, not working because the moveit python bindings have no function pick("obejct", grasp)
    self.scene.remove_world_object("test")
    p = PoseStamped()
    p.header.frame_id = self.REFERENCE_FRAME
   
    p.pose.orientation.w = 1.0
    p.pose.position.x = 0.35
    p.pose.position.y = 0.0
    p.pose.position.z = 0.4

    print "Pickup pose: ", p
    
    self.scene.add_box("test", p, (0.01, 0.01, 0.1))

    self.group.pick("test")
    """

    

    """ second try with cartesian path, faild because path is not executed?
    p = PoseStamped()
    p.header.frame_id = self.REFERENCE_FRAME
    p.pose.position.x = 0.4
    p.pose.position.y = 0
    #p.pose.position.z = 0.
    
    p.pose.position.z = 0.38
    
    #p.pose.orientation.w = 1
    
    q = quaternion_from_euler(-0.032, -0.0376, 0)
    
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    
    
    #this is only needed for 5Dof Katana Robot
    self.replace_yaw_angle_with_reachable_value(p)

    print 'Desired pose', p.pose

    waypoints = [p.pose]
    (trajectory, fraction_completed) = self.group.compute_cartesian_path(waypoints, 0.01, 0.03)

    print 'Trajectory', trajectory

    self.group.execute(trajectory)
    """

    """3rd try, not working, no goal state found, perhaps python function not working?
    #0..5: x,y,z,r,p,y    2 = z-axis
    self.group.shift_pose_target(0, -0.01, self.GRIPPER_FRAME)

    self.group.go()

    """

   
     
  def move_home(self):  
    print 'Move to home position'  
    self.group.set_named_target('home_stable')
    self.group.go()
    
  def move_to_drop_position(self):
    print 'Move to drop position'
    self.move_to_position_in_robot_frame(self.AIMING_CENTER_POSITION)
    
  def open_gripper(self):
    print "Open Gripper"
    goal = GripperCommandGoal()
    #positition open max 0.3
    goal.command.position = 0.0
    self.client.send_goal(goal)
    
  def close_gripper(self):
    print "Close Gripper"
    goal = GripperCommandGoal()
    #position close max -0.44
    #goal.command.position = -0.19
    goal.command.position = -0.21 
    self.client.send_goal(goal)
       
  def move_relative_to_last_position(self, offset_vector = (0,0)):
    

    """Moves to the position defined by last_position + offset_vector.
    
    Arguments:
    >>offset_vector -- the offset as tupel(x,y) (default (0,0))
    """
    print 'Move relative to last position: ' , self.last_position, 'offset: ', offset_vector
    
    new_position = [self.last_position[0] + offset_vector[0], self.last_position[1] + offset_vector[1]]
    
    self.move_to_position_in_gripper_frame(new_position)
    
    print 'New Position: ', new_position
    
  def move_to_position_in_gripper_frame(self, target_point = [0,0], z_axis = 0.4):
    self.group.set_pose_reference_frame(self.GRIPPER_FRAME)

    self.move_to_position(target_point)

  def move_to_position_in_robot_frame(self, target_point = [0,0], z_axis = 0.4):
    self.group.set_pose_reference_frame(self.REFERENCE_FRAME)

    self.move_to_position(target_point)
   
    
  def move_to_position(self, target_point = [0,0], z_axis = 0.4):
    """Moves to the position given by target_point in self.REFERENCE_FRAME
    
    Arguments:
    >>target_point -- the offset as vector (default [0,0])
    """
    print 'Move to position'

      
    
    #save position for relative movement
    self.last_position = list(target_point)
    
    p = PoseStamped()
    p.header.frame_id = self.REFERENCE_FRAME
    p.pose.position.x = target_point[0]
    p.pose.position.y = target_point[1]
    
    p.pose.position.z = z_axis
    
    #p.pose.orientation.w = 1
    
    q = quaternion_from_euler(-0.032, -0.0376, 0)
    
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    
    
    #this is only needed for 5Dof Katana Robot
    self.replace_yaw_angle_with_reachable_value(p)

    print "Planning frame: " ,self.group.get_planning_frame()
    print "Pose reference frame: ",self.group.get_pose_reference_frame()
    
    #group.set_rpy_target([0, 0, 0],"katana_gripper_tool_frame")
    #group.set_position_target([p.pose.position.x,p.pose.position.y,p.pose.position.z], GRIPPER_FRAME)
    
   
    
    self.group.set_pose_target(p, self.GRIPPER_FRAME)
    
    self.group.go()
    
    print "Current rpy: " , self.group.get_current_rpy(self.GRIPPER_FRAME)

  def replace_yaw_angle_with_reachable_value(self, pose_stamped):
    #print "Original pose" , pose_stamped
    euler_orientation = list(euler_from_quaternion([pose_stamped.pose.orientation.x,pose_stamped.pose.orientation.y,pose_stamped.pose.orientation.z,pose_stamped.pose.orientation.w]))
    euler_orientation[2] = atan2(pose_stamped.pose.position.y, pose_stamped.pose.position.x)
    
    quaternion = quaternion_from_euler(euler_orientation[0], euler_orientation[1], euler_orientation[2])
    pose_stamped.pose.orientation.x = quaternion[0]
    pose_stamped.pose.orientation.y = quaternion[1]
    pose_stamped.pose.orientation.z = quaternion[2]
    pose_stamped.pose.orientation.w = quaternion[3]
    
    #print "Adjusted yaw: " , pose_stamped
    
  def get_current_gripper_position(self):
        
    try:
      (trans,rot) = self.tf_listener.lookupTransform(self.REFERENCE_FRAME, self.GRIPPER_FRAME, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print 'Error: No tf found!'
    
    print 'Transformation:' , trans
    print 'Rotation:' , rot
    
    return trans
  
  def save_current_gripper_position(self):
    """
    sf = 'value is %s' % self.var.get()
    
    self.saved_positions[name] = self.get_current_gripper_position()
    pickle.dump(self.saved_positions, open('positions.p', 'wb'))
    """
    
    
  def get_gripper_position_by_name(self, name):
    self.saved_positions.update(pickle.load(open('positions.p', 'rb')))  
    return self.saved_positions[name]
  

  def take_reference_picture(self):
    resp = self.call_service('/robodart_vision/take_reference_picture', Empty)
    return resp

  def get_bullseye_center_offset(self):
    resp = self.call_service('/robodart_vision/get_bullseye_center_offset', Point)
    return [resp.x, resp.y]

  def get_dart_center_offset(self):
    resp = self.call_service('/robodart_vision/get_dart_center_offset', Point)
    return [resp.x, resp.y]

  def call_service(self, serviceName, srv):
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
  
    
if __name__ == '__main__':
  # Initialize the node and name it.
  rospy.init_node('robodart_control_node')

  try:
    my_robodart_control = Robodart_control()
    
    
  except rospy.ROSInterruptException: pass

  gui = Tk()
  
  mm_btn = Button(gui, command = my_robodart_control.open_gripper, text = 'Open Gripper', height=1, width=15)
  mm_btn.grid(row=0, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.close_gripper, text = 'Close Gripper', height=1, width=15)
  mm_btn.grid(row=1, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.move_to_drop_position, text = 'Drop Position', height=1, width=15)
  mm_btn.grid(row=2, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.move_home, text = 'Home', height=1, width=15)
  mm_btn.grid(row=3, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.get_current_gripper_position, text = 'Get Gripper Position', height=1, width=15)
  mm_btn.grid(row=4, column=0)
  
  mm_btn = Button(gui, command = my_robodart_control.pickup_dart, text = 'pickup_dart', height=1, width=15)
  mm_btn.grid(row=5, column=0)

  mm_btn = Button(gui, command = my_robodart_control.throw_dart, text = 'throw_dart', height=1, width=15)
  mm_btn.grid(row=6, column=0)



  
  mm_btn = Button(gui, command = my_robodart_control.center_dart_board, text = 'Center Dart Board', height=1, width=15)
  mm_btn.grid(row=7, column=0)
  
  mm_btn = Button(gui, command = exit, text = 'Exit', height=1, width=15)
  mm_btn.grid(row=8, column=0)
  
  """
  # initial value
  self.var = StringVar(gui)
  self.var.set('aiming_center_position')
  mm_option_menu = OptionMenu(gui, self.var, *my_robodart_control.saved_positions, command = my_robodart_control.save_current_gripper_position)
  mm_option_menu.grid(row=6, column=0)
"""
     

  gui.mainloop()
