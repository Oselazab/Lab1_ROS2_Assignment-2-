"""Run after launching lab1 in a separate terminal, in the same ROS_DOMAIN_ID."""
import math
import threading
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from lab1_interfaces.action import GoToPose, DrawSquare
from lab1_interfaces.msg import TargetPose
from lab1_interfaces.srv import SetSpeedLimits
from std_srvs.srv import Trigger
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter

rclpy.init()
n = Node('lab1_verification')
e = MultiThreadedExecutor(num_threads=2)
e.add_node(n)
thread = threading.Thread(target=e.spin, daemon=True)
thread.start()
poses=[]
velocities=[]
n.create_subscription(Pose, '/turtle1/pose', lambda p: poses.append(p), 10)
n.create_subscription(Twist, '/turtle1/cmd_vel', lambda v: velocities.append(v), 10)

def wait(f, timeout=90):
 deadline=time.monotonic()+timeout
 while not f.done() and time.monotonic()<deadline: time.sleep(.02)
 assert f.done(), 'Response timeout'
 return f.result()

def service(kind,name,request):
 c=n.create_client(kind,name)
 assert c.wait_for_service(timeout_sec=10),name
 return wait(c.call_async(request))

def goal(kind,name,request):
 c=ActionClient(n,kind,name)
 assert c.wait_for_server(timeout_sec=10), name
 feedback=[]
 h=wait(c.send_goal_async(request,feedback_callback=lambda f:feedback.append(f)))
 return c,h,feedback

def reset():
 assert service(Trigger,'/reset_turtle',Trigger.Request()).success
 time.sleep(.15)
 p=poses[-1]
 assert math.hypot(p.x-5.544445,p.y-5.544445)<.01 and abs(p.theta)<.01

def reached(x,y,yaw):
 time.sleep(.1)
 p=poses[-1]
 assert math.hypot(p.x-x,p.y-y)<.06,(p,x,y)
 assert abs(math.atan2(math.sin(p.theta-yaw), math.cos(p.theta-yaw)))<.05

try:
 reset();reset()
 print('PASS repeated reset',flush=True)
 assert not service(SetSpeedLimits,'/set_speed_limits',SetSpeedLimits.Request(max_linear_speed=-1.,max_angular_speed=2.)).success
 assert not service(SetSpeedLimits,'/set_speed_limits',SetSpeedLimits.Request(max_linear_speed=float('nan'),max_angular_speed=2.)).success
 assert service(SetSpeedLimits,'/set_speed_limits',SetSpeedLimits.Request(max_linear_speed=3.,max_angular_speed=3.)).success
 invalid=SetParametersAtomically.Request(parameters=[Parameter('max_linear_speed',value=0.).to_parameter_msg()])
 assert not service(SetParametersAtomically,'/lab1_server/set_parameters_atomically',invalid).result.successful
 print('PASS parameter and service validation',flush=True)
 for x,y,yaw in [(6.3,5.544445,1.57),(5.2,5.544445,-3.0),(5.2,6.3,-1.57),(5.2,5.2,0.)]:
  c,h,fb=goal(GoToPose,'/go_to_pose',GoToPose.Goal(target=TargetPose(x=x,y=y,yaw=yaw)))
  assert h.accepted
  result=wait(h.get_result_async())
  assert result.result.success and len(fb)>0,result
  reached(x,y,yaw)
 print('PASS four directions and final yaw',flush=True)
 c,h,_=goal(GoToPose,'/go_to_pose',GoToPose.Goal(target=TargetPose(x=20.,y=5.,yaw=0.)))
 assert not h.accepted
 c,h,_=goal(GoToPose,'/go_to_pose',GoToPose.Goal(target=TargetPose(x=9.,y=9.,yaw=0.)))
 assert h.accepted
 assert not service(Trigger,'/reset_turtle',Trigger.Request()).success
 c2,h2,_=goal(GoToPose,'/go_to_pose',GoToPose.Goal(target=TargetPose(x=3.,y=3.,yaw=0.)))
 assert not h2.accepted
 time.sleep(.3)
 assert service(SetSpeedLimits,'/set_speed_limits',SetSpeedLimits.Request(max_linear_speed=.5,max_angular_speed=.6)).success
 time.sleep(.2)
 velocities.clear();time.sleep(.3)
 assert velocities and all(abs(v.linear.x)<=.50001 and abs(v.angular.z)<=.60001 for v in velocities)
 assert wait(h.cancel_goal_async()).goals_canceling
 assert wait(h.get_result_async()).status==5
 print('PASS live speed caps, overlap rejection and motion cancellation',flush=True)
 reset()
 assert service(SetSpeedLimits,'/set_speed_limits',SetSpeedLimits.Request(max_linear_speed=3.,max_angular_speed=3.)).success
 for run in range(2):
  c,h,fb=goal(DrawSquare,'/draw_square',DrawSquare.Goal(side_length=.7))
  assert h.accepted
  result=wait(h.get_result_async())
  assert result.result.success,result
  assert fb[-1].feedback.completed_sides==4
  reached(5.544445,5.544445,0.)
  print(f'PASS square run {run+1}',flush=True)
 c,h,_=goal(DrawSquare,'/draw_square',DrawSquare.Goal(side_length=8.))
 assert not h.accepted
 c,h,_=goal(DrawSquare,'/draw_square',DrawSquare.Goal(side_length=1.))
 assert h.accepted
 time.sleep(.5)
 assert wait(h.cancel_goal_async()).goals_canceling
 assert wait(h.get_result_async()).status==5
 reset()
 print('PASS square bounds, cancellation and reset recovery',flush=True)
 print('ALL INTEGRATION CHECKS PASSED',flush=True)
finally:
 e.shutdown();thread.join(timeout=3);n.destroy_node();rclpy.shutdown()
