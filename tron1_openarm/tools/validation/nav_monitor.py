"""검증용: AMCL·오도메트리·정답 자세와 /cmd_vel 을 3 초마다 출력 (Nav2 주행 디버그).  python3 nav_monitor.py <초>"""
import math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
rclpy.init(); n=Node('dbg', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
S={}
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
n.create_subscription(PoseStamped,'/ground_truth/pose',lambda m:S.__setitem__('gt',(m.pose.position.x,m.pose.position.y,yaw(m.pose.orientation))),10)
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:S.__setitem__('amcl',(m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation))),10)
n.create_subscription(Odometry,'/odom',lambda m:S.__setitem__('odom',(m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation))),10)
n.create_subscription(Twist,'/cmd_vel',lambda m:S.__setitem__('cmd',(m.linear.x,m.angular.z)),10)
t0=time.time(); last=0
while time.time()-t0<float(__import__('sys').argv[1]):
    rclpy.spin_once(n,timeout_sec=0.1)
    if time.time()-last>3:
        last=time.time(); f=lambda k: tuple(round(x,2) for x in S.get(k,()))
        print(int(time.time()-t0),'gt',f('gt'),'amcl',f('amcl'),'odom',f('odom'),'cmd',f('cmd'),flush=True)
