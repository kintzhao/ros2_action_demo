#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import math
import rclpy                                              
from rclpy.node import Node 
from rclpy.clock import Clock
from rclpy.clock import ROSClock

#import tf_transformations                                 # TF坐标变换库
#from tf_transformations import euler_from_quaternion

from tf2_ros import TransformException                    # TF左边变换的异常类
from tf2_ros.buffer import Buffer                         # 存储坐标变换信息的缓冲类
from tf2_ros.transform_listener import TransformListener  # 监听坐标变换的监听器类
from geometry_msgs.msg import TransformStamped, Pose2D  
from geometry_msgs.msg import Twist                       # ROS2 速度控制消息
from rclpy.action import ActionServer
from navigation_action_msg.action import CarNavigate
from nav_msgs.msg import Odometry
import numpy as np

from rclpy.executors import MultiThreadedExecutor

def euler_from_quaternion(x,y,z,w):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def wor_to_loc( w, cen):
    cos_robot = np.cos(cen.theta)
    sin_robot = np.sin(cen.theta)
    diff = Pose2D()
    diff.x = w.x - cen.x
    diff.y = w.y - cen.y
    diff.theta = w.theta - cen.theta   
    diff.theta = np.arctan2(np.sin(diff.theta), np.cos(diff.theta))

    loc_target = Pose2D()
    loc_target.x = diff.x *cos_robot +diff.y*sin_robot
    loc_target.y = diff.y *cos_robot -diff.x*sin_robot
    loc_target.theta = np.arctan2(loc_target.y, loc_target.x)  #diff.theta

    return loc_target
 
class carFollowingServer(Node):

    def __init__(self, name):
        super().__init__(name)                                 
        self.get_logger().info(f" carFollowingServer Init")
        self._action_server = ActionServer( self, CarNavigate, 'car_navigation_action', self.execute_callback)

        self.tf_buffer = Buffer()                                   # 创建保存坐标变换信息的缓冲区
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 创建坐标变换的监听器
        # self.target_position = None

        self.declare_parameter('base_frame', 'base_link')             # 创建一个源坐标系名的参数
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value# 优先使用外部设置的参数值，否则用默认值

        self.declare_parameter('target_frame', 'odom')             # 创建一个目标坐标系名的参数
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value    # 优先使用外部设置的参数值，否则用默认值
        self.trans = TransformStamped()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1) # 创建跟随运动小车的速度话题
        #self.timer = self.create_timer(0.03, self.on_timer)          # 创建一个固定周期的定时器，处理坐标信息
        self.cur_odom = Odometry()        
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    async def execute_callback(self, goal_handle):
    #def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = CarNavigate.Result()
        result.finish = False
        self.feedback_msg = CarNavigate.Feedback()
        msg = Twist()                                      # 创建速度控制消息

        self.goal_handle = goal_handle
        target_2d = Pose2D()
        target_2d.x = goal_handle.request.x          # 获取客户端发送的目标点的数据
        target_2d.y = goal_handle.request.y

        time.sleep(0.1)
        while True:
            # Check if there's a cancel request
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.finish = False
                return result
        
            self.car_position_x = self.cur_odom.pose.pose.position.x #  self.trans.transform.translation.x     # 小车的x坐标位置
            self.car_position_y = self.cur_odom.pose.pose.position.y # self.trans.transform.translation.y     # 小车的y坐标位置
            quat = self.cur_odom.pose.pose.orientation
            euler = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)

            robot_2d = Pose2D()
            robot_2d.x = self.cur_odom.pose.pose.position.x
            robot_2d.y = self.cur_odom.pose.pose.position.y
            robot_2d.theta = euler[2]

            relate_2d = wor_to_loc(target_2d, robot_2d)
 
            
            self.get_logger().info(f"robot_2d: {robot_2d.x, robot_2d.y, robot_2d.theta}, target_2d: {target_2d.x, target_2d.y}")
            self.get_logger().info(f"relate_2d: {relate_2d.x, relate_2d.y, relate_2d.theta} ")

            scale_rotation_rate = 0.5                          # 根据小车角度，计算角速度
            msg.angular.z = scale_rotation_rate * relate_2d.theta
            scale_forward_speed = 0.1                          # 根据小车距离，计算线速度
            msg.linear.x = scale_forward_speed * math.sqrt( relate_2d.x ** 2 + relate_2d.y ** 2)
            v_flag = 1.0
            if msg.linear.x < 0.0:
               v_flag = -1.0

            if abs(msg.linear.x) > 0.5:
               msg.linear.x = 0.5
            elif abs(msg.linear.x) < 0.05:
               msg.linear.x = 0.05
            msg.linear.x = msg.linear.x *v_flag
            
            if msg.angular.z > 0.6:
               msg.angular.z = 0.6 
            if msg.angular.z < -0.6:
               msg.angular.z = -0.6                
            self.cmd_pub.publish(msg)                        # 发布速度指令，小车运动
            self.get_logger().info(f"vel: {msg.linear.x, msg.angular.z}")


            self.feedback_msg.x = relate_2d.x     # 反馈信息为小车与目标点的相对位置距离
            self.feedback_msg.y = relate_2d.y
            if abs(self.feedback_msg.x)<=0.02 and abs(self.feedback_msg.y)<=0.02:
                msg.angular.z = 0.0
                msg.linear.x = 0.0
                self.cmd_pub.publish(msg)                    # 让小车停止运动
                self.get_logger().info(f" reached goal")

                result.finish = True
                goal_handle.succeed()
                #goal_handle.publish_result(result)
                return result
            else:
                #self.get_logger().info(f"Surplus distance {self.feedback_msg.x} , {self.feedback_msg.y}")
                goal_handle.publish_feedback(self.feedback_msg)
            time.sleep(0.03)

    def on_timer(self):
        try:
            now = ROSClock().now() #rclpy.time.Time()                                 # 获取ROS系统的当前时间
            self.trans = self.tf_buffer.lookup_transform( self.target_frame, self.base_frame, now) # 监听当前时刻源坐标系到目标坐标系的坐标变换
        except TransformException as ex:                            # 如果坐标变换获取失败，进入异常报告
            self.get_logger().info(
                f'Could not transform {self.target_frame} to {self.base_frame}: {ex}')

        pos  = self.trans.transform.translation                          # 获取位置信息
        quat = self.trans.transform.rotation                             # 获取姿态信息（四元数）
        #euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        euler = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        # self.get_logger().info('Get %s --> %s pose: [%f, %f, %f] [%f, %f, %f]' 
        #   % (self.target_frame, self.base_frame, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]))

    def odom_callback(self, msg):
        # Handle the incoming odometry message here
        self.cur_odom = msg
        #self.get_logger().info(f"Received odometry message: Position: {msg.pose.pose.position}, Orientation: {msg.pose.pose.orientation}")
 
def main(args=None):
    rclpy.init(args=args)                       
    print("car_following_server")
    node = carFollowingServer("car_following_server")        
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()