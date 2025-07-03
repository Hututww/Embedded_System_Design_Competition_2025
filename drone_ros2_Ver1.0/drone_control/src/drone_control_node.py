#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
import time

class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')
        self.current_state = State()
        self.target_pose = None
        self.armed = False
        self.mode = 'MANUAL'
        
        # 订阅无人机状态
        self.sub_state = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)
        
        # 订阅目标位置
        self.sub_target = self.create_subscription(
            PoseStamped,
            '/drone/target_pose',
            self.target_callback,
            10)
        
        # 发布位置指令
        self.pub_pose = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10)
        
        # 发布速度指令
        self.pub_vel = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10)
        
        # 服务客户端
        self.client_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.client_mode = self.create_client(SetMode, '/mavros/set_mode')
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('无人机控制节点已启动')
    
    def state_callback(self, msg):
        self.current_state = msg
        self.armed = msg.armed
        self.mode = msg.mode
    
    def target_callback(self, msg):
        self.target_pose = msg
        self.get_logger().info(f'收到目标位置: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})')
    
    def timer_callback(self):
        if self.target_pose is None:
            return
        
        # 切换到OFFBOARD模式
        if self.mode != 'OFFBOARD':
            self.set_mode('OFFBOARD')
        
        # 解锁无人机
        if not self.armed:
            self.arm()
        
        # 发布目标位置
        self.pub_pose.publish(self.target_pose)
    
    def arm(self):
        while not self.client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，等待中...')
        
        req = CommandBool.Request()
        req.value = True
        future = self.client_arm.call_async(req)
        future.add_done_callback(self.arm_callback)
    
    def arm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('无人机已解锁')
            else:
                self.get_logger().error('解锁失败')
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {e}')
    
    def set_mode(self, mode):
        while not self.client_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，等待中...')
        
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.client_mode.call_async(req)
        future.add_done_callback(lambda f: self.mode_callback(f, mode))
    
    def mode_callback(self, future, mode):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f'已切换到{mode}模式')
            else:
                self.get_logger().error(f'切换到{mode}模式失败')
        except Exception as e:
            self.get_logger().error(f'服务调用异常: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    