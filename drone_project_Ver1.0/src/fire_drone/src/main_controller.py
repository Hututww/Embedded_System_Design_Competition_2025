#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fire_drone.msg import FireDetectionResult  # 导入自定义消息类型
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        
        # 订阅火灾检测结果
        self.fire_sub = self.create_subscription(
            FireDetectionResult, 'fire_detection/result', self.fire_callback, 10)
        
        # 发布目标位置
        self.target_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)
        
        # 服务客户端（用于模式切换和解锁）
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        
        # 初始化参数
        self.fire_detected = False
        self.fire_position = None
        self.search_altitude = 10.0  # 搜索高度（米）
        
        self.get_logger().info('Main controller initialized')
    
    def fire_callback(self, msg):
        """处理火灾检测结果"""
        self.fire_detected = msg.is_fire_detected
        
        if self.fire_detected:
            # 保存火灾位置
            self.fire_position = (msg.x, msg.y, msg.z)
            self.get_logger().info(f"Fire detected at position: {self.fire_position}")
            
            # 飞向火灾位置（简化版，实际需考虑坐标系转换）
            self.goto_fire_location()
        else:
            self.get_logger().info("No fire detected, continuing search")
            # 执行搜索模式（简化版）
            self.search_pattern()
    
    def goto_fire_location(self):
        """飞向火灾位置"""
        if not self.fire_position:
            return
        
        # 创建目标位置消息
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"  # 假设使用map坐标系
        
        # 设置目标位置（这里简化处理，实际需根据相机坐标系转换）
        target.pose.position.x = self.fire_position[0]
        target.pose.position.y = self.fire_position[1]
        target.pose.position.z = self.search_altitude  # 保持安全高度
        
        # 发布目标位置
        self.target_pub.publish(target)
        self.get_logger().info(f"Flying to fire location: ({target.pose.position.x}, {target.pose.position.y}, {target.pose.position.z})")
    
    def search_pattern(self):
        """执行搜索模式（示例：方形搜索）"""
        # 这里仅为示例，实际项目中应实现更复杂的搜索算法
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        
        # 设置搜索路径点（简化版）
        target.pose.position.x = 5.0  # 示例X坐标
        target.pose.position.y = 5.0  # 示例Y坐标
        target.pose.position.z = self.search_altitude
        
        self.target_pub.publish(target)
        self.get_logger().info(f"Searching: ({target.pose.position.x}, {target.pose.position.y}, {target.pose.position.z})")

def main(args=None):
    rclpy.init(args=args)
    controller = MainController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()