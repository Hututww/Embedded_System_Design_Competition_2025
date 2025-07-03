#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        self.bridge = CvBridge()
        self.fire_detected = False
        self.target_pose = None
        
        # 订阅Yolov5检测结果
        self.sub_detections = self.create_subscription(
            Detection2DArray,
            '/yolov5/detections',
            self.detections_callback,
            10)
        
        # 订阅红外温度数据
        self.sub_thermal = self.create_subscription(
            Float32,
            '/thermal/max_temperature',
            self.thermal_callback,
            10)
        
        # 发布目标位置
        self.pub_target = self.create_publisher(
            PoseStamped,
            '/drone/target_pose',
            10)
        
        # 发布报警信息
        self.pub_alarm = self.create_publisher(
            String,
            '/fire/alarm',
            10)
        
        # 发布图像
        self.pub_image = self.create_publisher(
            Image,
            '/fire/image',
            10)
        
        self.get_logger().info('火焰检测节点已启动')
    
    def detections_callback(self, msg):
        # 检查是否检测到火焰
        for detection in msg.detections:
            if detection.results[0].id == 0 and detection.results[0].score > 0.7:
                self.get_logger().info('检测到火焰!')
                self.fire_detected = True
                
                # 计算目标位置 (简化版，实际需根据相机模型和深度信息计算)
                self.target_pose = PoseStamped()
                self.target_pose.header = msg.header
                self.target_pose.pose.position.x = 10.0  # 示例值，实际需计算
                self.target_pose.pose.position.y = 0.0
                self.target_pose.pose.position.z = 2.0
                
                # 发布目标位置
                self.pub_target.publish(self.target_pose)
                
                # 发布火焰图像
                self.pub_image.publish(detection.source_img)
                break
        else:
            self.fire_detected = False
    
    def thermal_callback(self, msg):
        # 检查是否检测到火焰且温度超过阈值
        if self.fire_detected and msg.data > 100.0:
            self.get_logger().warning(f'高温报警! 温度: {msg.data}°C')
            alarm_msg = String()
            alarm_msg.data = f'火灾报警! 坐标: ({self.target_pose.pose.position.x}, {self.target_pose.pose.position.y}), 温度: {msg.data}°C'
            self.pub_alarm.publish(alarm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    