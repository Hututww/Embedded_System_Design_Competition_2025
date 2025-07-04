#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from fire_drone.msg import FireDetectionResult  # 导入自定义消息类型

class Yolov5Detector(Node):
    def __init__(self):
        super().__init__('yolov5_detector')
        self.bridge = CvBridge()
        
        # 订阅相机图像
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # 发布火灾检测结果
        self.detection_pub = self.create_publisher(
            FireDetectionResult, 'fire_detection/result', 10)  # 使用自定义消息
        
        # 加载YOLOv5模型
        self.model = torch.hub.load(
            '/home/root/drone_ws/src/fire_drone/yolo',  # 本地YOLOv5路径
            'custom',
            path='/home/root/drone_ws/src/fire_drone/weights/best.pt',
            source='local')
        
        # 设置模型参数
        self.model.conf = 0.5  # 置信度阈值
        self.get_logger().info('YOLOv5 detector initialized')
    
    def image_callback(self, msg):
        # 转换ROS图像消息为OpenCV格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # 执行目标检测
        results = self.model(cv_image)
        
        # 解析检测结果
        detections = results.pandas().xyxy[0]
        
        # 创建消息实例
        fire_msg = FireDetectionResult()
        fire_msg.is_fire_detected = False
        
        # 查找火灾目标（根据实际类别名称调整）
        fire_detections = detections[detections['name'] == 'fire']
        if not fire_detections.empty:
            # 取置信度最高的检测结果
            most_confident = fire_detections.iloc[fire_detections['confidence'].idxmax()]
            
            # 填充消息字段
            fire_msg.is_fire_detected = True
            fire_msg.x = (most_confident['xmin'] + most_confident['xmax']) / 2  # 目标中心x
            fire_msg.y = (most_confident['ymin'] + most_confident['ymax']) / 2  # 目标中心y
            fire_msg.z = 0.0  # 2D检测，深度需通过其他方式估计
            fire_msg.confidence = most_confident['confidence']
            
            self.get_logger().info(f"Fire detected! Confidence: {fire_msg.confidence:.2f}")
        
        # 发布检测结果
        self.detection_pub.publish(fire_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = Yolov5Detector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()