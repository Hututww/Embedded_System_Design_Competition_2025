#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class ThermalCameraNode(Node):
    def __init__(self):
        super().__init__('thermal_camera_node')
        self.bridge = CvBridge()
        
        # 订阅RGB图像 (来自Yolov5)
        self.sub_image = self.create_subscription(
            Image,
            '/yolov5/image_raw',
            self.image_callback,
            10)
        
        # 发布最大温度
        self.pub_temp = self.create_publisher(
            Float32,
            '/thermal/max_temperature',
            10)
        
        # 发布温度图像
        self.pub_temp_image = self.create_publisher(
            Image,
            '/thermal/image',
            10)
        
        # 相机参数 (需要根据实际红外相机校准)
        self.temp_min = 20.0  # 最小温度范围
        self.temp_max = 150.0  # 最大温度范围
        
        self.get_logger().info('红外相机节点已启动')
    
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 模拟红外图像处理 (实际项目需使用真实红外相机数据)
            # 这里使用彩色映射来模拟温度分布
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            normalized_image = cv2.normalize(gray_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            thermal_image = cv2.applyColorMap(normalized_image, cv2.COLORMAP_JET)
            
            # 计算最大温度 (模拟值)
            max_temp = np.max(normalized_image) * (self.temp_max - self.temp_min) / 255.0 + self.temp_min
            
            # 发布最大温度
            temp_msg = Float32()
            temp_msg.data = max_temp
            self.pub_temp.publish(temp_msg)
            
            # 发布温度图像
            temp_image_msg = self.bridge.cv2_to_imgmsg(thermal_image, 'bgr8')
            temp_image_msg.header = msg.header
            self.pub_temp_image.publish(temp_image_msg)
            
            self.get_logger().debug(f'当前最大温度: {max_temp:.1f}°C')
        except Exception as e:
            self.get_logger().error(f'图像处理异常: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    