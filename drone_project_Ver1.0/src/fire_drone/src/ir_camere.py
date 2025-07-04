import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class IRCamera(Node):
    def __init__(self):
        super().__init__('ir_camera')
        self.bridge = CvBridge()
        # 订阅原始图像（假设话题与相机驱动发布一致）
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        # 发布处理后的红外图像（可用于检测模块）
        self.image_pub = self.create_publisher(
            Image, '/ir_camera/processed_image', 10)

    def image_callback(self, msg):
        """处理红外相机图像：转 OpenCV、预处理、（可选）增强火焰特征等"""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 示例：转灰度（简化，实际可加红外图像专属算法，比如测温阈值过滤）
            gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            # 发布处理后图像
            processed_msg = self.bridge.cv2_to_imgmsg(gray_img, encoding='mono8')
            self.image_pub.publish(processed_msg)
            self.get_logger().info('处理并发布红外相机图像')
        except Exception as e:
            self.get_logger().error(f'图像处理失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    ir_cam = IRCamera()
    rclpy.spin(ir_cam)
    ir_cam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()