#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk
import threading

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.bridge = CvBridge()
        
        # 订阅图像和报警信息
        self.sub_image = self.create_subscription(
            Image,
            '/fire/image',
            self.image_callback,
            10)
        
        self.sub_alarm = self.create_subscription(
            String,
            '/fire/alarm',
            self.alarm_callback,
            10)
        
        # 初始化GUI
        self.root = tk.Tk()
        self.root.title('无人机火焰检测系统')
        self.root.geometry('1000x600')
        
        # 创建标签显示图像
        self.image_label = tk.Label(self.root)
        self.image_label.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # 创建文本区域显示报警信息
        self.alarm_text = tk.Text(self.root, height=10, width=80)
        self.alarm_text.pack(side=tk.BOTTOM, fill=tk.X)
        
        # 当前图像
        self.current_image = None
        
        # 启动GUI线程
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()
        
        self.get_logger().info('GUI节点已启动')
    
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 调整图像大小以适应GUI
            cv_image = cv2.resize(cv_image, (960, 540))
            
            # 转换为PIL图像
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.current_image = Image.fromarray(rgb_image)
            
            # 更新GUI
            self.update_image()
        except Exception as e:
            self.get_logger().error(f'图像处理异常: {e}')
    
    def alarm_callback(self, msg):
        # 在文本区域显示报警信息
        self.alarm_text.insert(tk.END, f'\n[{self.get_clock().now().to_msg().sec}] {msg.data}')
        self.alarm_text.see(tk.END)
        
        # 添加高亮效果
        self.alarm_text.tag_add('highlight', f'end-2l linestart', f'end-2l lineend')
        self.alarm_text.tag_config('highlight', foreground='red', font=('Arial', 12, 'bold'))
    
    def update_image(self):
        if self.current_image:
            # 转换为Tkinter可用的格式
            photo = ImageTk.PhotoImage(image=self.current_image)
            
            # 更新标签
            self.image_label.config(image=photo)
            self.image_label.image = photo
    
    def run_gui(self):
        # 启动GUI主循环
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = GUINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    