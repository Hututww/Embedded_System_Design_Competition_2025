import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        # 订阅飞控状态
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        # 发布目标位置
        self.pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        # 服务客户端：起飞、模式切换
        self.arm_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.current_state = State()
        self.init_services()

    def state_callback(self, msg):
        self.current_state = msg

    def init_services(self):
        # 等待服务端就绪
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /mavros/cmd/takeoff 服务...')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /mavros/set_mode 服务...')

    def takeoff(self, altitude):
        """无人机起飞到指定高度"""
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'起飞至 {altitude} 米成功')
        else:
            self.get_logger().error('起飞失败')

    def goto_position(self, x, y, z):
        """设置无人机目标位置（局部坐标系）"""
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.pos_pub.publish(pose)
        self.get_logger().info(f'前往目标位置: ({x}, {y}, {z})')

    def set_mode(self, mode):
        """切换飞行模式"""
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info(f'切换模式为 {mode} 成功')
        else:
            self.get_logger().error(f'切换模式 {mode} 失败')

def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    # 示例：起飞 → 前往目标点 → 悬停（可结合实际逻辑扩展）
    controller.set_mode('OFFBOARD')
    controller.takeoff(2.0)  # 起飞到 2 米
    controller.goto_position(5.0, 0.0, 2.0)  # 前往局部坐标系 (5,0,2)
    time.sleep(5)  # 悬停等待
    rclpy.shutdown()

if __name__ == '__main__':
    main()