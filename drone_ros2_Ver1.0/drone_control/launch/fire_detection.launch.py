from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    yolov5_weights = DeclareLaunchArgument(
        'weights',
        default_value='$(find yolov5_ros)/yolov5/weights/yolov5s.pt',
        description='Yolov5模型权重文件路径'
    )
    
    # 相机参数
    camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='相机图像话题'
    )
    
    # 创建节点
    yolov5_node = Node(
        package='yolov5_ros',
        executable='yolov5_ros',
        name='yolov5_node',
        output='screen',
        parameters=[{
            'weights': LaunchConfiguration('weights'),
            'image_topic': LaunchConfiguration('camera_topic'),
            'conf': 0.7,  # 置信度阈值
            'iou': 0.45,  # NMS IoU阈值
            'device': 'cpu',  # 使用CPU推理
        }]
    )
    
    fire_detection_node = Node(
        package='drone_control',
        executable='fire_detection_node',
        name='fire_detection_node',
        output='screen'
    )
    
    drone_control_node = Node(
        package='drone_control',
        executable='drone_control_node',
        name='drone_control_node',
        output='screen'
    )
    
    thermal_camera_node = Node(
        package='drone_control',
        executable='thermal_camera_node',
        name='thermal_camera_node',
        output='screen'
    )
    
    gui_node = Node(
        package='drone_control',
        executable='gui_node',
        name='gui_node',
        output='screen'
    )
    
    # 创建启动描述
    return LaunchDescription([
        yolov5_weights,
        camera_topic,
        yolov5_node,
        fire_detection_node,
        drone_control_node,
        thermal_camera_node,
        gui_node
    ])    