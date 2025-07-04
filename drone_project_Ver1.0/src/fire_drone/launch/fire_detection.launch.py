from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 获取包的路径
    pkg_share = FindPackageShare('fire_drone')
    
    # 声明启动参数
    config_file = LaunchConfiguration('config_file')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'params.yaml']),
        description='Path to the configuration file'
    )
    
    # 声明MAVROS串口参数
    fcu_url = LaunchConfiguration('fcu_url')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyUSB0:57600',
        description='FCU connection URL'
    )
    
    # 声明相机设备参数
    camera_device = LaunchConfiguration('camera_device')
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )
    
    # 声明是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time from /clock topic'
    )
    
    return LaunchDescription([
        # 声明启动参数
        config_file_arg,
        fcu_url_arg,
        camera_device_arg,
        use_sim_time_arg,
        
        # 启动USB相机驱动
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'video_device': camera_device,
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'use_sim_time': use_sim_time
            }]
        ),
        
        # 启动YOLOv5检测器
        Node(
            package='fire_drone',
            executable='yolov5_detector.py',
            name='yolov5_detector',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # 启动无人机控制器
        Node(
            package='fire_drone',
            executable='drone_controller.py',
            name='drone_controller',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # 启动主控制器
        Node(
            package='fire_drone',
            executable='main_controller.py',
            name='main_controller',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ]
        ),
        
        # 启动MAVROS（与飞控通信）
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace='mavros',
            output='screen',
            parameters=[
                {'fcu_url': fcu_url},
                {'system_id': 1},
                {'component_id': 1},
                {'use_sim_time': use_sim_time},
                PathJoinSubstitution([pkg_share, 'config', 'mavros_params.yaml'])
            ]
        )
    ])