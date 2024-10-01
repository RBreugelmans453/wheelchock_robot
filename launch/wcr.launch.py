import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg
import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('wheelchock_robot'))
    urdf_file = os.path.join(pkg_path,'wcs.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_file)

    # Lidar node variables
    lidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    lidar_node_name = 'ydlidar_ros2_driver_node'

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    imu_params = PathJoinSubstitution(
        [
            FindPackageShare("mpu6050driver"),
            "params",
            "mpu6050.yaml",
        ]
    )

    imu_filter_config = PathJoinSubstitution(
        [
            FindPackageShare("wheelchock_robot"),
            "config",
            "madgwick.yaml",
        ]
    )

    robot_localization_config = PathJoinSubstitution(
        [
            FindPackageShare("wheelchock_robot"),
            "config",
            "ekf.yaml",
        ]
    )

    mpu6050driver_node = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050driver_node',
        output="screen",
        emulate_tty=True,
        parameters=[imu_params]
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[robot_localization_config],
    )

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_node",
        output="screen",
        parameters=[imu_filter_config, {"use_mag": False}, {"publish_tf": False}, {"fixed_frame": "odom"}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Create the hardware interface node
    odrive_controller_node = Node(
        package='wheelchock_robot',
        executable='odrive_control',
        name='odrive_control',
        output='screen',
    )

    # Create a lidar node
    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               lidar_share_dir, 'params', 'TG.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        params_declare,
        robot_state_publisher_node,
        joint_state_publisher_node,
        odrive_controller_node,
        imu_filter,
        mpu6050driver_node,
        robot_localization,

        driver_node,
        tf2_node,
    ])