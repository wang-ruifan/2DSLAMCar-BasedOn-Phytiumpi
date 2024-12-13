# Launch gui to show current speed and control car, cartographer with LiDAR sensor to mapping, rviz2 for visualization
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Rviz2 configuration file
  rviz2_config = os.path.join(
      get_package_share_directory('car_gui'),
      'rviz2',
      'car_lidar.rviz'
  )
    
  # Cartographer configuration file
  pkg_share = FindPackageShare(package='car_gui').find('car_gui')
  use_sim_time = LaunchConfiguration('use_sim_time',default='false')
  resolution = LaunchConfiguration('resolution', default='0.01')
  publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.1')  
  configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(pkg_share))
  configuration_basename = LaunchConfiguration('configuration_basename', default='car2d.lua')

  # Cartographer node
  # Use the 'cartographer_node' component to start the SLAM algorithm
  cartographer_node = Node(
    package='cartographer_ros',
    executable='cartographer_node',
    name='cartographer_node',
    output='screen',
    parameters=[{'use_sim_time':use_sim_time}],
    arguments=['-configuration_directory',configuration_directory,'-configuration_basename', configuration_basename]
  )

  # Cartographer occupancy grid node
  # Use the 'cartographer_occupancy_grid_node' component to convert the map to an occupancy grid
  cartographer_occupancy_grid_node = Node(
    package='cartographer_ros',
    executable='cartographer_occupancy_grid_node',
    name='cartographer_occupancy_grid_node',
    output='screen',
    parameters=[{'use_sim_time':use_sim_time}],
    
    arguments=['-resolution',resolution,'-publish_period_sec', publish_period_sec]
  )
  
  # Rviz2 node
  # Use the 'rviz2' component to visualize the map and robot base_link current position in map
  rviz2_node = Node(
      package='rviz2',
      executable='rviz2',
      name='car_gui_rviz',
      parameters=[{'use_sim_time':use_sim_time}],
      arguments=['-d',rviz2_config],
      output='screen'
  )

  # Car GUI node
  # Use the 'car_gui' component to show current speed and control car
  car_gui = Node(
      package='car_gui',
      executable='car_gui',
      name='car_gui'
  )

  # LiDAR node
  # Use the 'ldlidar_sl_ros2_node' component to get LiDAR data
  ldlidar_node = Node(
      package='ldlidar_sl_ros2',
      executable='ldlidar_sl_ros2_node',
      name='ldlidar_publisher_ld14',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyACM0'},
        {'serial_baudrate' : 115200},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
  )

  # base_link to base_laser tf node
  # Use the 'static_transform_publisher' component to define the transformation between base_link and base_laser
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld14',
    arguments=['0','0','0.18','0','0','0','base_link','base_laser']
  ) 

  # Define LaunchDescription variable
  ld = LaunchDescription()

  # Add nodes to LaunchDescription
  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)
  ld.add_action(cartographer_node)
  ld.add_action(cartographer_occupancy_grid_node)
  ld.add_action(rviz2_node)
  ld.add_action(car_gui)
  
  return ld
