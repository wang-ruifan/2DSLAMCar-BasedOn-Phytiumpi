include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_laser",
  -- 发布map到odom之间的位姿态
  published_frame = "odom",
  odom_frame = "odom",
  -- 节点不用提供推测的里程计数据
  provide_odom_frame = false,
  -- 仅发布2D位资地图
  publish_frame_projected_to_2d = true,
  -- 使用odom发布的里程计数据辅助建图
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  -- 使用一个雷达
  num_laser_scans = 1,
  -- 不使用多波雷达
  num_multi_echo_laser_scans = 0,
  -- 不分割
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  -- 默认设置
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.3,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


-- 启动2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- 激光雷达最小扫描距离0.10,比机器人半径小的都忽略
TRAJECTORY_BUILDER_2D.min_range = 0.10
-- 激光雷达最大扫描距离为8
TRAJECTORY_BUILDER_2D.max_range = 8
-- 传感器数据超出有效范围最大值为2
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
-- 没有IMU，不使用IMU数据
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 使用实时回环检测来进行前端的扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- 提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- 对移动的敏感度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100
-- 对转向的敏感度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.001
-- Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.min_score = 0.65
-- 全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
