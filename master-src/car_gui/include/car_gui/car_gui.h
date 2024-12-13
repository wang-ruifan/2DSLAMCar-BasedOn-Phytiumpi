/******************************************************************************
 * 
 *  file: car_gui.h
 *  author: Ruifan Wang
 *  description: This file contains the declaration of the car_gui class..
 * 
*******************************************************************************/
#ifndef CARGUI_H
#define CARGUI_H

#include <QApplication>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mainwindow.h"

/******************************************************************************
 * 
 *  class: Publisher
 *  description: This class is the car_gui class. It is a Ros2 node that publishes
 *              the odometry message and broadcasts the tf message. It also contains
 *              the declaration of the mainwindow class.
 *
*******************************************************************************/
class Publisher : public rclcpp::Node{
    public:
    Publisher();

    private:
    //Timer callback function
    void timer_callback();

    //Variables
    float sampling_time;
    //Odometry variables
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    //Ros2 variables
    //Timer
    rclcpp::TimerBase::SharedPtr timer_;
    //Time
    rclcpp::Time last_time_, now_time_;
    //Broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    //Odometry message publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

    //Mainwindow
    MainWindow *mainwindow;
};

#endif