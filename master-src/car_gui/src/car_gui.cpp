/******************************************************************************
 * 
 *  file: car_gui.cpp
 *  author: Ruifan Wang
 *  description: This file is the main file of the car_gui node. It creates a
 *              publisher object and a mainwindow object. 
 *              The publisher object is used to publish the odometry message 
 *              and the mainwindow object is used to control the car. 
 * 
*******************************************************************************/
#include "car_gui.h"
#include "mainwindow.h"

/******************************************************************************
 * 
 *  name: Publisher
 *  class: Publisher
 *  author: Ruifan Wang
 *  description: This is the constructor of the Publisher class. It initializes
 *             the timer, the tf2 broadcaster, the odom publisher and the mainwindow. 
 * 
*******************************************************************************/
Publisher::Publisher() : Node("car_gui")
{
    RCLCPP_INFO(this->get_logger(),"node %s started", "car_gui");
    // timer initialization, the timer will call the timer_callback function every 30ms
    timer_ = this->create_wall_timer(std::chrono::milliseconds(30),std::bind(&Publisher::timer_callback,this));
    // tf2 broadcaster and odom publisher initialization, the odom publisher will publish the odometry message
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",50);
    // mainwindow initialization, the mainwindow is used to control the car and display the GUI
    mainwindow = new MainWindow();
    mainwindow->show();
}

/******************************************************************************
 * 
 *  name: timer_callback
 *  class: Publisher
 *  author: Ruifan Wang
 *  description: This function is called by the timer every 30ms. It uses the 
 *              mainwindow's controlCar function to get the current car speed
 *              and the control the car according to the rocker position. Then
 *              it calculates the odometry message and publishes it.
 * 
*******************************************************************************/
void Publisher::timer_callback()
{
    // odometry message and tf2 broadcaster initialization
    geometry_msgs::msg::TransformStamped odom_trans;
    nav_msgs::msg::Odometry odom_msg;

    // car break flag and speed initialization
    int flag = 0;
    float vx = 0.0,vz = 0.0;

    // call the mainwindow's controlCar function to get the current car speed
    mainwindow->controlCar(&flag, &vx, &vz);
    //RCLCPP_INFO(this->get_logger(),"publishing: %f %f", vx, vz);
    
    // odometry calculation correction
    vx = vx * 2.0;
    vz = vz * 1.74;
    
    // sampling time calculation
    now_time_ = this->now();
    if(last_time_.seconds() == 0)   last_time_ = now_time_;
    sampling_time = (now_time_ - last_time_).seconds();
    last_time_ = now_time_;

    /************odometry calculation**********/
    // x and y are the position of the car in the odom frame
    // z is the yaw of the car in the odom frame

    // x = x + vx * cos(z) * sampling_time
    x += (vx*cos(z)) * sampling_time;
    // y = y + vx * sin(z) * sampling_time
    y += (vx*sin(z)) * sampling_time;
    // z = z + vz * sampling_time
    z += vz * sampling_time;
    // print the odometry message for testing
    RCLCPP_INFO(this->get_logger(),"x: %lf y: %lf z: %lf %lf",x,y,z,sampling_time);

    /************odometry message publishing**********/

    // orientation calculation
    tf2::Quaternion orientation;
    // call the setRPY function to set the orientation of the car according to the yaw
    orientation.setRPY(0.0,0.0,z);

    // set the odometry broadcast message x, y
    // x, y are the position of the car in the odom frame
    odom_trans.header.stamp = this->now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = 0 - x;
    odom_trans.transform.translation.y = 0 - y;
    odom_trans.transform.rotation = tf2::toMsg(orientation);
    odom_broadcaster_->sendTransform(odom_trans);

    // set the odometry message x, y, vx, vz
    // x, y are the position of the car in the odom frame
    // vx is the linear velocity of the car
    // vz is the angular velocity of the car
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = 0 - x;
    odom_msg.pose.pose.position.y = 0 - y;
    odom_msg.pose.pose.orientation = tf2::toMsg(orientation);
    
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = -vx;
    odom_msg.twist.twist.angular.z = -vz;
    odom_publisher_->publish(odom_msg);
}
 
/******************************************************************************
 * 
 *  name: main
 *  author: Ruifan Wang
 *  description: This is the main function of the car_gui node. It initializes
 *              the rclcpp node and the QApplication object. Then it creates a
 *              publisher object and a spin thread to run the publisher. Finally
 *              it runs the QApplication object.
 * 
*******************************************************************************/
int main(int argc, char* argv[]){
    // rclcpp node initialization
    rclcpp::init(argc, argv);
    // QApplication object initialization
    QApplication app(argc, argv);
    // publisher object initialization
    auto publisher = std::make_shared<Publisher>();
    // spin thread run the publisher
    std::thread spin_thread([publisher]() -> void { rclcpp::spin(publisher); });
    // detach the spin thread
    spin_thread.detach();
    // execute the QApplication object
    app.exec();
    // shutdown the rclcpp node
    rclcpp::shutdown();
    return 0;
}
