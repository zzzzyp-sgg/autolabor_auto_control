#include "autolabor_auto_control/auto_control_node.h"

#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace autolabor_auto_control {

AutoControl::AutoControl() {
    ros::NodeHandle private_node("~");
    
    private_node.param("linear_min", linear_min_, 0.05);
    private_node.param("linear_max", linear_max_, 0.25);
    private_node.param("linear_state", linear_state_, 0);
    
    private_node.param("angular_min", angular_min_, 1 / 180 * M_PI);
    private_node.param("angular_max", angular_max_, 10 / 180 * M_PI);
    private_node.param("angular_state", angular_state_, 0);
    
    private_node.param("rate", rate_, 10.0);
    
    linear_scale_ = linear_min_;
    angular_scale_ = angular_min_;
    send_flag_ = true;
}

AutoControl::~AutoControl() {
    linear_state_ = 0;
    angular_state_ = 0;

    if (send_flag_)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = linear_state_ * linear_scale_;
        twist.angular.z = angular_state_ * angular_scale_;
        twist_pub_.publish(twist);
        send_flag_ = false;

        ROS_WARN("AutoControl: stop");
    }

    twist_pub_timer_.stop();
}

void AutoControl::changeState(const autolabor_auto_control::motionState::ConstPtr& ms_msg) {
    // linear
    linear_state_ = ms_msg->linear_state;
    // augular
    angular_state_ = ms_msg->angular_state;
}

void AutoControl::twistCallback(const ros::TimerEvent &) {
    if (send_flag_) {
        geometry_msgs::Twist twist;
        twist.linear.x = linear_state_ * linear_scale_;
        twist.angular.z = angular_state_ * angular_scale_;
        twist_pub_.publish(twist);
        ROS_DEBUG_STREAM("linear: " << twist.linear.x << " angular: " << twist.angular.z);
    }
}

void AutoControl::subscriberThread() {
    ros::NodeHandle private_node("~");
    ros::Subscriber sub = private_node.subscribe<autolabor_auto_control::motionState>(
        "motion_state_topic", 10, boost::bind(&AutoControl::motionStateCallback, this, _1));

    ros::spinOnce();
}

void AutoControl::motionStateCallback(const autolabor_auto_control::motionState::ConstPtr& msg) {
    changeState(msg);
}

void AutoControl::run() {
    // TODO publish
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    twist_pub_timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &AutoControl::twistCallback, this);
    // subscribe state change
    boost::thread state_thread(boost::bind(&AutoControl::subscriberThread, this));

    ros::spin();
}
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "auto_control_node");
  autolabor_auto_control::AutoControl auto_control;
  auto_control.run();
  return 0;
}