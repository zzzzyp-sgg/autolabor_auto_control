#ifndef AUTO_CONTROL_NODE_H
#define AUTO_CONTROL_NODE_H

#include "ros/ros.h"
#include <cmath>
#include "geometry_msgs/Twist.h"
#include <autolabor_auto_control/motionState.h>

namespace autolabor_auto_control {
class AutoControl
{
public:
    /// @brief  Constructor
    AutoControl();

    /// @brief  Destructor
    ~AutoControl();
    
    /// @brief  run
    void run();

    /// @brief  subscribe msg thread
    void subscriberThread();

    /// @brief  motion state callback
    void motionStateCallback(const autolabor_auto_control::motionState::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    ros::Timer twist_pub_timer_;

    int linear_state_, angular_state_;

    double rate_;
    double linear_scale_, angular_scale_;
    double linear_min_, linear_max_;
    double angular_min_, angular_max_;
    bool send_flag_;

    /// @brief  publish twist message
    void twistCallback(const ros::TimerEvent &);

    /// @brief  change state
    /// @todo   receive message, and change state
    void changeState(const autolabor_auto_control::motionState::ConstPtr& ms_msg);
};
}

#endif  // AUTO_CONTROL_NODE_H