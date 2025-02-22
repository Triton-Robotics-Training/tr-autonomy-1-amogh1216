#ifndef YOUR_SOLUTION_SRC_SPIN_SOL_H_
#define YOUR_SOLUTION_SRC_SPIN_SOL_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using ArrayMsg = std_msgs::msg::Float64MultiArray;

using std::placeholders::_1;
using namespace std::chrono_literals;

class SpinSolution : public rclcpp::Node {
 public:
  SpinSolution();
 private:
    // your code here

    void topic_callback_pos(const ArrayMsg & msg);
    void topic_callback_vel(const ArrayMsg & msg);
    void timer_callback();


    rclcpp::Subscription<ArrayMsg>::SharedPtr subscription_pos;
    rclcpp::Subscription<ArrayMsg>::SharedPtr subscription_vel;
    rclcpp::Publisher<ArrayMsg>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    ArrayMsg latest_pos;
    ArrayMsg latest_vel;
};

#endif //YOUR_SOLUTION_SRC_SPIN_SOL_H_
