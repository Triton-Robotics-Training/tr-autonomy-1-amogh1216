#ifndef YOUR_SOLUTION_SRC_SPIN_SLOW_UPDATE_H_
#define YOUR_SOLUTION_SRC_SPIN_SLOW_UPDATE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using ArrayMsg = std_msgs::msg::Float64MultiArray;

using namespace std::chrono_literals;

using std::placeholders::_1;

class SlowSolution : public rclcpp::Node {
 public:
  SlowSolution();
 private:
  // your code here
  void topic_callback(const ArrayMsg & msg)
    {
      if (msg.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty message!");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "I heard: '%f', %f", msg.data[0], msg.data[1]);
      latest_pos = msg;
    }

  void timer_callback()  
    {
      if (latest_pos.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "No data received yet.");
        return;
      } 
      ArrayMsg predictedpos = latest_pos;

      // predictedpos.data[0] = 1;
      // predictedpos.data[1] = 1;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f'", predictedpos.data[0], predictedpos.data[1]);
      publisher_->publish(predictedpos);
    }
    
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ArrayMsg>::SharedPtr publisher_;
  size_t count_;
  ArrayMsg latest_pos;
  rclcpp::Subscription<ArrayMsg>::SharedPtr subscription_;

};

#endif //YOUR_SOLUTION_SRC_SPIN_SLOW_UPDATE_H_
