#include "spin_slow_update.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlowSolution>());
  rclcpp::shutdown();
  return 0;
}

SlowSolution::SlowSolution() : Node("slowsolution"), count_(0) {
  subscription_ = this->create_subscription<ArrayMsg>(
    "measuredpos", 10, std::bind(&SlowSolution::topic_callback, this, _1));

  publisher_ = this->create_publisher<ArrayMsg>("predictedpos", 10);

  timer_ = this->create_wall_timer(100ms, std::bind(&SlowSolution::timer_callback, this));
  
}

// your code here
