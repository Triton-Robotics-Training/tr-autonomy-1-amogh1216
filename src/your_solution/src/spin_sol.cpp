#include "spin_sol.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpinSolution>());
  rclcpp::shutdown();
  return 0;
}

//your code here
SpinSolution::SpinSolution() : Node("spinsolution"), count_(0) {

  subscription_pos = this->create_subscription<ArrayMsg>(
    "measuredpos", 10, std::bind(&SpinSolution::topic_callback_pos, this, _1));

  subscription_vel = this->create_subscription<ArrayMsg>(
    "measuredvel", 10, std::bind(&SpinSolution::topic_callback_vel, this, _1));

  publisher_ = this->create_publisher<ArrayMsg>("predictedpos", 10);

  timer_ = this->create_wall_timer(1ms, std::bind(&SpinSolution::timer_callback, this));

}

void SpinSolution::topic_callback_pos(const ArrayMsg & msg)
  {
    if (msg.data.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty message!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "I heard position: '%f', %f", msg.data[0], msg.data[1]);
    latest_pos = msg;
  }

void SpinSolution::topic_callback_vel(const ArrayMsg & msg)
  {
    if (msg.data.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Received empty message!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "I heard velocity: '%f', %f", msg.data[0], msg.data[1]);
    latest_vel = msg;
  }

void SpinSolution::timer_callback()
  {
    if (latest_pos.data.empty() || latest_vel.data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Complete data not received yet.");
      return;
    }

    // checks if measuredpos topic updated, if not, then predict position based on last received 
    // position and last received velocity
    if (!prevpos.data.empty() && abs(prevpos.data[0] - latest_pos.data[0]) <= 0.00001 && 
          abs(prevpos.data[1] - latest_pos.data[1]) <= 0.00001) {
      count_ += 1;
    }
    else {
      count_ = 0;
    }

    predictedpos = latest_pos;
    // .01 for 1ms between publishing
    predictedpos.data[0] = latest_pos.data[0] + (latest_vel.data[0] * .001 * count_);
    predictedpos.data[1] = latest_pos.data[1] + (latest_vel.data[1] * .001 * count_);


    prevpos = latest_pos;

    RCLCPP_INFO(this->get_logger(), "Vel: '%f', '%f'", latest_vel.data[0], latest_vel.data[1]);
    RCLCPP_INFO(this->get_logger(), "Pos: '%f', '%f'", latest_pos.data[0], latest_pos.data[1]);
    RCLCPP_INFO(this->get_logger(), "PredPos: '%f', '%f'", predictedpos.data[0], predictedpos.data[1]);
    publisher_->publish(predictedpos);
  }
