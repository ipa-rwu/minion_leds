#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "minion_leds/srv/set_leds.hpp"

using SetLeds = minion_leds::srv::SetLeds;


class LedsClient : public rclcpp::Node
{
public:
  LedsClient();

private:
  int32_t _timeout;
  std::chrono::milliseconds _server_timeout;
  std::string _service_name;
  std::string _ultrasonic_topic;
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr _range_subscription;

  rclcpp::Client<SetLeds>::SharedPtr _service_client;
  rclcpp::CallbackGroup::SharedPtr callback_group_input_;

  void callback_ultrasonic(const sensor_msgs::msg::Range::SharedPtr msg);
  void check_future(std::shared_future<SetLeds::Response::SharedPtr> future_result);
};
