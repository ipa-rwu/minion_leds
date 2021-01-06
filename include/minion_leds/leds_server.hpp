#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <functional>
#include <memory>

#include "leds.hpp"
#include "minion_leds/srv/set_leds.hpp"

using SetLeds = minion_leds::srv::SetLeds;


class LedsService : public rclcpp::Node
{
public:
  LedsService();

private:
  Leds *led_obj;
  uint32_t _led_pin_R;
  uint32_t _led_pin_G;
  uint32_t _led_pin_B;

  std::string _service_name;
  rclcpp::Service<SetLeds>::SharedPtr _leds_service;

void handle_service(const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<SetLeds::Request> request,
              const std::shared_ptr<SetLeds::Response> response);
};
