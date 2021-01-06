#include <functional>
#include <memory>
#include <string>
#include "minion_leds/leds_server.hpp"


LedsService::LedsService() : Node("minion_leds_server")
  {
    this->declare_parameter<int>("led_pin_red", 3);
    this->declare_parameter<int>("led_pin_green", 2);
    this->declare_parameter<int>("led_pin_blue", 5);
    this->declare_parameter<std::string>("leds_service", "/leds_server");

    this->get_parameter("led_pin_red", this->_led_pin_R);
    this->get_parameter("led_pin_green", this->_led_pin_G);
    this->get_parameter("led_pin_blue", this->_led_pin_B);

    this->get_parameter("leds_service", this->_service_name);

    RCLCPP_INFO(this->get_logger(), "Using green led pin %i", this->_led_pin_G);
    led_obj = new Leds(_led_pin_R, _led_pin_G, _led_pin_B);

    this->_leds_service = this->create_service<SetLeds>(_service_name, std::bind(&LedsService::handle_service, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "Ready for action on service %s", _service_name.c_str() );
  }

void LedsService::handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SetLeds::Request> request,
    const std::shared_ptr<SetLeds::Response> response
  )
  {
    (void)request_header;
 
    this->led_obj->setColorLed(request->r, request->g, request->b);
    
    response->success = 1;
  }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedsService>());
  rclcpp::shutdown();
  return 0;
}