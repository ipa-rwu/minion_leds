#include <string>
#include "minion_leds/leds_client.hpp"


LedsClient::LedsClient() : Node("minion_leds_client")
  {
    callback_group_input_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    this->declare_parameter<int32_t>("time_out", 1000);

    this->declare_parameter<std::string>("leds_service", "/leds_server");
    this->declare_parameter<std::string>("ultrasonic_topic", "/ultrasonic_range");

    this->get_parameter("leds_service", this->_service_name);
    this->get_parameter("ultrasonic_topic", this->_ultrasonic_topic);
    this->get_parameter("time_out", this->_timeout);

    _server_timeout = std::chrono::milliseconds(_timeout);

    this->_service_client = this->create_client<SetLeds>(_service_name, rmw_qos_profile_services_default, callback_group_input_);
    
    // Make sure the server is actually there before continuing
    RCLCPP_INFO(
      this->get_logger(), "Waiting for \"%s\" service",
      _service_name.c_str());

    if (!_service_client->wait_for_service(std::chrono::milliseconds(10000)))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Service " << this->_service_name << " is unavailable.");
    }
    RCLCPP_INFO(this->get_logger(), "Connected to %s" , _service_name.c_str() );

    
    this->_range_subscription = this->create_subscription<sensor_msgs::msg::Range>(
      _ultrasonic_topic,
      rclcpp::QoS(10),
      std::bind(&LedsClient::callback_ultrasonic, this, std::placeholders::_1)
      );
    RCLCPP_INFO(this->get_logger(), "Ready for subscribing to %s", this->_ultrasonic_topic.c_str() );
  }

void LedsClient::callback_ultrasonic(const sensor_msgs::msg::Range::SharedPtr msg)
{
  auto ultrasonic_dist = msg->range;
  auto request = std::make_shared<SetLeds::Request>();

  auto response_received_callback = [this](rclcpp::Client<SetLeds>::SharedFuture result) 
  {
    auto response = result.get();
    while (response->success != 1 && rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (response->success != 1)
    {
        RCLCPP_WARN(
        this->get_logger(),
        "Node timed out while executing service call to %s.", _service_name.c_str());
    }
    // RCLCPP_INFO(this->get_logger(), "Service responsed '%s' : %i", _service_client->get_service_name(), response->success);
  };

  if (ultrasonic_dist == std::numeric_limits<float>::infinity())
  {
    request->r = false;
    request->g = true;
    request->b = false;

    auto result = _service_client->async_send_request(request, response_received_callback);
  }

  if (0.2 <= ultrasonic_dist && ultrasonic_dist <= 0.6)
  {
    request->r = true;
    request->g = true;
    request->b = false;
    auto result = _service_client->async_send_request(request, response_received_callback);
  }

  if (ultrasonic_dist <= 0.2)
  {
    request->r = true;
    request->g = false;
    request->b = false;
    auto result = _service_client->async_send_request(request, response_received_callback);
  }

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<LedsClient>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}