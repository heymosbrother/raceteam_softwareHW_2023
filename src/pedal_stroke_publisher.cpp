#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class PedalStrokePublisher : public rclcpp::Node 
{
public:
	PedalStrokePublisher() : Node("pedal_stroke_publisher"), count_(0)
	{
		publisher_ = this->create_publisher<std_msgs::msg::Int32>("pedal_stroke", 10);
		timer_ = this->create_wall_timer(500ms, std::bind(&PedalStrokePublisher::publishData, this));
	}

private:
	void publishData()
	{
		auto message = std_msgs::msg::Int32();
		// simulate a sin wave
		message.data = static_cast<int>(2048 + 2047 * std::sin(2 * M_PI * count_ / 20.0) * std::cos(count_ / 17.7)); 	
		publisher_->publish(message);
		count_++;
		RCLCPP_INFO(this->get_logger(), "Publishing %d", message.data);
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
	int count_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PedalStrokePublisher>());
	rclcpp::shutdown();
	return 0;
}

