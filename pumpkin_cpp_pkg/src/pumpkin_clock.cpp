#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

class PumpkinClockPublisherNode : public rclcpp::Node
{
public:
    PumpkinClockPublisherNode() : Node("pumpkin_clock_publisher")
    {
        mPublishFrequency = this->declare_parameter("publish_frequency", 1.0);

        mClockPublisher = this->create_publisher<builtin_interfaces::msg::Time>("/pumpkin/clock", 10);

        // The (1.0 / frequency) will give you the 'period' in seconds to publish the data, have to muliply by
        // 1000 and use milliseconds because the chrono::seconds expects an integer
        mClockTimer = this->create_wall_timer(std::chrono::milliseconds((int) (1000.0 / this->mPublishFrequency)),
                                                std::bind(&PumpkinClockPublisherNode::publishClock, this));

        RCLCPP_INFO(this->get_logger(), "Clock publisher has been started.");
    }

private:
    void publishClock()
    {
        auto msg = builtin_interfaces::msg::Time();
        msg = this->now();
        mClockPublisher->publish(msg);
    }

    double mPublishFrequency;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr mClockPublisher;
    rclcpp::TimerBase::SharedPtr mClockTimer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PumpkinClockPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}