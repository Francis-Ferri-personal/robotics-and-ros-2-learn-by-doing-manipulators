#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

using namespace std::chrono_literals;

class SimplePublisherNode : public rclcpp::Node
{
    public:
        // For initialization we use variable(value) 
        SimplePublisherNode() : Node("simpler_publisher"), counter_(0){
            pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
            timer_ = create_wall_timer(1s, std::bind(&SimplePublisherNode::timerCallback, this));

            RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
        }


    private:
        unsigned int counter_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        void timerCallback(){
            auto msg = std_msgs::msg::String();
            msg.data = "Hello ROS 2 - counter: " + std::to_string(counter_++);
            pub_ -> publish(msg);
        }



};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}