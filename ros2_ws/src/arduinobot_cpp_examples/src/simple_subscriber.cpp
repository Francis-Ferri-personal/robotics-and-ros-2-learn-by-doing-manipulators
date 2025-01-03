#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class SimpleSubscriberNode : public rclcpp::Node
{
public:
    SimpleSubscriberNode() : Node("simple_subscriber")
    {
        sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSubscriberNode::msgCallback, this, _1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void msgCallback(const std_msgs::msg::String msg) const{
        RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data.c_str());
    }
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node =  std::make_shared<SimpleSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}