import rclpy

from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriberNode(Node):
    def __init__(self):
        super().__init__("simple_subscriber")
        self.subs_ = self.create_subscription(String, "chatter", self.messageCallback, 10)

    def messageCallback(self, msg: String):
        self.get_logger().info(f"I heard: {msg.data}")


def main():
    rclpy.init()
    simpleSubscriber = SimpleSubscriberNode()
    rclpy.spin(simpleSubscriber)
    simpleSubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()