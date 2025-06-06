import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trivia_bot.espeak_python_easy_interface import text_to_speech

class Speaker(Node):
    def __init__(self):
        super().__init__('speaker')

        self.sub = self.create_subscription(String, 'speaker', self.callback, 10)
    
    def callback(self, msg):
        message = msg.data
        text_to_speech(message)
        self.get_logger().info(f"Question: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = Speaker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    