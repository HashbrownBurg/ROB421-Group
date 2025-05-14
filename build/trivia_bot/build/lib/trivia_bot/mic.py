#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_event_handler import ParameterEventHandler

class Mic(Node):
    def __init__(self):
        super().__init__('mic')
        self.pub = self.create_publisher(String, 'mic', 10)

        self.declare_parameter('guess', 'blue')

        self.handler = ParameterEventHandler(self)
        self.handler.add_parameter_callback(
            parameter_name='guess',
            node_name='mic',
            callback=self.guess_callback
        )

    def send(self):
        param_value = self.get_parameter('guess').get_parameter_value()
        msg = String()
        msg.data = param_value.string_value 
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def guess_callback(self, parameter):
        value = parameter_value_to_python(parameter.value)
        self.get_logger().info(f"Guess changed to: {value}")
        self.send()

def main(args=None):
    rclpy.init(args=args)
    node = Mic()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
