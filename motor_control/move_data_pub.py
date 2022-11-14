#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
    
    
class dxl_move_pub(Node): 
    def __init__(self):
        super().__init__("move_data_pub") 

        self.publisher_speed = self.create_publisher(String, "move_data_speed", 10)
        self.publisher_angle = self.create_publisher(String, "move_data_angle", 10)
        self.timer_speed = self.create_timer(1, self.publish_speed)
        self.timer_angle = self.create_timer(1, self.publish_angle)
        self.get_logger().info("speed and angle pubs are ready")
        self.get_logger()

    def publish_speed(self):
        speed_input = String()
        speed_input.data = "0"
        self.publisher_speed.publish(speed_input)

    def publish_angle(self):
        angle_input = String()
        angle_input.data = "0"
        self.publisher_angle.publish(angle_input)


    
    
def main(args=None):
    rclpy.init(args=args)
    node = dxl_move_pub()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()