import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


class LinInter(Node):
    def __init__(self):
        super().__init__('Linear_Int')
        # Node initialization and setup here
        self.publisher_ = self.create_publisher(Point, 'robot_position', 10)
        self.update_rate = 0.2
        self.timer = self.create_timer(self.update_rate, self.update_position) #Position is updated every second
        self.velocity_msg = Twist()
        #Receive the (starting) current position, and the 
        self.starting_position = self.declare_parameter("starting_position").get_parameter_value().double_array_value
        self.final_position = self.declare_parameter("final_position").get_parameter_value().double_array_value    

        self.current_position = self.starting_position

        self.get_logger().info(f'Starting Position: {self.current_position}')
        self.get_logger().info(f'Final Position: {self.final_position}')

        self.starting_x_pos = self.starting_position[0]
        self.starting_y_pos = self.starting_position[1]
        self.starting_z_pos = self.starting_position[2]

        self.final_x_pos = self.final_position[0]
        self.final_y_pos = self.final_position[1]
        self.final_z_pos = self.final_position[2]

        #Using twist to set the x, y, and z velocity from the start
        self.velocity_msg.linear.x = (self.final_position[0] - self.starting_position[0]) * self.travel_rate(5)
        self.velocity_msg.linear.y = (self.final_position[1] - self.starting_position[1]) * self.travel_rate(5)
        self.velocity_msg.linear.z = (self.final_position[2] - self.starting_position[2]) * self.travel_rate(5)

        self.tolerance = 0.01
    def travel_rate(self, t):
        #We assume that b(t) function for rate of robot travel 
        #over time is linear to start
        #We assume that it takes 5 seconds for the task to run to completion
        #This assumption is LARGE, but it is the same bounded constant
        #assumption taken by Dr. Turpin in his paper
        r = 1/t
        return r


    def update_position(self):
        #The current position of the robot is updated here.
        #The update rate is basically the amount of time that has elapsed since
        #the last run.
        # self.get_logger().info(f'Current Position: {self.current_position}')
        var = True
        if (abs(self.final_position[0] - self.current_position[0]) < self.tolerance) and var:
            self.velocity_msg.linear.x = 0
            self.velocity_msg.linear.y = 0
            self.velocity_msg.linear.z = 0
            self.get_logger().info(f'Finished Position: {self.current_position}')
            var = False
        else:
            pass
    
        self.current_position[0] += self.update_rate * self.velocity_msg.linear.x
        self.current_position[1] += self.update_rate * self.velocity_msg.linear.y
        self.current_position[2] += self.update_rate * self.velocity_msg.linear.z


def main(args=None):
    rclpy.init(args=args)
    node = LinInter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()