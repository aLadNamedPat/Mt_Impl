import rclpy
import numpy as np
from rclpy.node import Node
import ast
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

from msg_tst.msg import TaskCompletion, RobotPath

class MultiLinInter(Node):
    def __init__(self):
        super().__init__('MultiLinear_Int')
        ###Node initialization and setup here
        self.update_rate = 0.2
        # self.timer = self.create_timer(self.update_rate, self.update_position) #Position is updated every second        
        ###All robots must have an ID to track where they are traveling to  
        self.robotID = int(self.declare_parameter('robot_id', 0).get_parameter_value().integer_value)
        self.publisher_ = self.create_publisher(TaskCompletion, "task_completion", 10)
        # self.publisher_2 = self.create_publisher(PoseStamped, f"Robot{self.robotID}", 10)
        self.subscription = self.create_subscription(
            RobotPath,
            'robot_paths',
            self.listener_callback,
            10
        )

        self.current_time = 0
        self.final_time = 3

        self.final_position = []
        self.starting_position = []
        self.current_position = []
        # self.get_logger().info(f'Starting Position: {self.current_position}')
        # self.get_logger().info(f'Final Position: {self.final_position}')
        self.velocity_msg = Twist()

        self.tolerance = 0.01

    def listener_callback(self, msg):
        # self.get_logger().info(f'Robot {self.robotID} has been received.')
        if (int(msg.robot_id) == self.robotID):
            self.starting_position = ast.literal_eval(msg.path)[0]
            self.final_position = ast.literal_eval(msg.path)[1]
            self.current_position = self.starting_position
            self.get_logger().info(f'Robot {self.robotID} has been launched.')
            self.setPosAndVel()
            self.timer = self.create_timer(self.update_rate, self.update_position)

    def setPosAndVel(self):
        self.starting_x_pos = self.starting_position[0]
        self.starting_y_pos = self.starting_position[1]
        self.starting_z_pos = self.starting_position[2]

        self.final_x_pos = self.final_position[0]
        self.final_y_pos = self.final_position[1]
        self.final_z_pos = self.final_position[2]

        #Using twist to set the x, y,  - self.starting_position[2]) and z velocity from the start
        self.current_position[0] = self.final_position[0] * self.travel_rate()[1] + self.starting_position[0] * self.travel_rate()[0]
        self.current_position[1] = self.final_position[1] * self.travel_rate()[1] + self.starting_position[1] * self.travel_rate()[0]
        self.current_position[2] = self.final_position[2] * self.travel_rate()[1] + self.starting_position[2] * self.travel_rate()[0]

    # def setPosAndVel(self):
    #     self.starting_x_pos = self.starting_position[0]
    #     self.starting_y_pos = self.starting_position[1]
    #     self.starting_z_pos = self.starting_position[2]

    #     self.final_x_pos = self.final_position[0]
    #     self.final_y_pos = self.final_position[1]
    #     self.final_z_pos = self.final_position[2]

    #     #Using twist to set the x, y, and z velocity from the start
    #     self.velocity_msg.linear.x = (self.final_position[0] - self.starting_position[0]) * self.travel_rate(5)
    #     self.velocity_msg.linear.y = (self.final_position[1] - self.starting_position[1]) * self.travel_rate(5)
    #     self.velocity_msg.linear.z = (self.final_position[2] - self.starting_position[2]) * self.travel_rate(5)

    def travel_rate(self):
        r1 = 1 - self.current_time / self.final_time
        r2 = self.current_time / self.final_time
        return r1, r2
    # def travel_rate(self, t):
    #     #We assume that b(t) function for rate of robot travel 
    #     #over time is linear to start
    #     #We assume that it takes 5 seconds for the task to run to completion
    #     #This assumption is LARGE, but it is the same bounded constant
    #     #assumption taken by Dr. Turpin in his paper
    #     r = 1/t


    #     return r
    # def publish_position(self):
    #     pose = PoseStamped()
    #     pose.header.stamp = self.get_clock().now().to_msg()
    #     pose.header.frame_id = "map"  # Set the frame_id to "map"
        
    #     # Set the pose position and orientation
    #     pose.pose.position.x = self.current_position[0]
    #     pose.pose.position.y = self.current_position[1]
    #     pose.pose.position.z = self.current_position[2]
    #     pose.pose.orientation.w = 0.0
        
    #     self.publisher_2.publish(pose)

    def update_position(self):
        #The current position of the robot is updated here.
        #The update rate is basically the amount of time that has elapsed since
        #the last run.
        # self.get_logger().info(f'Current Position: {self.current_position}')
        if (abs(self.final_position[0] - self.current_position[0]) < self.tolerance
                and abs(self.final_position[1] - self.current_position[1]) < self.tolerance
                and abs(self.final_position[2] - self.current_position[2]) < self.tolerance):
            self.velocity_msg.linear.x = 0
            self.velocity_msg.linear.y = 0
            self.velocity_msg.linear.z = 0
            self.get_logger().info(f'Robot {self.robotID} has reached goal at postion: {self.current_position}')
            self.timer.cancel()
            msg = TaskCompletion()
            msg.robot_id = str(self.robotID)
            msg.completed = True
            self.publisher_.publish(msg)
        self.current_time += self.update_rate
        self.setPosAndVel()
        # self.current_position[0] += self.update_rate * self.velocity_msg.linear.x
        # self.current_position[1] += self.update_rate * self.velocity_msg.linear.y
        # self.current_position[2] += self.update_rate * self.velocity_msg.linear.z


def main(args=None):
    rclpy.init(args=args)
    node = MultiLinInter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()