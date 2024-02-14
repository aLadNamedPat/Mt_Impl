import rclpy
import numpy as np
import ast
from rclpy.node import Node
from mt_impl.HungAlg import HungAlg
from mt_impl.DSM import DSM
from msg_tst.msg import RobotPath, TaskCompletion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist


class ControlNode(Node):
    def __init__(self):
        super().__init__('ControlNode')
        # Node initialization and setup here
        self.publisher_ = self.create_publisher(RobotPath, "robot_paths", 10)
        self.subscription = self.create_subscription(
            TaskCompletion,
            'task_completion',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscription Channel Openned')
        ###Receive the distance square matrix, robot starting values,
        ###and goal positions from the launch file
        self.sDM = self.declare_parameter("sDM", "[]").get_parameter_value().string_value
        self.rM = self.declare_parameter("rPM", "[]").get_parameter_value().string_value
        self.gM = self.declare_parameter("gPM", "[]").get_parameter_value().string_value

        ###Turn the strings into lists, and the turn those into arrays
        ###to input into the hungarian algorithm
        self.sDM = np.array(ast.literal_eval(self.sDM))
        self.rM = np.array(ast.literal_eval(self.rM))
        self.gM = np.array(ast.literal_eval(self.gM))
        
        self.get_logger().info(f'Logged square distance matrix {self.sDM}')

        ###Run the hungarian algorithm using the data that was received
        hAlg = HungAlg(self.sDM, self.rM, self.gM)

        ###Generate two matricies, one is assigned and one is not
        self.assigned, self.not_assigned = hAlg.hA()

        ###Create a completion tracker to track whether robots are finished
        ###With their tasks
        self.completion_tracker = [False for _ in range(len(self.rM))]
        self.send_assignments()

    #used to send assignments to robots after assignments are received
    def send_assignments(self):
        self.assignedGoals = []  
        for i in range(len(self.assigned)):
            msg = RobotPath()
            msg.robot_id = str(i)
            msg.path = str([self.assigned[i][0], self.assigned[i][1]])
            if (self.assigned[i][1] != self.assigned[i][0]):
                self.assignedGoals.append(self.assigned[i][1])
            self.publisher_.publish(msg)
            self.get_logger().info(f'Sending path {self.assigned[i][1]} to robot {i} at {self.assigned[i][0]}')

    def listener_callback(self, msg):
        if (msg.completed):
           self.completion_tracker[int(msg.robot_id)] = True
        
        update_val = True
        for i in range(len(self.completion_tracker)):
            if (not self.completion_tracker[i]):
                update_val = False
        
        if (update_val):
            self.update_alg()
            update_val = False
            self.completion_tracker = [False for _ in range(len(self.rM))]
            
    def update_alg(self):
        if (len(self.not_assigned) == 0):
            self.get_logger().info('All robots have finished traveling!')
        else:
            # self.get_logger().info(f'Number of robots {len(self.rM)}')
            # self.get_logger().info(f'Number of end positions {len(self.not_assigned)}')
            # self.get_logger().info(f'Current robot positions {self.assignedGoals}')
            # self.get_logger().info(f'Current robots unassigned {self.not_assigned}')
            dsm = DSM(len(self.rM), len(self.not_assigned), robot_radius=0.5, input_robots=self.assignedGoals, input_goals=self.not_assigned)
            self.sDM, self.rM, self.gM = dsm.distanceSquareMatrix()
            hAlg = HungAlg(self.sDM, self.rM, self.gM)
            self.assigned, self.not_assigned = hAlg.hA()
            self.get_logger().info(f'assigned robots {self.assigned}')
            if (len(self.assigned) == 0):
                self.get_logger().info('All robots have finished traveling!')
            else:
                self.send_assignments()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()