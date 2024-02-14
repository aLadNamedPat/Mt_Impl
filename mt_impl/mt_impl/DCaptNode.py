import rclpy
import numpy as np
from rclpy.node import Node
import ast
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

from mt_impl.DSM import DSM
from mt_impl.HungAlg import HungAlg

from msg_tst.msg import P2P

class DCaptNode(Node):
    def __init__(self):
        super().__init__('DCaptNode')
        ###Node initialization and setup here
        self.update_rate = 1
        # self.timer = self.create_timer(self.update_rate, self.update_position) 
        #Position is updated every second        
        ###All robots must have an ID to track where they are traveling to  
        self.robotID = int(self.declare_parameter('robot_id', 0).get_parameter_value().integer_value)
        self.numRobots = int(self.declare_parameter('totalRobots', 0).get_parameter_value().integer_value)
        self.starting_position = self.declare_parameter('startingPos').get_parameter_value().double_array_value
        self.goal_positions = ast.literal_eval(self.declare_parameter('goalPositions', '[[]]').get_parameter_value().string_value)
        self.commDistance = self.declare_parameter('commDist', 5).get_parameter_value().integer_value
        self.starting_position = np.round(np.array(self.starting_position), decimals=1).tolist()
        self.publisher_ = self.create_publisher(P2P, f"robot_information{self.robotID}", 10)

        #Publishing for rviz to keep track of
        self.publisher_2 = self.create_publisher(PoseStamped, f"Robot{self.robotID}", 10)

        self.current_position = self.starting_position

        self.final_time = 10
        self.current_time = 0
        self.reset_time = 0
        #Saves all the current positions
        self.oRP = {}
        #Dictionary that contains keys representing direct descendents being associated with links of secondary descendents
        self.robotLinks = {}
        #List representing all the current robots linked with the current robot (from direct descendents and their descendents)
        self.robotsAssociated = [self.robotID]
        #Current positions list of all robots in the associated robots list
        self.robotPositions = [self.current_position]
        for i in range(self.numRobots):
            if (i != self.robotID):
                self.create_subscription(
                    P2P,
                    f"robot_information{i}",
                    self.listener_callback,
                    10
                )
                      
        self.velocity_msg = Twist()
        self.tolerance = 0.01
        self.DSM = []
        self.firstRun = True
        self.runAlg()
        self.timer = self.create_timer(self.update_rate, self.update_position) #Position is updated every second that this is running

    def listener_callback(self, msg):
        self.oRP[msg.robot_id] = [ast.literal_eval(msg.current_position), ast.literal_eval(msg.goal_position)]
        self.robotsAssociated = []
        self.robotPositions = [self.current_position]
        if (self.distanceCalc(self.current_position, ast.literal_eval(msg.current_position)) < self.commDistance):
            #Updates the robots that are linked with the robot that was also linked with
            self.robotLinks[msg.robot_id] = msg.subscribed_robots
            self.get_logger().info(f'Robot {self.robotID} can hear {msg.robot_id}')
        else:
            #Removes the robot from the robot keys link if the robot is no longer in the communication distance
            if (msg.robot_id in self.robotLinks.keys()):
                try:
                    self.robotLinks.pop(msg.robot_id)
                except KeyError:
                    print("This robot link does not exist!")
        
        for key in self.robotLinks.keys():
            if (key not in self.robotsAssociated and key != self.robotID):
                self.robotsAssociated.append(key)
            #Goes through all the keys of the robot links (to append them to the robots associated)
            for robot_num in self.robotLinks[key]:
                if (robot_num not in self.robotsAssociated and robot_num != self.robotID):
                    self.robotsAssociated.append(robot_num)

        #This will now go through all of the robots that have been linked with the current robot
        #This will update the positions of the robots that have been associated with the current robot
        for robot_num in self.robotsAssociated:
            if (self.oRP.get(robot_num)[0] not in self.robotPositions and robot_num != self.robotID):
                self.robotPositions.append(self.oRP.get(robot_num)[0])
        # self.get_logger().info(f'Robot {self.robotID} has stored the following robot positions: {self.robotPositions}') 
        self.robotsAssociated.append(self.robotID)
        self.get_logger().info(f'The following robot positions are saved {self.robotsAssociated}')
        self.runAlg()
    
    def runAlg(self):
        #Run the hungarian algorithm here on all the robots that the robots have 
        a = DSM(len(self.robotsAssociated),
                len(self.goal_positions), 
                robot_radius = 0.5, 
                input_robots = self.robotPositions,
                input_goals = self.goal_positions)
        
        self.DSM, self.rM, self.gM = a.distanceSquareMatrix()
        hAlg = HungAlg(self.DSM, self.rM, self.gM)
        FM, NM = hAlg.hA()
        for i in range(len(FM)):
            if self.robotID == 0 or self.robotID == 1:
                self.get_logger().info(f'Robot positions are currently here: {self.robotPositions[i]}')
                self.get_logger().info(f'Robot starting position {FM[i][0]}, Robot ending position {FM[i][1]}')
            if (FM[i][0] == [round(num , 3) for num in self.current_position]):
                self.final_position = FM[i][1]
                # self.get_logger().info(f'Robot position has been updated!')
        self.setPosAndVel()
        
    def setPosAndVel(self):
        self.starting_x_pos = self.starting_position[0]
        self.starting_y_pos = self.starting_position[1]
        self.starting_z_pos = self.starting_position[2]

        self.final_x_pos = self.final_position[0]
        self.final_y_pos = self.final_position[1]
        self.final_z_pos = self.final_position[2]

        #Using twist to set the x, y, and z velocity from the start
        self.new_position[0] = self.current_position[0] * self.travel_rate()[0] + self.final_position[0] * self.travel_rate()[1]
        self.new_position[1] = self.current_position[1] * self.travel_rate()[0] + self.final_position[1] * self.travel_rate()[1]
        self.new_position[2] = self.current_position[2] * self.travel_rate()[0] + self.final_position[2] * self.travel_rate()[1]
    
    
    def travel_rate(self):
        #We assume that b(t) function for rate of robot travel 
        #over time is linear to start
        #We assume that it takes 5 seconds for the task to run to completion
        #This assumption is LARGE, but it is the same bounded constant
        #assumption taken by Dr. Turpin in his paper
        r1 = 1 - ((self.current_time - self.reset_time) / (self.final_time - self.reset_time))
        r2 = (self.current_time - self.reset_time) / (self.final_time - self.reset_time)
        return r1, r2

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
    
    def distanceCalc(self, arr1, arr2):
        arr1 = np.array(arr1)
        arr2 = np.array(arr2)
        dist = np.linalg.norm(arr2-arr1)
        return dist.tolist()
        
    def publish_position(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"  # Set the frame_id to "map"
        
        # Set the pose position and orientation
        pose.pose.position.x = self.current_position[0]
        pose.pose.position.y = self.current_position[1]
        pose.pose.position.z = self.current_position[2]
        pose.pose.orientation.w = 0.0
        
        self.publisher_2.publish(pose)


    def update_position(self):
        #The current position of the robot is updated here.
        #The update rate is basically the amount of time that has elapsed since
        #the last run.
        # self.get_logger().info(f'Current Position: {self.current_position}')
        if (self.firstRun):
            # self.get_logger().info(f'Robot {self.robotID} has sent the final position: {self.final_position} and the following subscribed_robots {self.robotsAssociated}') 
            # self.get_logger().info(f'Robot is currently at position {self.current_position}')
            self.firstRun = False
            msg = P2P()
            msg.robot_id = self.robotID
            msg.current_position = str(self.current_position)
            msg.goal_position = str(self.final_position) 
            msg.subscribed_robots = self.robotsAssociated

            self.publish_position()
            self.publisher_.publish(msg)

        elif (abs(self.final_position[0] - self.current_position[0]) < self.tolerance):
            self.velocity_msg.linear.x = 0
            self.velocity_msg.linear.y = 0
            self.velocity_msg.linear.z = 0
            self.get_logger().info(f'Robot {self.robotID} has reached goal at postion: {self.current_position}')
            self.timer.cancel()
            
            self.publish_position()
            self.publisher_.publish(msg)
            
        else:
            # self.get_logger().info(f'Robot {self.robotID} has sent the final position: {self.final_position} and the following subscribed_robots {self.robotsAssociated}') 
            # self.get_logger().info(f'Robot is currently at position {self.current_position}')

            self.current_position[0] += self.update_rate * self.velocity_msg.linear.x
            self.current_position[1] += self.update_rate * self.velocity_msg.linear.y
            self.current_position[2] += self.update_rate * self.velocity_msg.linear.z

            self.publisher_.publish(msg)
            self.publish_position()
            self.get_logger().info(f'Robot {self.robotID} has sent a message')

        self.current_time += self.update_rate


def main(args=None):
    rclpy.init(args=args)
    node = DCaptNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()