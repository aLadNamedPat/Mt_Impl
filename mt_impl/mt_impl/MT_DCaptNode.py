import rclpy
import numpy as np
from rclpy.node import Node
import copy
import ast
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

from mt_impl.DSM import DSM
from mt_impl.HungAlg import HungAlg

from msg_tst.msg import P2P
from msg_tst.srv import PathData

class MT_DCaptNode(Node):
    def __init__(self):
        super().__init__('MT_DCaptNode')
        ###Node initialization and setup here
        self.update_rate = 0.01

        #Final time - Time that all movements should be completed
        self.final_time = 10

        #Current time - Time that has elapsed so far
        self.current_time = 0

        #Reset time - Time that the robot last reset
        self.reset_time = 0

        # self.timer = self.create_timer(self.update_rate, self.update_position) 
        #Position is updated every second        
        ###All robots must have an ID to track where they are traveling to  
        self.robotID = int(self.declare_parameter('robot_id', 0).get_parameter_value().integer_value)
        self.numRobots = int(self.declare_parameter('totalRobots', 0).get_parameter_value().integer_value)
        self.starting_position = self.declare_parameter('startingPos').get_parameter_value().double_array_value
        self.goal_position = self.declare_parameter('goalPositions').get_parameter_value().double_array_value
        self.commDistance = self.declare_parameter('commDist', 5).get_parameter_value().integer_value
        self.starting_position = np.round(np.array(self.starting_position), decimals=1).tolist()

        #Set the final position
        self.final_position = self.goal_position
        #Set the current position to be the starting posit
        # self.current_position = self.starting_position

        self.current_position = copy.deepcopy(self.starting_position)

        #Set the new positions that will eventually be saved
        self.new_position = self.starting_position

        #Dictionary that contains keys representing direct descendents being associated with links of secondary descendents
        self.robotLinks = {}
        #List representing all the current robots linked with the current robot (from direct descendents and their descendents)
        self.robotsAssociated = [self.robotID]
        #Current positions list of all robots in the associated robots list
        self.robotPositions = [self.current_position]

        #Publishing position data for rviz2 to keep track of
        self.publisher_2 = self.create_publisher(PoseStamped, f"Robot{self.robotID}", 10)

        #Set up the service
        self.service = self.create_service(PathData, f'robot_path{self.robotID}', self.handle_trade_path_request) 

        #Set up the clients
        self.clientDicts = {}
        for i in range(self.numRobots):
            if (i != self.robotID):
                self.clientDicts[f'client{i}'] = self.create_client(PathData, f'robot_path{i}')
                while not self.clientDicts[f'client{i}'].wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Service not available, waiting again...')



        self.velocity_msg = Twist()
        self.tolerance = 0.0001
        self.DSM = []
        self.firstRun = True
        self.trade_in_progress = False

        self.timer = self.create_timer(self.update_rate, self.update_position) #Position is updated every second that this is running
    

    # def listener_callback(self, msg):
    #     self.oRP[msg.robot_id] = [ast.literal_eval(msg.current_position), ast.literal_eval(msg.goal_position)]
    #     self.robotsAssociated = []
    #     self.robotPositions = [self.current_position]
    #     if (self.distanceCalc(self.current_position, ast.literal_eval(msg.current_position)) < self.commDistance):
    #         #Updates the robots that are linked with the robot that was also linked with
    #         self.robotLinks[msg.robot_id] = msg.subscribed_robots
    #         self.get_logger().info(f'Robot {self.robotID} can hear {msg.robot_id}')
    #     else:
    #         #Removes the robot from the robot keys link if the robot is no longer in the communication distance
    #         if (msg.robot_id in self.robotLinks.keys()):
    #             try:
    #                 self.robotLinks.pop(msg.robot_id)
    #             except KeyError:
    #                 print("This robot link does not exist!")
        
    #     for key in self.robotLinks.keys():
    #         if (key not in self.robotsAssociated and key != self.robotID):
    #             self.robotsAssociated.append(key)
    #         #Goes through all the keys of the robot links (to append them to the robots associated)
    #         for robot_num in self.robotLinks[key]:
    #             if (robot_num not in self.robotsAssociated and robot_num != self.robotID):
    #                 self.robotsAssociated.append(robot_num)

    #     #This will now go through all of the robots that have been linked with the current robot
    #     #This will update the positions of the robots that have been associated with the current robot
    #     for robot_num in self.robotsAssociated:
    #         if (self.oRP.get(robot_num)[0] not in self.robotPositions and robot_num != self.robotID):
    #             self.robotPositions.append(self.oRP.get(robot_num)[0])
    #     # self.get_logger().info(f'Robot {self.robotID} has stored the following robot positions: {self.robotPositions}') 
    #     self.robotsAssociated.append(self.robotID)
    #     self.get_logger().info(f'The following robot positions are saved {self.robotsAssociated}')
    #     self.runAlg()
        

    def setPosAndVel(self):
        # self.starting_x_pos = self.starting_position[0]
        # self.starting_y_pos = self.starting_position[1]
        # self.starting_z_pos = self.starting_position[2]

        # self.final_x_pos = self.final_position[0]
        # self.final_y_pos = self.final_position[1]
        # self.final_z_pos = self.final_position[2]

        # #Using twist to set the x, y, and z velocity from the start
        # self.new_position[0] = self.current_position[0] * self.travel_rate()[0] + self.final_position[0] * self.travel_rate()[1]
        # self.new_position[1] = self.current_position[1] * self.travel_rate()[0] + self.final_position[1] * self.travel_rate()[1]
        # self.new_position[2] = self.current_position[2] * self.travel_rate()[0] + self.final_position[2] * self.travel_rate()[1]
        
        self.current_position[0] = self.starting_position[0] * self.travel_rate()[0] + self.final_position[0] * self.travel_rate()[1]
        self.current_position[1] = self.starting_position[1] * self.travel_rate()[0] + self.final_position[1] * self.travel_rate()[1]
        self.current_position[2] = self.starting_position[2] * self.travel_rate()[0] + self.final_position[2] * self.travel_rate()[1]

    
    def travel_rate(self):
        #We assume that b(t) function for rate of robot travel 
        #over time is linear to start
        #We assume that it takes 5 seconds for the task to run to completion
        #This assumption is LARGE, but it is the same bounded constant
        #assumption taken by Dr. Turpin in his paper
        r1 = 1 - ((self.current_time - self.reset_time) / (self.final_time - self.reset_time))
        r2 = (self.current_time - self.reset_time) / (self.final_time - self.reset_time)
        return r1, r2
    
    #Code should be structured such that each client is sending a request every time its position is updated to all other robots
    #*When a robot sends a request to another robot, it will be communicating and waiting for a response such that its path will have changed
    #Then if the path has changed, the robot will receive a request from another robot asking to switch again
    #All other robots receive these requests
    #Other robots can reject immediately if their positions are minimum of communication distance away

    #This is to send a request to the server
    def client_trade_proposal(self, num):
        if not self.trade_in_progress:
            self.trade_in_progress = True
            self.req = PathData.Request()
            self.req.robot_id = self.robotID
            self.req.current_location = self.current_position
            self.req.goal_location = self.final_position
            self.req.robots_associated = self.robotsAssociated
            future = self.clientDicts[f'client{num}'].call_async(self.req)
            self.get_logger().info(f'Robot trade initiated between {self.robotID} and {num}')
            future.add_done_callback(self.handle_service_response)
        else:
            # self.get_logger().info(f'Robot trade request from {self.robotID} blocked for {num}')
            pass
    #This is the server's response
    #If the trade is successful, then the service
    #should be removed from the list of services that the client is 
    #in communication with
    def handle_service_response(self, future):
        try: 
            response = future.result()
            accepted = response.accepted
            new_goal_location = response.switched_goal_location
        except Exception as e:
            pass
        finally:
            self.trade_in_progress = False

        if accepted:
            self.final_position = new_goal_location
            self.reset_time = self.current_time
            self.starting_position = copy.deepcopy(self.current_position)

    #request is what is received from the client
    #response is what is sent back by the service
    def handle_trade_path_request(self, request, response):
        crid = request.robot_id
        cril = np.array(request.current_location)
        crgl = np.array(request.goal_location)
        current_location = np.array(self.current_position)
        final_position = np.array(self.final_position)
        if (self.trade_in_progress):
            self.get_logger().info(f"Trade blocked between {request.robot_id} and {self.robotID}")
        if ((np.linalg.norm(current_location - cril) < self.commDistance and not self.trade_in_progress)):
            # self.get_logger().info(f'Robot {self.robotID} is running!')
            csd = np.linalg.norm(final_position - current_location) ** 2 + np.linalg.norm(crgl - cril) ** 2 #current squared distance
            nsd = np.linalg.norm(final_position - cril) ** 2 + np.linalg.norm(crgl - current_location) ** 2 #new squared distance 
            if (nsd < csd):
                response.accepted = True
                response.switched_goal_location = self.final_position
                self.final_position = crgl
                self.reset_time = self.current_time
                self.starting_position = copy.deepcopy(self.current_position)
                self.get_logger().info(f'Robot {self.robotID} has switched goals with {crid}')
            else:
                response.accepted = False
                response.switched_goal_location = []
        else:
            response.accepted = False
            response.switched_goal_location = []
        return response
    

    def make_all_trades(self):
        for i in range(self.numRobots):
            if (i != self.robotID):
                # self.get_logger().info(f"Trade proposal sent to {i} from {self.robotID}")
                self.client_trade_proposal(i)

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
        if (self.firstRun):
            self.firstRun = False
            self.get_logger().info(f'Final position is {self.final_position}')
            self.publish_position()
            self.make_all_trades()
        elif (abs(self.final_position[0] - self.current_position[0]) < self.tolerance 
              and abs(self.final_position[1] - self.current_position[1]) < self.tolerance
              and abs(self.final_position[2] - self.current_position[2]) < self.tolerance):
            self.velocity_msg.linear.x = 0
            self.velocity_msg.linear.y = 0
            self.velocity_msg.linear.z = 0
            self.get_logger().info(f'Robot {self.robotID} has reached goal at postion: {self.current_position}')
            self.timer.cancel()
            self.publish_position()
            self.make_all_trades()
        else:
            # self.get_logger().info(f'Current position is {self.current_position}')
            # self.get_logger().info(f'Robot {self.robotID} final position is {self.final_position}')
            self.publish_position()
            self.make_all_trades()
        self.current_time += self.update_rate
        self.setPosAndVel() #We want to update the new position after the final position has been updated

def main(args=None):
    rclpy.init(args=args)
    node = MT_DCaptNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()