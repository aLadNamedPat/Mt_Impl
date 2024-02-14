import numpy as np
import math
#We want to generate robots and goals in a 3 dimension space

class DSM:
    def __init__(self, robots, goals, robot_radius = 1, input_robots = [], input_goals = []): #number of robots and number of goals
        self.robots = robots
        self.goals = goals
        self.robot_radius = robot_radius
        if ((len(input_robots) == robots  and len(input_goals) == goals) and
            (len(input_robots[0]) == 3 and len(input_goals[0]) == 3)):
            #IF proper inputs for robot positions and goals are given, then we do not need
            #to generate new initial positions or goal positions
            self.gPos = np.array(input_goals)
            self.rPos = np.array(input_robots)
        else:
            #IF proper inputs for robot positions and goals are NOT given, then we do need
            #to generate new initial positions and goal positions
            self.gPos = np.zeros((goals, 3)) #Goal positions
            self.rPos = np.zeros((robots, 3)) #Robot positions
            self.generateGoals()
            self.generateRobots()
        self.markZeros = []
        self.tFMCopy = []
    #We constrain the space of the environment (where the robot can travel to) 
    #to be 10 wide, 10 long, and 10 tall (all values are the same for simplicity of rand function)
    
    def generateGoals(self):
        #Generate the first goal position to begin
        newPoint = np.random.rand(3, 1) * 10
        for i in range(self.goals):
            for j in range(i):
                currPoint = self.gPos[j]
                # currPoint = np.reshape(self.gPos[j], (3,1))
                newPoint = np.random.rand(3, 1) * 10
                distance = np.linalg.norm(currPoint - newPoint ** .5)
                while (distance < (2 * math.sqrt(2) * self.robot_radius)):
                    newPoint = np.random.rand(3, 1) * 10
                    distance = np.linalg.norm(currPoint - newPoint ** .5)
            #Once it is confirmed that the new points generated are 2sqrt(2) away from the previous generated
            #goals, the new point is added to the set
            self.gPos[i] = np.reshape(newPoint, (1, 3))
        return self.gPos

    def generateRobots(self):
        #The same as generating goals 
        newPoint = np.random.rand(3, 1) * 10
        for i in range(self.robots):
            for j in range(i):
                currPoint = self.rPos[j]
                newPoint = np.random.rand(3, 1) * 10
                distance = np.linalg.norm(currPoint - newPoint)
                while (distance < (2 * math.sqrt(2) * self.robot_radius)):
                    newPoint = np.random.rand(3, 1) * 10
                    distance = np.linalg.norm(currPoint - newPoint)
            #Once it is confirmed that the new points generated are 2sqrt(2) away from the previous generated
            #goals, the new point is added to the set
            self.rPos[i] = np.reshape(newPoint, (1, 3))
        return self.rPos

    def distanceSquareMatrix(self):
        #Generate goals
        gDM = np.kron(self.gPos, np.ones((self.robots, 1))) #Generate a point matrix for goals
        gDM = np.reshape(gDM, (len(self.gPos), self.robots * 3))
        # print("goal matrix", gDM)
        rDM = np.kron(self.rPos, np.ones((self.goals, 1)))
        rDM = np.reshape(rDM, (self.robots, len(self.gPos), 3))
        rDM = rDM.transpose(1,0,2)
        rDM = np.reshape(rDM, (len(self.gPos), self.robots * 3)) #Generate a point matrix for robots
        # print("robot matrix", rDM)
        sM = (rDM - gDM)
        sM = np.reshape(sM, (self.goals, self.robots, 3))
        sM = np.sum(sM ** 2, axis = 2) #Produce the sum of distance squares matrix
        lgVal = np.max(sM)
        lgVal += 1
        robotNums = self.robots
        goalNums = self.goals
        if (robotNums > goalNums):
            while(robotNums > goalNums):
                dum_row = np.full(self.robots, lgVal)
                sM = np.vstack((sM, dum_row))
                goalNums += 1
        elif (self.robots < self.goals):
            while(robotNums < goalNums):
                dum_col = np.full((self.goals, 1), lgVal)
                sM = np.hstack((sM, dum_col))
                robotNums += 1
        # print(sM)
        # print(lgVal)
        #The number of robots is the rows
        #The number of goals is the columns
        self.sM = sM
        return self.sM, self.rPos, self.gPos

# a = DSM(3, 2, 0.5)
# a = DSM(3, 2, 0.5, [[3.6, 2.1, 7.1], [8.7, 3.2, 7.1], [4.7, 0.9, 9.9]], [[9.6, 7.4, 7.8], [2.4, 3.6, 0.3]])
# print(a.distanceSquareMatrix()[0])
# print(a.distanceSquareMatrix()[1])
# print(a.distanceSquareMatrix()[2])