import numpy as np
import math
#We want to generate robots and goals in a 3 dimension space

class Map:
    def __init__(self, robots, goals, robot_radius = 1): #number of robots and number of goals
        self.robots = robots
        self.goals = goals
        self.robot_radius = robot_radius
        self.gPos = np.zeros((goals, 3)) #Goal positions
        self.rPos = np.zeros((robots, 3)) #Robot positions
        self.dM = np.zeros((self.robots, self.goals)) #Distance squared matrix
        self.aM = np.zeros((self.robots, self.goals)) #Assignment matrix
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
        if (self.robots > self.goals):
            dum_row = np.full(self.robots, lgVal)
            sM = np.vstack((sM, dum_row))
        elif (self.robots < self.goals):
            dum_col = np.full((self.goals, 1), lgVal)
            sM = np.hstack((sM, dum_col))
        # print(sM)
        # print(lgVal)
        #The number of robots is the rows
        #The number of goals is the columns
        self.sM = sM
        return gDM, rDM, sM
    
    def hA_helper(self): #Hungarian algorithm helper function
    #This function subtracts the minimum value of each ROW and COLUMN from rows and columns
        self.hM = np.copy(self.sM)
        for row in range(self.hM.shape[0]):
            self.hM[row] = self.hM[row] - np.min(self.hM[row])
        
        for col in range(self.hM.shape[1]):
            self.hM[:,col] = self.hM[:,col] - np.min(self.hM[:, col])
        
        return self.hM
    
    #The point of this function is to determine if rows and columns are covered by zeros after this is complete
    def min_zeros(self): #Function to label the min number of rows to mark that contain a zero value
        min_row = [999999, -1]
        for row in range(self.hM.shape[0]): #Goes through all of the rows
            if (np.sum(self.tFMCopy[row] == True)) > 0 and min_row[0] > np.sum(self.tFMCopy[row] == True):
                #Checks if the number of zeros in the row is greater than 1
                #Second if statement checks if the number of zeros is minimized
                min_row = [np.sum(self.tFMCopy[row] == True), row]
            
        #zero_index is set to the index of the column where the zero is first found in the row
        zero_col = np.where(self.tFMCopy[min_row[1]] == True)[0][0] 
        #markZero matrix appends the minimum row and the column index
        self.markZeros.append((min_row[1], zero_col)) 
        #The row where the minimum number of zeros was found is set to all false
        self.tFMCopy[min_row[1], :] = False
        #The column where the zero_index was found is set to all false
        self.tFMCopy[:, zero_col] = False

    def mark_matrix(self):
        #Creates a new copy of self.hM that is determined based on if the value equals 0
        self.tFM = (self.hM == 0)
        self.tFMCopy = self.tFM.copy()

        #While true exists in self.tFMCopy, the function will continue running and saving
        #the rows and columns that are covered by the zeros
        while (True in self.tFMCopy):
            self.min_zeros()
    
        #Recording the row and column indexes seperately.
        marked_zero_row = []
        marked_zero_col = []
        for i in range(len(self.markZeros)):
            #Records the value of the rows
            marked_zero_row.append(self.markZeros[i][0])
            #Records the value of the columns
            marked_zero_col.append(self.markZeros[i][1])
        
        #The non-marked rows list is composed of all the rows that were unmarked
        non_marked_row = list(set(range(self.hM.shape[0])) - set(marked_zero_row))
        #Create a list to track the number of rows that have not been marked,
        #but could be marked by columns 
        self.marked_cols = []
        check_switch = True
        while check_switch:
            check_switch = False
            #Searching through all the non_marked_rows
            for i in range(len(non_marked_row)):
                row_array = self.tFM[non_marked_row[i], :]
                #Searching through the unmarked_rows for columns with 0s
                for j in range(row_array.shape[0]):
                    if row_array[j] == True and j not in self.marked_cols:
                        self.marked_cols.append(j)
                        check_switch = True
            #If the zero is accounted for by a marked column, then it is removed from the marked rows
            for row_num, col_num in self.markZeros:
                    if row_num not in non_marked_row and col_num in self.marked_cols:
                        non_marked_row.append(row_num)
                        check_switch = True

        # print("this is new non_marked_rows")
        # print(non_marked_row)
        self.marked_rows = list(set(range(self.hM.shape[0])) - set(non_marked_row))

    #If the matrix does not initially satisfy the matrix properties, matrix adjustment is needed
    def adjust_matrix(self):
        # print(self.marked_rows)
        # print(self.marked_cols)
        non_zero_element = []
        for row in range(len(self.hM)):
            if row not in self.marked_rows:
                for col in range(len(self.hM[row])):
                    if col not in self.marked_cols:
                        non_zero_element.append(self.hM[row][col])
        #Find the minimum of all the non-zero elements
        min_num = min(non_zero_element)
        # print(min_num)

        #If the row is not in any marked columns or marked rows, the number is subtracted by the min
        for row in range(len(self.hM)):
            if row not in self.marked_rows:
                for col in range(len(self.hM[row])):
                    if col not in self.marked_cols:
                        self.hM[row, col] =  self.hM[row, col] - min_num
        
        #If the row is in a marked row or marked column, then the number adds the min
        for row in range(len(self.marked_rows)):
            for col in range(len(self.marked_cols)):
                self.hM[self.marked_rows[row], self.marked_cols[col]] = self.hM[self.marked_rows[row], self.marked_cols[col]] + min_num

        # print(self.hM)

            
    def hA(self): #Hungarian algorithm
        self.generateGoals()
        self.generateRobots()
        b = self.distanceSquareMatrix()
        print(b[2])
        self.hA_helper()
        n = self.hM.shape[0]
        count_zero_lines = 0
        while count_zero_lines < n:
            self.markZeros = []
            self.mark_matrix()
            count_zero_lines = len(self.marked_cols) + len(self.marked_rows)

            if count_zero_lines < n:
                self.adjust_matrix()

        #We would like to return a 3 dimensional list containing
        #[[[Starting Pos 1], [Ending Pos 1]], [[Starting Pos 2], [Ending Pos 2]]]
        #We would like to return a 3 dimensional list containing
        #[[[Starting Pos 1], [Ending Pos 1]], [[Starting Pos 2], [Ending Pos 2]]]
        FM = []
        print(self.markZeros)
        print("starting positions\n", np.round(self.rPos, decimals = 1))
        print("ending positions\n", np.round(self.gPos, decimals = 1))
        print(len(self.gPos))
        for i in range(len(self.markZeros)):
            if (self.markZeros[i][1] >= self.goals):
                FM.append([np.round(self.rPos[self.markZeros[i][0]], decimals = 1).tolist(), 
                           np.round(self.rPos[self.markZeros[i][0]], decimals = 1).tolist()])
            elif (self.markZeros[i][0] >= self.robots):
                pass
            else:
                FM.append([np.round(self.rPos[self.markZeros[i][0]], decimals = 1).tolist(), 
                           np.round(self.gPos[self.markZeros[i][1]], decimals = 1).tolist()])
        return FM