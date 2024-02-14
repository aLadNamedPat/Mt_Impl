import numpy as np
import math
# from DSM import DSM

class HungAlg:
    #Takes the distance square matrix, the initial robot positions, and the goal positions as inputs
    def __init__(self, dsm, rPos, gPos):
        self.sM = dsm
        self.rPos = rPos
        self.gPos = gPos
        
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
        # print(self.hM)
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

        #List representing the initial and final points of robots in a single movement action
        FM = []
        #List representing the goal points that have not yet been reached. This should be a n by 3
        #matrix representing all of the unselected matricies
        NM = []
        # print(self.markZeros)
        # print("starting positions\n", np.round(self.rPos, decimals = 1))
        # print("ending positions\n", np.round(self.gPos, decimals = 1))
        # print(len(self.gPos))
        print(self.markZeros)
        for i in range(len(self.markZeros)):
            if (self.markZeros[i][0] >= len(self.gPos)):
                #Maybe change the 0 to a 1
                FM.append([np.round(self.rPos[self.markZeros[i][1]], decimals = 3).tolist(), 
                           np.round(self.rPos[self.markZeros[i][1]], decimals = 3).tolist()])
            elif (self.markZeros[i][1] >= len(self.rPos)):
                NM.append(np.round(self.gPos[self.markZeros[i][0]], decimals = 3).tolist())
            else:
                FM.append([np.round(self.rPos[self.markZeros[i][1]], decimals = 3).tolist(), 
                           np.round(self.gPos[self.markZeros[i][0]], decimals = 3).tolist()])
        #We would like to return both the list of robot transitions (start to end)
        #As well as the list of robot goals not yet reached.
        distance_sum = 0
        for i in range(len(FM)):
            fm0 = np.array(FM[i][0])
            fm1 = np.array(FM[i][1])
            diff = fm0 - fm1
            distance_sum += np.linalg.norm(diff) ** 2
        print("distance sum is: ")
        print(distance_sum)
        return FM, NM
    
# starting_pos1 = [1.1, 3.2, 1.6]
# starting_pos2 = [1.1, 3.2, 1.6]
# starting_pos3 = [7.5, 3.0, 9.6]

# final_position1 = [7.738, 4.375, 1.849]
# final_position2 = [3.355, 9.318, 9.163]
# final_position3 = [4.541, 9.475, 0.32]

# a = DSM(1, 3, 1, [starting_pos1], [final_position1, final_position2, final_position3])
# b = DSM(1, 3, 1, [starting_pos3], [final_position1, final_position3, final_position2])

# a1, a2, a3 = a.distanceSquareMatrix()
# b1, b2, b3 = b.distanceSquareMatrix()

# print(a1)
# print(a2)
# print(a3)
# print(b1)
# print(b2)
# print(b3)
# a = HungAlg(a1, a2, a3)
# b = HungAlg(b1, b2, b3)

# print(a.hA()[0])
# print(b.hA()[0])