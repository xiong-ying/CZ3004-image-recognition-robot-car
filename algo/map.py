# TITLE: map.py
# AUTHOR: Xiong Ying
# CREATE DATE: 14/07/2022
# UPDATE DATE: 17/7/2022
# PURPOSE: To initialize a map, some functions to modify the map, and some variables about the robot



""" VARIABLES """



# MAP

# size is 20 by 20, initially empty, filled with 0

# size of map
MAP_SIZE = 20

# Initialize MAP that marks obstacles
MAP = [[0 for i in range(MAP_SIZE)] for i in range(MAP_SIZE)]

# Initialize MAP that marks accessibility
ACCESS_MAP = [[0 for i in range(MAP_SIZE)] for i in range(MAP_SIZE)]



# ROBOT
# Initial position and direction of robot
ROBOT_POSITION = (0,0,1)

#variables
ROBOT_BORDER = 3
ROBOT_WIDTH = 2
TURN_RADIUS = 4

# size of 1 move
ROBOT_STEP = 1



# OBSTACLES
# size
OBSTACLE_WIDTH = 1

# distance from robot to obstacle
CLEARANCE = 2



""" FUNCTIONS """



# FUNCTION: mark the obstacle on the console MAP
# REMARK: 1 = obstacle, 0 = empty
def markObstaclesOnMAP(obstacles):

    # Mark the obstacles position on the MAP as 0
    for obstacle in obstacles:
        MAP[obstacle[0]][obstacle[1]] = 1
    # End of function markObstaclesOnMAP(MAP, obstacles)



# FUNCTION: mark the accessbility of robot on the MAP
# REMARK: 1 = virtual obstacle, 0 = accessible
def markAccessOnMAP(obstacles):

    # Mark the obstacles position on the MAP as 0
    # Mark the bottom left 3 by 3 area of obstacle and the top 1 row and right 1 column as non-accessible
    for obstacle in obstacles:
        for i in range(ROBOT_BORDER+1):
            for j in range(ROBOT_BORDER+1):
                ACCESS_MAP[obstacle[0]-i+1][obstacle[1]-j+1] = 1
    # End of function markAccessOnMAP(obstacles)



# FUNCTION: print out the current MAP that marks obstacles
def printMap():

    # print title of MAP
    print("-----------------------------------------MAP-----------------------------------------")
    print(" y")
    print(" ↑")
    # y coordinate: rows, x coordinate: column
    # print y coordinates and MAP
    for i in range(MAP_SIZE): #y coordinate
        print (str(MAP_SIZE-1-i).zfill(2), " ", end=" ")
        for j in range(MAP_SIZE):  #x coordinate
            print(MAP[j][MAP_SIZE-1-i], end=" | ")
        print("")

    # print x coordinates axis
    print ("    ", end=' ')
    for i in range(MAP_SIZE):
        print(str(i).zfill(2), end='  ')
    print('→ x')
    print("\n")
    # End of Function printMAP()



# FUNCTION: print out the current MAP that marks accessibility / virtual obstacle
def printAccessMap():

    # print title of MAP
    print("-----------------------------------------MAP-----------------------------------------")
    print(" y")
    print(" ↑")
    # y coordinate: rows, x coordinate: column
    # print y coordinates and MAP
    for i in range(MAP_SIZE): #y coordinate
        print (str(MAP_SIZE-1-i).zfill(2), " ", end=" ")
        for j in range(MAP_SIZE):  #x coordinate
            print(ACCESS_MAP[j][MAP_SIZE-1-i], end=" | ")
        print("")

    # print x coordinates axis
    print ("    ", end=' ')
    for i in range(MAP_SIZE):
        print(str(i).zfill(2), end='  ')
    print('→ x')
    print("\n")
    # End of Function printAccessMap()



# FUNCTION: to check whether a node.position is an obstacle
# PARAMETER: node.position
# RETURN: boolean
# True = node is an obstacle
# False = node is NOT an obstacle
def checkIsObstacle(node_position):

    # within map
    if 0 <= node_position[0] < MAP_SIZE  and 0 <= node_position[1] < MAP_SIZE :
        
        # 1: obstacle
        if MAP[node_position[0]][node_position[1]] == 1:
            return True

        # 0: accessible
        else:
            return False

    # outside map
    else:

        # not an obstacle
        return False

    # End of function checkIsObstacle(node)



# FUNCTION: to check whether a node.position is accessible by the robot
# PARAMETER: node.position
# RETURN: boolean
# True = node is accessible by robot
# False = node is a virtual obstacle and NOT accessible by robot
def checkAccessible(node_position):

    # within map
    if 0 <= node_position[0] < MAP_SIZE  and 0 <= node_position[1] < MAP_SIZE :
        
        # 1: has virtual obstacle, not accessible
        if ACCESS_MAP[node_position[0]][node_position[1]] == 1:
            return False

        # 0: accessible
        else:
            return True

    # outside map
    else:
        return True

    # End of function checkAccessible(node)



# FUNCTION: check if can make a turns, with enough clearance
# PARAMETER: node.position, offset of x coordinate, offset of y coordinate
# RETURN: boolean
# True = have enough clearance
# False = not enough clearance
def checkTurnClearance(node_position, xOffset, yOffset):

    try:
        # check clearance with turn radius area
        for i in range(TURN_RADIUS):
            for j in range(TURN_RADIUS):

                # if encounter any obstacle
                if checkIsObstacle([node_position[0]+xOffset+i, node_position[1]+yOffset+j]) == True:

                    # return False, not enough clearance
                    return False

        # if didn't encounter any obstacle, return True, enough clearance
        return True

    # if out of index, means outside of map, return True, enough clearance
    except:
        return True

    # End of function checkTurnClearance(xOffset, yOffset)



# main() FUNCTION
def main():
    print("map.py")
    # Display the default initial map
    printMap()


if __name__ == '__main__':
    main()
