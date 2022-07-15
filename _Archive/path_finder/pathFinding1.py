# title: pathFinding.py
# author: Xiong Ying
# date: 09/07/2022 Saturday
# purpose: To find the Hamiltonian Path for the ROBOT to traverse through 5 obstacles

import math
import time

# Initialize MAP 20 by 20, filled with 1
# MAP representation:
# 1 = accessible, 0 = non-accesible
MAP = [[1 for i in range(20)] for i in range(20)]

# ROBOT Initialize position and angle
ROBOT = [0, 0, "N"]

# Other Variables
ROBOT_WIDTH = 3
OBSTACLE_WIDTH = 1
CLEARANCE = 2
ROBOT_STEP = 1
######### TODO: verify whether the turn is within 20 by 20 cm area ###########
TURN_RADIUS = 3


# Function: calculate the ROBOT's desired location and facing angle, based on the obstacles' location and direction
# return: list[[]]
def findVertex(obstacles):
    # Initialize empty list
    vertex = []
    # For each obstacles, calculate the desired ROBOT position and angle
    for i in range(len(obstacles)):
        match obstacles[i][2]:
            case "N":
                vertex.append([obstacles[i][0] - 1, obstacles[i][1] + OBSTACLE_WIDTH + CLEARANCE, "S"])
            case "S":
                vertex.append([obstacles[i][0] - 1, obstacles[i][1] - CLEARANCE - ROBOT_WIDTH, "N"])
            case "W":
                vertex.append([obstacles[i][0] - CLEARANCE - ROBOT_WIDTH, obstacles[i][1] - 1, "E"])
            case "E":
                vertex.append([obstacles[i][0] + OBSTACLE_WIDTH + CLEARANCE, obstacles[i][1] - 1, "W"])
            case default:
                return "Error"

    return vertex
    # print (vertex)
    # End of function findVertex(obstacles)


# Function: mark the obstacle on the console MAP
def markObstaclesOnMAP(obstacles):

    # Mark the obstacles position on the MAP as 0
    for i in range(len(obstacles)):
        MAP[obstacles[i][0]][obstacles[i][1]] = 0
    # End of function markObstaclesOnMAP(MAP, obstacles)


# Function: print out the current MAP
def printMAP():

    # print title of MAP
    print("-----------------------------------------MAP-----------------------------------------")
    print(" y")
    print(" ↑")
    # y coordinate: rows, x coordinate: column
    # print y coordinates and MAP
    for i in range(20): #y coordinate
        print (str(19-i).zfill(2), " ", end=" ")
        for j in range(20):  #x coordinate
            print(MAP[j][19-i], end=" | ")
        print("")

    # print x coordinates axis
    print ("    ", end=' ')
    for i in range(20):
        print(str(i).zfill(2), end='  ')
    print('→ x')
    print("\n")
    # End of Function printMAP(MAP)


# Function: Check if the node is accessible
# return: boolean
def checkAccessible(node):
    if node[0] < 0 or node[0] > (19-ROBOT_WIDTH+1) or node[1] < 0 or node[1] > (19-ROBOT_WIDTH+1):
        print("not accessible because out of border")
        return False
    else:
    # if 0 <= node[0] <= (19-ROBOT_WIDTH+1) and 0 <= node[1] <= (19-ROBOT_WIDTH+1):
        for i in range(ROBOT_WIDTH):
            for j in range(ROBOT_WIDTH):
                #print("for i = ", i, " , j = ", j)
                #print("The node (", node[0]+i, ", ", node[1]+j, ") on the map is ", MAP[node[0]+i][node[1]+j])
                #print("Type of MAP[][] is ", type(MAP[node[0]+i][node[1]+j]))
                #print(" == 0? ", MAP[node[0]+i][node[1]+j] == 0)
                if MAP[node[0]+i][node[1]+j] == 0:
                    #print("before return")
                    return False
                    #
                    print("after return")
                    break
        return True

    # End of function checkAccessible(MAP, node)

# Function: Check if the node is an obstacle
# return: boolean
def isObstacle(node):
    if 0 <= node[0] <= 19 and 0 <= node[1] <= 19:
        if MAP[node[0]][node[1]] == 1:
            return True
        else:
            return False
    else:
        # print("check Accessible: false")
        return False
    # End of function isObstacle(node)


# Function: check if can make a turns, with enough clearance
# return: boolean
def checkTurnClearance(node, xOffset, yOffset):
    for i in range(TURN_RADIUS):
        for j in range(TURN_RADIUS):
            #print("i th loop: ", i, " ; j th loop: ", j)
            #print("is it accessible? ", isObstacle([node[0]+xOffset+i, node[1]+yOffset+j]))
            if isObstacle([node[0]+xOffset+i, node[1]+yOffset+j]) == False:
                return False
    return True
    # End of function checkTurnClearance(xOffset, yOffset)


# Function: check if there is obstacle in front 3 by 3 area
# return: boolean
def checkFrontGotObstacle(node):
    print("Start of checkFrontGotObstacle(node)")
    match node[2]:
        case "N":
            if checkTurnClearance(node, 0, 3) == False:
                print("checkTurnClearance(node, 0, 3): ", checkTurnClearance(node, 0, 3))
                return True
            else:
                return False
        case "S":
            if checkTurnClearance(node, 0, -3) == False:
                print("checkTurnClearance(node, 0, -3): ", checkTurnClearance(node, 0, -3))
                return True
            else:
                return False
        case "W":
            if checkTurnClearance(node, -3, 0) == False:
                print("checkTurnClearance(node, -3, 0): ", checkTurnClearance(node, -3, 0))
                return True
            else:
                return False
        case "E":
            if checkTurnClearance(node, 3, 0) == False:
                print("checkTurnClearance(node, 3, 0): ", checkTurnClearance(node, 3, 0))
                return True
            else:
                return False

    # End of function checkFrontGotObstacle(node)


# Function: find a Hamiltonian path of all Vertex using nearest neighbour greedy heuristic algorithm
def findGreedyPath(ROBOT, vertex):
    # Initialize a local list to store the shortest distance from current node to each vertex
    distance = [100 for i in range (len(vertex))] # 100 is just a random super big number
    # a local variable foor distance comparison
    nearest = 100
    # a local list to store the visited flag for 5 vertex
    visited = [0 for i in range(len(vertex))]
    # record the current node position
    currentNode = ROBOT
    # empty list
    plannedPath = []

    # loop 5 times to find the sequence of 5 vertex in planned path
    for i in range(len(vertex)):
        # for each node in the planned path, compute the distance from current node

        # for debug
        #print("i = ", i)
        #print(plannedPath)

        for j in range(len(vertex)):

            # for debug
            #print("j = ", j)

            if visited[j] == 0:
                # compute the distance from current node to jth node
                distance[j] = math.sqrt(pow(abs(currentNode[0] - vertex[j][0]), 2) + pow(abs(currentNode[1] - vertex[j][1]), 2))
        # find the node with nearest distance from current node
        nearest = min(distance)
        for k in range(len(distance)):

            # for debug
            # print("k = ", k)

            if distance[k] == nearest and visited[k] == 0:
                # append the nearest node to plannedPath
                plannedPath.append(vertex[k])
                # set visited flag to 1
                visited[k] = 1
                # set kth node distance to a random big number, so it won't interfere subsequent calculation
                distance[k] = 100

    return plannedPath
    # End of function findGreedyPath(vertex, ROBOT)


# Function: moveForward 1 step
# Return: ROBOT = [x coordinate, y coordinate, angle]
def moveForward(ROBOT):

    robotNextPos = []
    match ROBOT[2]:
        case "N":
            robotNextPos.append(ROBOT[0])
            robotNextPos.append(ROBOT[1] + ROBOT_STEP)
            robotNextPos.append(ROBOT[2])
        case "S":
            robotNextPos.append(ROBOT[0])
            robotNextPos.append(ROBOT[1] - ROBOT_STEP)
            robotNextPos.append(ROBOT[2])
        case "W":
            robotNextPos.append(ROBOT[0] - ROBOT_STEP)
            robotNextPos.append(ROBOT[1])
            robotNextPos.append(ROBOT[2])
        case "E":
            robotNextPos.append(ROBOT[0] + ROBOT_STEP)
            robotNextPos.append(ROBOT[1])
            robotNextPos.append(ROBOT[2])
    if checkAccessible(robotNextPos) == True:
        # print("Move Forward: valid move")
        return robotNextPos
    else:
        # print("Move Forward: cannot make the move, out of border")
        return ROBOT
    # End of function moveForward(ROBOT)


# Function: moveForward 1 step
# Return: ROBOT = [x coordinate, y coordinate, angle]
def moveBackward(ROBOT):

    robotNextPos = []
    match ROBOT[2]:
        case "N":
            robotNextPos.append(ROBOT[0])
            robotNextPos.append(ROBOT[1] - ROBOT_STEP)
            robotNextPos.append(ROBOT[2])
        case "S":
            robotNextPos.append(ROBOT[0])
            robotNextPos.append(ROBOT[1] + ROBOT_STEP)
            robotNextPos.append(ROBOT[2])
        case "W":
            robotNextPos.append(ROBOT[0] + ROBOT_STEP)
            robotNextPos.append(ROBOT[1])
            robotNextPos.append(ROBOT[2])
        case "E":
            robotNextPos.append(ROBOT[0] - ROBOT_STEP)
            robotNextPos.append(ROBOT[1])
            robotNextPos.append(ROBOT[2])
    if checkAccessible(robotNextPos) == True:
        # print("Move Backward: valid move")
        return robotNextPos
    else:
        # print("Move Backward: cannot make the move, out of border")
        return ROBOT
    # End of function moveBackward(ROBOT)


# Function: moveForwardLeft 1 step
# Return: ROBOT = [x coordinate, y coordinate, angle]
def moveForwardLeft(ROBOT):

    robotNextPos = []
    match ROBOT[2]:
        case "N":
            if checkTurnClearance(ROBOT, -1, 2):
                robotNextPos.append(ROBOT[0] - TURN_RADIUS)
                robotNextPos.append(ROBOT[1] + TURN_RADIUS)
                robotNextPos.append("W")
            else:
                # print("Move ForwardLeft: cannot make the move, not enough cleanrance")
                return ROBOT
        case "S":
            if checkTurnClearance(ROBOT, 1, -2):
                robotNextPos.append(ROBOT[0] + TURN_RADIUS)
                robotNextPos.append(ROBOT[1] - TURN_RADIUS)
                robotNextPos.append("E")
            else:
                # print("Move ForwardLeft: cannot make the move, not enough cleanrance")
                return ROBOT
        case "W":
            if checkTurnClearance(ROBOT, -2, -1):
                robotNextPos.append(ROBOT[0] - TURN_RADIUS)
                robotNextPos.append(ROBOT[1] - TURN_RADIUS)
                robotNextPos.append("S")
            else:
                # print("Move ForwardLeft: cannot make the move, not enough cleanrance")
                return ROBOT
        case "E":
            if checkTurnClearance(ROBOT, 2, 1):
                robotNextPos.append(ROBOT[0] + TURN_RADIUS)
                robotNextPos.append(ROBOT[1] + TURN_RADIUS)
                robotNextPos.append("N")
            else:
                # print("Move ForwardLeft: cannot make the move, not enough cleanrance")
                return ROBOT

    if checkAccessible(robotNextPos) == True:
        # print("Move ForwardLeft: valid move")
        return robotNextPos
    else:
        # print("Move ForwardLeft: cannot make the move, out of border")
        return ROBOT
    # End of function moveForwardLeft(ROBOT)


# Function: moveForwardRight 1 step
# Return: ROBOT = [x coordinate, y coordinate, angle]
def moveForwardRight(ROBOT):
    robotNextPos = []
    match ROBOT[2]:
        case "N":
            if checkTurnClearance(ROBOT, 1, 2):
                robotNextPos.append(ROBOT[0] + TURN_RADIUS)
                robotNextPos.append(ROBOT[1] + TURN_RADIUS)
                robotNextPos.append("E")
            else:
                # print("Move ForwardRight: cannot make the move, not enough cleanrance")
                return ROBOT
        case "S":
            if checkTurnClearance(ROBOT, -1, -2):
                robotNextPos.append(ROBOT[0] - TURN_RADIUS)
                robotNextPos.append(ROBOT[1] - TURN_RADIUS)
                robotNextPos.append("W")
            else:
                # print("Move ForwardRight: cannot make the move, not enough cleanrance")
                return ROBOT
        case "W":
            if checkTurnClearance(ROBOT, -2, 1):
                robotNextPos.append(ROBOT[0] - TURN_RADIUS)
                robotNextPos.append(ROBOT[1] + TURN_RADIUS)
                robotNextPos.append("N")
            else:
                # print("Move ForwardRight: cannot make the move, not enough cleanrance")
                return ROBOT
        case "E":
            if checkTurnClearance(ROBOT, 2, -1):
                robotNextPos.append(ROBOT[0] + TURN_RADIUS)
                robotNextPos.append(ROBOT[1] - TURN_RADIUS)
                robotNextPos.append("S")
            else:
                # print("Move ForwardRight: cannot make the move, not enough cleanrance")
                return ROBOT

    if checkAccessible(robotNextPos) == True:
        # print("Move ForwardRight: valid move")
        return robotNextPos
    else:
        # print("Move ForwardRight: cannot make the move, out of border")
        return ROBOT
    # End of function moveForwardRight(ROBOT)


# Function: moveBackwardLeft 1 step
# Return: ROBOT = [x coordinate, y coordinate, angle]
def moveBackwardLeft(ROBOT):
    robotNextPos = []
    match ROBOT[2]:
        case "N":
            if checkTurnClearance(ROBOT, -1, -2):
                robotNextPos.append(ROBOT[0] - TURN_RADIUS)
                robotNextPos.append(ROBOT[1] - TURN_RADIUS)
                robotNextPos.append("E")
            else:
                # print("Move BackwardLeft: cannot make the move, not enough cleanrance")
                return ROBOT
        case "S":
            if checkTurnClearance(ROBOT, 1, 2):
                robotNextPos.append(ROBOT[0] + TURN_RADIUS)
                robotNextPos.append(ROBOT[1] + TURN_RADIUS)
                robotNextPos.append("W")
            else:
                # print("Move BackwardLeft: cannot make the move, not enough cleanrance")
                return ROBOT
        case "W":
            if checkTurnClearance(ROBOT, 2, -1):
                robotNextPos.append(ROBOT[0] + TURN_RADIUS)
                robotNextPos.append(ROBOT[1] - TURN_RADIUS)
                robotNextPos.append("N")
            else:
                # print("Move BackwardLeft: cannot make the move, not enough cleanrance")
                return ROBOT
        case "E":
            if checkTurnClearance(ROBOT, -2, 1):
                robotNextPos.append(ROBOT[0] - TURN_RADIUS)
                robotNextPos.append(ROBOT[1] + TURN_RADIUS)
                robotNextPos.append("S")
            else:
                # print("Move BackwardLeft: cannot make the move, not enough cleanrance")
                return ROBOT

    if checkAccessible(robotNextPos) == True:
        # print("Move BackwardLeft: valid move")
        return robotNextPos
    else:
        # print("Move BackwardLeft: cannot make the move, out of border")
        return ROBOT
    # End of function moveBackwardLeft(ROBOT)


# Function: moveBackwardRight 1 step
# Return: ROBOT = [x coordinate, y coordinate, angle]
def moveBackwardRight(node):
    robotNextPos = []
    match node[2]:
        case "N":
            if checkTurnClearance(node, 1, -2) :
                robotNextPos.append(node[0] + TURN_RADIUS)
                robotNextPos.append(node[1] - TURN_RADIUS)
                robotNextPos.append("W")
            else:
                # print("Move BackwardRight: cannot make the move, not enough cleanrance")
                return node
        case "S":
            if checkTurnClearance(node, -1, 2) :
                robotNextPos.append(node[0] - TURN_RADIUS)
                robotNextPos.append(node[1] + TURN_RADIUS)
                robotNextPos.append("E")
            else:
                # print("Move BackwardRight: cannot make the move, not enough cleanrance")
                return node
        case "W":
            if checkTurnClearance(node, 2, 1) :
                robotNextPos.append(node[0] + TURN_RADIUS)
                robotNextPos.append(node[1] + TURN_RADIUS)
                robotNextPos.append("S")
            else:
                # print("Move BackwardRight: cannot make the move, not enough cleanrance")
                return node
        case "E":
            if checkTurnClearance(node, -2, -1) :
                robotNextPos.append(node[0] - TURN_RADIUS)
                robotNextPos.append(node[1] - TURN_RADIUS)
                robotNextPos.append("N")
            else:
                # print("Move BackwardRight: cannot make the move, not enough cleanrance")
                return node
    if checkAccessible(robotNextPos) == True:
        print("turn backward right, robotNextPos: ", robotNextPos, " accessible: ", checkAccessible(robotNextPos))

        # print("Move BackwardRight: valid move")
        return robotNextPos
    else:
        # print("Move BackwardRight: cannot make the move, out of border")
        return node
    # End of function moveBackwardRight(ROBOT)


# Helper Function: test ROBOT movement
def testMovement():
    global ROBOT
    ROBOT = [5,5,"N"]
    print("ROBOT is ", ROBOT)
    robotPos = moveForward(ROBOT)
    print (robotPos)
    robotPos = moveBackward(ROBOT)
    print (robotPos)
    robotPos = moveForwardLeft(ROBOT)
    print (robotPos)
    robotPos = moveForwardRight(ROBOT)
    print (robotPos)
    robotPos = moveBackwardLeft(ROBOT)
    print (robotPos)
    robotPos = moveBackwardRight(ROBOT)
    print (robotPos)

    # End of function testMovement()


# Function: g cost from current node to next node
def greedy(currentNode, nextNode):
    turnCost = 0
    cost = ROBOT_STEP

    # check to see if turning is required to get to that direction (not in the same angle dimension)
    if currentNode[2] != nextNode[2]:
        turnCost = TURN_RADIUS

    #return the sum of the cost to move 1 node + the cost to turn (if turning is done)
    return cost + turnCost;
    # End of function gCost(currentNode, nextNode)


# Function: Heuristic algorithm using manhattan distance from current node to end node.
def heuristic(currentNode, targetNode):
    manhattanDis = (abs(currentNode[0] - targetNode[0]) + abs(currentNode[1] - targetNode[1]))*ROBOT_STEP
    print("manhattanDis is", manhattanDis)

    turnCost = 0

    # if in front has the obstacle, but not the desired one, add to turn cost
    if checkFrontGotObstacle(currentNode) == True and currentNode != targetNode:
        print("checkFrontGotObstacle(currentNode) is ", checkFrontGotObstacle(currentNode))
        print("There is obstacle in front of it.")
        turnCost += TURN_RADIUS

    # if direction different, has turn cost
    if currentNode[2] != targetNode[2]:
        turnCost += TURN_RADIUS
        # if opposite direction, turn cost = turn radius * 2
        match targetNode[2]:
            case "N":
                if currentNode[2] == "S":
                    turnCost += TURN_RADIUS
            case "S":
                if currentNode[2] == "N":
                    turnCost += TURN_RADIUS
            case "W":
                if currentNode[2] == "E":
                    turnCost += TURN_RADIUS
            case "E":
                if currentNode[2] == "W":
                    turnCost += TURN_RADIUS
    else: #if direction is the same
        # if 2 coordinates all different, at least 2 turns
        if currentNode[0] != targetNode[0] and currentNode[1] != targetNode[1]:
            turnCost += TURN_RADIUS * 2
        # if 1 coordinate is same
        else:
            match targetNode[2]:
                case "N":
                    if currentNode[0] == targetNode[0] and currentNode[1] < targetNode[1]:
                        turnCost += 0
                    else:
                        turnCost += TURN_RADIUS * 2
                case "S":
                    if currentNode[0] == targetNode[0] and currentNode[1] > targetNode[1]:
                        turnCost += 0
                    else:
                        turnCost += TURN_RADIUS * 2
                case "W":
                    if currentNode[0] > targetNode[0] and currentNode[1] == targetNode[1]:
                        turnCost += 0
                    else:
                        turnCost += TURN_RADIUS * 2
                case "E":
                    if currentNode[0] < targetNode[0] and currentNode[1] == targetNode[1]:
                        turnCost += 0
                    else:
                        turnCost += TURN_RADIUS * 2

    print("heuristic turnCost is", turnCost)
    # prefer nodes in the same direction as the end direction
    return manhattanDis + turnCost
    # End of function hCost(currentNode, targetNode)


# Function: plan the trip between 2 vertex
def planTrip(ROBOT, target):
    # to record preview move, avoid to mvoe back and forth and stuck
    previousMove = 0
    gCost = 0
    hCost = 0

    trip = []
    totalCost = [100 for i in range(6)]
    validMove = [1 for i in range(6)]

    print("target = ", target)

    while ROBOT[0] != target[0] or ROBOT[1] != target[1] or ROBOT[2] != target[2]:
        print("")
        print("while loop: ")

        print("ROBOT = ", ROBOT)

        # get possible next Node in all 6 directions, put in a list
        # print("get possible nodes")
        possibleNode = [moveForward(ROBOT), moveBackward(ROBOT), moveForwardLeft(ROBOT), moveBackwardLeft(ROBOT), moveForwardRight(ROBOT), moveBackwardRight(ROBOT)]

        print("F, B, F-L, B-L, F-R, B-RQ")
        print("possible next node = ", possibleNode)

        # calculate the cost of each direction
        # f=g+h
        for i in range(len(possibleNode)):

            # if invalid move, mark in the validMove list

            if possibleNode[i] == ROBOT:
                validMove[i] = 0

            # if it's valid move, calculate cost
            else:
                validMove[i] = 1

                print("Robot is ", ROBOT)
                print("possible Node is ", possibleNode[i])

                gCost = greedy(ROBOT, possibleNode[i])
                hCost = hCost(ROBOT, target)

                # Old version

                # g: movement cost from current position to the node
                # if i == 0 or i == 1:
                    #g = ROBOT_STEP
                #else:
                    #g = TURN_RADIUS


                # h: heuristic cost from next node to the target

                # Mahattan Distance
                #h = abs(target[0]-possibleNode[i][0]) + abs(target[1]-possibleNode[i][1])

                # If direction not the same as target, add turn radius
                #if possibleNode[i][2] != target[2]:
                    #h += TURN_RADIUS

                    #match target[2]:
                        #case "N":
                            #if possibleNode[i][2] == "S":
                                #h += TURN_RADIUS
                        #case "S":
                            #if possibleNode[i][2] == "N":
                                #h += TURN_RADIUS
                        #case "W":
                            #if possibleNode[i][2] == "E":
                                #h += TURN_RADIUS
                        #case "E":
                            #if possibleNode[i][2] == "W":
                                #h += TURN_RADIUS

                #else: # if direction is the same
                    # but the 2 coordinate is different, will have at least 2 turns
                    #if possibleNode[i][0] != target[0] and possibleNode[i][1] != target[1] :
                        #h += TURN_RADIUS * 2

                    #else: # if at least 1 coordinate is the same
                        #match target[2]:
                            #case "N":
                                #if possibleNode[i][2] == "N" and possibleNode[i][0] == target[0]:
                                    #h = h
                                #else:
                                    #h += TURN_RADIUS * 3
                            #case "S":
                                #if possibleNode[i][2] == "S" and possibleNode[i][0] == target[0]:
                                    #h = h
                                #else:
                                    #h += TURN_RADIUS * 3
                            #case "W":
                                #if possibleNode[i][2] == "W" and possibleNode[i][1] == target[1]:
                                    #h = h
                                #else:
                                    #h += TURN_RADIUS * 3
                            #case "E":
                                #if possibleNode[i][2] == "E" and possibleNode[i][1] == target[1]:
                                    #h = h
                                #else:
                                    #h += TURN_RADIUS * 3


                # total cost = movement cost + heuristic cost
                f = gCost + hCost

                # record in totalCost list
                totalCost[i] = f

        # do not go back and forth repeat
        if previousMove%2 == 0:
            validMove[previousMove+1] = 0
        else:
            validMove[previousMove-1] = 0

        # set invalid move cost to super big
        for i in range(len(totalCost)):
            if validMove[i] == 0:
                totalCost[i] = 100

        print("valid move = ", validMove)
        print("minimum cost = ", min(totalCost))
        print("total cost = ", totalCost)

        minCost = min(totalCost)
        # find the node with minimum cost, and choose it as the next node
        for i in range(6):
            if totalCost[i] == minCost:
                previousMove = i
                ROBOT = possibleNode[i]
                print("ROBOT move to :", ROBOT)
                trip.append(ROBOT)
                print("trip = ", trip)
                break

    return trip, ROBOT
    # End of function planTrip()


# Function: plan the trip using A star algorithm
def planTripAlgo(target):
    # A* (star) Pathfinding
    global ROBOT

    # Initialize both open and closed list
    # let the openList equal empty list of nodes
    openList = []
    # let the closedList equal empty list of nodes
    closedList = []

    # initialize gCostArray and hCostArray
    gCostArray = [[0 for i in range(20)] for i in range(20)]
    print("gCostArray: ", gCostArray)
    hCostArray = [[0 for i in range(20)] for i in range(20)]
    print("hCostArray: ", hCostArray)
    fCostArray = [[0 for i in range(20)] for i in range(20)]
    print("fCostArray: ", fCostArray)

    parentNodeArray = [[None for i in range(20)] for i in range(20)]
    print("parentNodeArray: ", parentNodeArray)
    #VisitedArray = [[0 for i in range(20)] for i in range(20)]
    #print("VisitedArray: ", VisitedArray)

    previousNode = ROBOT


    # Add the start node
    # put the startNode on the openList (leave it's f at zero)
    openList.append(ROBOT)
    print("openList: ", openList)

    #  Loop until you find the end
    # while the openList is not empty
    while len(openList) > 0:
        print(" ")
        print("Enter while loop, openList: ", openList)


        #smallestFCost = 100
        #for i in range(len(openList)):
            #FCost = fCostArray[openList[i][0]][openList[i][1]]
            #if FCost < smallestFCost:
                #smallestFCost = FCost
                #smallestFNode = i
        #currentNode = openList[i]

        # Get the current node
        # let the currentNode equal the node with the least f value
        currentNode = openList[0]
        currentIndex = 0
        for index, item in enumerate(openList):
            if fCostArray[item[0]][item[1]] < fCostArray[currentNode[0]][currentNode[1]]:
                currentNode = item
                currentIndex = index

        # remove the currentNode from the openList
        # openList.pop(currentIndex)
        # remove all items from openList
        openList.clear()

        # add the currentNode to the closedList
        previousNode = currentNode
        closedList.append(currentNode)
        print("closedList: ", closedList)

        #  Found the goal
        # if currentNode is the goal
        if currentNode == target:
            ROBOT = target
            return closedList
            # Found the goal! Backtrack to get path
            # path = []
            # current = currentNode
            # while current is not None:
                # path.append(current)
                # current = parentNodeArray[current[0]][current[1]]
            # return path[::-1]# Return reversed path


        #  Generate children
        # let the children of the currentNode equal the adjacent nodes
        childNode = [moveForward(currentNode), moveBackward(currentNode), moveForwardLeft(currentNode), moveBackwardLeft(currentNode), moveForwardRight(currentNode), moveBackwardRight(currentNode)]
        for item in reversed(childNode):
            if item == currentNode or item == previousNode: #or VisitedArray[item[0]][item[1]] == 1:
                childNode.remove(item)

        #childNode.reverse()
        # print("childNode after reverse: ", childNode )
        print("childNode after remove: ", childNode )

        # Loop through children
        for child in childNode:

            parentNodeArray[child[0]][child[1]] = currentNode

            # Child is on the closed list
            if child in closedList:
                continue

            # Create the f, g, and h values
            # child.g = currentNode.g + distance between child and current
            gCostArray[child[0]][child[1]] = gCostArray[currentNode[0]][currentNode[1]] + greedy(currentNode, child)
            print("gCostArray is", gCostArray[currentNode[0]][currentNode[1]])
            print("greedy(currentNode, child) is", greedy(currentNode, child))
            print("gCost of (", child[0], ",", child[1],"): ", gCostArray[child[0]][child[1]])
            # child.h = distance from child to end
            hCostArray[child[0]][child[1]] = heuristic(child, target)
            print("hCost of (", child[0], ",", child[1],"): ", hCostArray[child[0]][child[1]])
            # child.f = child.g + child.h
            fCostArray[child[0]][child[1]] = gCostArray[child[0]][child[1]] + hCostArray[child[0]][child[1]]
            print("fCost of (", child[0], ",", child[1],"): ", fCostArray[child[0]][child[1]])
            print(" ")
            # Child is already in the open list
            if child in openList:
                childIndex = openList.index(child)
                openChild = openList[childIndex]
                if gCostArray[child[0]][child[1]] > gCostArray[openChild[0]][openChild[1]]:
                    continue

            # Add the child to the open list
            openList.append(child)




# Main Function: compute the shortest Hamiltonian path to traverse all vertex
def mainAlgo():

    # 1.   Get info from other modules

    # 1.1  The bottom left coordinates and image facing direction of 5 obstacles
    # Format = [x coordinate, y coordinate, direction-North West South East]

    ############ TODO: Need to figure out how to get info from Android #############
    obstacles = [[5, 7, "S"], [15, 4, "N"], [12, 9, "E"], [5, 13, "W"], [15, 15, "S"]]





    # 2.   Pre-processing data
    global ROBOT
    # 2.1  Calculate ROBOT's desired location and angle in order to scan 5 images
    vertex = findVertex(obstacles)
    # Print
    print ("ROBOT should go to ", vertex)


    # 3.2  Mark obstacles coordinate on the console MAP as 0
    # MAP representation:
    # 1 = accessible, 0 = non-accesible
    markObstaclesOnMAP(obstacles)
    # Print the MAP
    printMAP()


    #ROBOT = [4, 3, "N"]
    #testMovement()
    #print("ROBOT is ", ROBOT)





    # 3.3 Check if all the desired vertex are accessible,
    # if yes, then proceed with the algorithm
    # if no, request to change obstacle location
    for i in range(len(vertex)):
        if checkAccessible(vertex[i]) == True:
            # print("Vertex is accessble")
            continue
        else:
            print("The obstacle at coordinate (", obstacles[i][0], ", ", obstacles[i][1], ") is not accessible.")
            print("Please change the position of obstacle")
    ############### TODO: return error, request to change location of obstacles ###############


    # 4.  Path finding, plan a Hamiltonian Path to access all vertex
    # 4.1 greedy heuristic algorithm to plan the path
    plannedPath = findGreedyPath(ROBOT, vertex)
    # print
    print("Found a greedy path: ", plannedPath)


    # 4.2 for each section of the path, find the shortest trip between 2 Vertex

    # Initialize a empty list
    plannedTrip = []

    for vertex in plannedPath:
        print("vertex is ", vertex)
        trip = planTripAlgo(vertex)
        print("Planned Trip to reach vertex ", vertex, " is ", plannedTrip)


    # try plan trip for the first section

    #trip = planTripAlgo(plannedPath[0])
    #plannedTrip.append(trip)
    #print("Planned Trip is ", plannedTrip)
    #print("ROBOT is ", ROBOT)

    # try plan trip for the second section
    #ROBOT = [4, 2, "N"]
    #trip = planTripAlgo(plannedPath[1])
    #plannedTrip.append(trip)
    #print("Planned Trip is ", plannedTrip)
    #print("ROBOT is ", ROBOT)

    # testing checkAccessible function
    #ROBOT = [4, 6, "S"]
    #check = checkAccessible(ROBOT)
    #print("check is ", check)






# Main Function
if __name__ == "__main__":
    mainAlgo()
