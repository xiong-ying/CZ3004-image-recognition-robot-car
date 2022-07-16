# title: pathFinding.py
# author: Xiong Ying
# date: 09/07/2022 Saturday
# purpose: To find the Hamiltonian Path for the ROBOT to traverse through 5 obstacles


# size of map
MAP_SIZE = 20

# Initialize MAP 20 by 20, filled with 1
# MAP representation:
# 1 = accessible, 0 = non-accesible
MAP = [[1 for i in range(MAP_SIZE)] for i in range(MAP_SIZE)]

# ROBOT Initialize position and angle
ROBOT = [0, 0, "N"]

# ROBOT boundary area
ROBOT_BORDER = 3

# ROBOT actual width
ROBOT_WIDTH = 2

# Obstacle size
OBSTACLE_WIDTH = 1

# distance from robot to obstacle
CLEARANCE = 2

# size of 1 move
ROBOT_STEP = 1

# turn is within 40 by 40 cm area towards the turn direction
TURN_RADIUS = 4


# Function: calculate the ROBOT's desired location and facing angle, based on the obstacles' location and direction
# return: list[[]]
def findVertex(obstacles):
    # Initialize empty list
    vertex = []
    # For each obstacles, calculate the desired vertex position and angle
    for obstacle in obstacles:
        match obstacle[2]:
            case "N":
                vertex.append([obstacle[0] - 1, obstacle[1] + OBSTACLE_WIDTH + CLEARANCE, "S"])
            case "S":
                vertex.append([obstacle[0] - 1, obstacle[1] - CLEARANCE - ROBOT_BORDER, "N"])
            case "W":
                vertex.append([obstacle[0] - CLEARANCE - ROBOT_BORDER, obstacle[1] - 1, "E"])
            case "E":
                vertex.append([obstacle[0] + OBSTACLE_WIDTH + CLEARANCE, obstacle[1] - 1, "W"])
            case default:
                return "Error"

    return vertex
    # print (vertex)
    # End of function findVertex(obstacles)


# Function: mark the obstacle on the console MAP
def markObstaclesOnMAP(obstacles):

    # Mark the obstacles position on the MAP as 0
    for obstacle in obstacles:
        MAP[obstacle[0]][obstacle[1]] = 0
    # End of function markObstaclesOnMAP(MAP, obstacles)


# Function: print out the current MAP
def printMAP():

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
    # End of Function printMAP(MAP)


# Function: Check if the node is accessible
# return: boolean
def checkAccessible(node):
    #if node[0] < 0 or node[0] > (19-ROBOT_BORDER+1) or node[1] < 0 or node[1] > (19-ROBOT_BORDER+1):
        #print("not accessible because out of border")
        #return False
    #else:
    try:
        for i in range(ROBOT_BORDER):
            for j in range(ROBOT_BORDER):
                if MAP[node[0]+i][node[1]+j] == 0:
                    return False
        return True
    except:
        return True
    # End of function checkAccessible(node)


# Function: Check if the node is an obstacle
# return: boolean
def isObstacle(node):
    if 0 <= node[0] <= 19 and 0 <= node[1] <= 19:
        # 0 : obstacle
        if MAP[node[0]][node[1]] == 0:
            return True
        else:
            return False
    else:
        return False
    # End of function isObstacle(node)


# Function: check if can make a turns, with enough clearance
# return: boolean
def checkTurnClearance(node, xOffset, yOffset):
    for i in range(TURN_RADIUS):
        for j in range(TURN_RADIUS):
            #print("i th loop: ", i, " ; j th loop: ", j)
            #print("is it accessible? ", isObstacle([node[0]+xOffset+i, node[1]+yOffset+j]))
            if isObstacle([node[0]+xOffset+i, node[1]+yOffset+j]) == True:
                return False
    return True
    # End of function checkTurnClearance(xOffset, yOffset)


# Function: check if there is obstacle in front
# return: boolean
def checkFrontGotObstacle(node):
    print("Start of checkFrontGotObstacle(node)")
    match node[2]:
        case "N":
            if checkTurnClearance(node, 0, TURN_RADIUS) == False:
                # print("checkTurnClearance(node, 0, 3): ", checkTurnClearance(node, 0, 3))
                return True
            else:
                return False
        case "S":
            if checkTurnClearance(node, 0, -1 * TURN_RADIUS) == False:
                # print("checkTurnClearance(node, 0, -3): ", checkTurnClearance(node, 0, -3))
                return True
            else:
                return False
        case "W":
            if checkTurnClearance(node, -1 * TURN_RADIUS, 0) == False:
                # print("checkTurnClearance(node, -3, 0): ", checkTurnClearance(node, -3, 0))
                return True
            else:
                return False
        case "E":
            if checkTurnClearance(node, TURN_RADIUS, 0) == False:
                # print("checkTurnClearance(node, 3, 0): ", checkTurnClearance(node, 3, 0))
                return True
            else:
                return False
    # End of function checkFrontGotObstacle(node)


# Function: find a Hamiltonian path of all Vertex using nearest neighbour greedy heuristic algorithm
def findGreedyPath(node, vertex):
    # Initialize a local list to store the shortest distance from current node to each vertex
    distance = [100 for i in range (len(vertex))] # 100 is just a random super big number
    # a local variable foor distance comparison
    nearest = 100 # 100 is just a random super big number
    # a local list to store the visited flag for 5 vertex
    visited = [0 for i in range(len(vertex))]
    # record the current node position
    currentNode = node
    # empty list
    plannedPath = []

    # loop 5 times to find the sequence of 5 vertex in planned path
    for i in range(len(vertex)):
        # for each node in the planned path, compute the distance from current node
        for j in range(len(vertex)):

            if visited[j] == 0:
                # compute the distance from current node to jth node
                distance[j] = (pow(abs(currentNode[0] - vertex[j][0]), 2) + pow(abs(currentNode[1] - vertex[j][1]), 2)) ** 0.5
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
def moveForward(node):

    nextNode = node.copy()
    #print("Moveforward: nextnode is ", nextNode)
    match node[2]:
        case "N":
            nextNode[1] = node[1] + ROBOT_STEP
        case "S":
            nextNode[1] = node[1] - ROBOT_STEP
        case "W":
            nextNode[0] = node[0] - ROBOT_STEP
        case "E":
            nextNode[0] = node[0] + ROBOT_STEP
    #print("Moveforward: nextnode is ", nextNode)
    if checkAccessible(nextNode) == True:
        # print("Move Forward: valid move")
        return nextNode
    else:
        # print("Move Forward: cannot make the move, out of border")
        return node
    # End of function moveForward(node)


# Function: moveForward 1 step
# Return: ROBOT = [x coordinate, y coordinate, angle]
def moveBackward(node):

    nextNode = node.copy()
    match node[2]:
        case "N":
            nextNode[1] = node[1] - ROBOT_STEP
        case "S":
            nextNode[1] = node[1] + ROBOT_STEP
        case "W":
            nextNode[0] = node[0] + ROBOT_STEP
        case "E":
            nextNode[0] = node[0] - ROBOT_STEP
    if checkAccessible(nextNode) == True:
        # print("Move Backward: valid move")
        return nextNode
    else:
        # print("Move Backward: cannot make the move, out of border")
        return node
    # End of function moveBackward(ROBOT)


# Function: turnLeft
# Return: node = [x coordinate, y coordinate, angle]
def turnLeft(node):
    nextNode = node.copy()
    if checkAccessible(nextNode) == True:
        match nextNode[2]:
            case "N":
                if checkTurnClearance(node, ROBOT_WIDTH - TURN_RADIUS, 0):
                    nextNode[2] = "W"
                    nextNode[0] -= 1
            case "S":
                if checkTurnClearance(node, 0, ROBOT_WIDTH - TURN_RADIUS):
                    nextNode[2] = "E"
                    nextNode[0] += 1
            case "W":
                if checkTurnClearance(node, ROBOT_WIDTH - TURN_RADIUS, ROBOT_WIDTH - TURN_RADIUS):
                    nextNode[1] -= 1
                    nextNode[2] = "S"
            case "E":
                if checkTurnClearance(node, 0, 0):
                    nextNode[1] += 1
                    nextNode[2] = "N"
            # print("Move ForwardRight: valid move")
        return nextNode
    else:
        return node
    # End of function turnLeft(node)


# Function: turnRight
# Return: node = [x coordinate, y coordinate, angle]
def turnRight(node):
    nextNode = node.copy()
    if checkAccessible(nextNode) == True:
        match nextNode[2]:
            case "N":
                if checkTurnClearance(node, 0, 0):
                    nextNode[0] += 1
                    nextNode[2] = "E"
            case "S":
                if checkTurnClearance(node, ROBOT_WIDTH - TURN_RADIUS, ROBOT_WIDTH - TURN_RADIUS):
                    nextNode[0] -= 1
                    nextNode[2] = "W"
            case "W":
                if checkTurnClearance(node, ROBOT_WIDTH - TURN_RADIUS, 0):
                    nextNode[1] += 1
                    nextNode[2] = "N"
            case "E":
                if checkTurnClearance(node, 0, ROBOT_WIDTH - TURN_RADIUS):
                    nextNode[1] -= 1
                    nextNode[2] = "S"
            # print("Move ForwardRight: valid move")
        return nextNode
    else:
        return node
    # End of function turnRight(ROBOT)



# Helper Function: test ROBOT movement
def testMovement():
    global ROBOT
    ROBOT = [1,5,"N"]
    print("ROBOT is ", ROBOT)
    robotPos = moveForward(ROBOT)
    print (robotPos)
    robotPos = moveBackward(ROBOT)
    print (robotPos)
    robotPos = turnLeft(ROBOT)
    print (robotPos)
    robotPos = turnRight(ROBOT)
    print (robotPos)
    # End of function testMovement()



# Function: g cost simple, from current node to next node
def gCost(currentNode, nextNode):

    cost = (abs(currentNode[0] - nextNode[0]) + abs(currentNode[1] - nextNode[1]))*ROBOT_STEP

    return cost
    # End of function gCost(currentNode, nextNode)

# Function: h cost simple
def hCost(currentNode, targetNode):

    manhattanDis = (abs(currentNode[0] - targetNode[0]) + abs(currentNode[1] - targetNode[1]))*ROBOT_STEP

    print("heuristic cost is", manhattanDis)
    return manhattanDis
    # End of function hCost(currentNode, targetNode)



# Function: g cost from current node to next node
def greedy(currentNode, nextNode):

    #cost = (abs(currentNode[0] - nextNode[0]) + abs(currentNode[1] - nextNode[1]))*ROBOT_STEP

    turnCost = 0
    cost = ROBOT_STEP

    # check to see if turning is required to get to that direction (not in the same angle dimension)
    if currentNode[2] != nextNode[2]:
        turnCost = TURN_RADIUS

    # return the sum of the cost to move 1 node + the cost to turn (if turning is done)
    return cost + turnCost
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

    validMove = [1 for i in range(4)]


    # Add the start node
    # put the startNode on the openList (leave it's f at zero)
    openList.append(ROBOT)
    print("openList: ", openList)

    #  Loop until you find the end
    # while the openList is not empty
    while len(openList) > 0:
        print(" ")
        print("Enter while loop, openList: ", openList)

        # Get the current node
        # let the currentNode equal the node with the least f value
        currentNode = openList[0]
        currentIndex = 0
        for index, item in enumerate(openList):
            if fCostArray[item[0]][item[1]] < fCostArray[currentNode[0]][currentNode[1]]:
                currentNode = item
                currentIndex = index

        # remove the currentNode from the openList
        openList.pop(currentIndex)

        # remove all items from openList
        # openList.clear()

        # add the currentNode to the closedList
        closedList.append(currentNode)
        print("closedList: ", closedList)

        #  Found the goal
        # if currentNode is the goal
        if currentNode == target:

            # Found the goal! Backtrack to get path
            path = []
            current = currentNode
            while current is not None:
                path.append(current)
                current = parentNodeArray[current[0]][current[1]]
                return path[::-1]# Return reversed path


        #  Generate children
        # let the children of the currentNode equal the adjacent nodes
        # check if it's a valid move
        childNode = [moveForward(currentNode), moveBackward(currentNode), turnLeft(currentNode), turnRight(currentNode)]
        for index, item in enumerate(childNode):
            if item == currentNode: #or VisitedArray[item[0]][item[1]] == 1:
                validMove[index] = 0
            else:
                validMove[index] = 1

        # print("childNode after reverse: ", childNode )
        print("childNode after remove: ", childNode )

        # Loop through children
        for child in childNode:

            if validMove[childNode.index(child)] == 1:

                parentNodeArray[child[0]][child[1]] = currentNode

                # Child is on the closed list
                if child in closedList:
                    continue

                # Create the f, g, and h values
                # child.g = currentNode.g + distance between child and current
                gCostArray[child[0]][child[1]] = gCostArray[currentNode[0]][currentNode[1]] + gCost(currentNode, child)
                #print("gCostArray is", gCostArray[currentNode[0]][currentNode[1]])
                #print("greedy(currentNode, child) is", greedy(currentNode, child))
                print("gCost of (", child[0], ",", child[1],"): ", gCostArray[child[0]][child[1]])

                # child.h = distance from child to end
                hCostArray[child[0]][child[1]] = hCost(child, target)
                print("hCost of (", child[0], ",", child[1],"): ", hCostArray[child[0]][child[1]])

                # child.f = child.g + child.h
                fCostArray[child[0]][child[1]] = gCostArray[child[0]][child[1]] + hCostArray[child[0]][child[1]]
                print("fCost of (", child[0], ",", child[1],"): ", fCostArray[child[0]][child[1]])

                print(" ")
                # Child is already in the open list
                if child in openList:
                    openChildIndex = openList.index(child)
                    openChild = openList[openChildIndex]
                    if gCostArray[child[0]][child[1]] > gCostArray[openChild[0]][openChild[1]]:
                        continue

                # Add the child to the open list
                openList.append(child)


def astar(target):

    global ROBOT

    # Arrays
    # initialize gCostArray and hCostArray
    gCostArray = [[0 for i in range(20)] for i in range(20)]
    hCostArray = [[0 for i in range(20)] for i in range(20)]
    fCostArray = [[0 for i in range(20)] for i in range(20)]
    parentNodeArray = [[None for i in range(20)] for i in range(20)]

    validMove = [0 for i in range(4)]

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(ROBOT)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if fCostArray[item[0]][item[1]] < fCostArray[current_node[0]][current_node[1]]:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == target:
            path = []
            current = current_node
            while current is not None:
                path.append(current)
                current = parentNodeArray[current[0]][current[1]]
            return path[::-1] # Return reversed path

        # Generate children
        childNode = [moveForward(currentNode), moveBackward(currentNode), turnLeft(currentNode), turnRight(currentNode)]
        for child in childNode:
                # Make sure within range
            if child[0] > (len(MAP_SIZE) - 1) or child[0] < 0 or child[1] > (len(MAP_SIZE) -1) or child[1] < 0:
                validMove[childNode.index(child)] = 0
                continue

                # Make sure walkable terrain
            if MAP[child[0]][child[1]] == 0:
                validMove[childNode.index(child)] = 0
                continue

            validMove[childNode.index(child)] = 1

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            gCostArray[child[0]][child[1]] = gCostArray[current_node[0]][current_node[1]] + 1
            hCostArray[child[0]][child[1]] = ((child[0] - target[0]) ** 2) + ((child[1] - target[1]) ** 2)
            fCostArray[child[0]][child[1]]= hCostArray[child[0]][child[1]] + gCostArray[child[0]][child[1]]

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and gCostArray[child[0]][child[1]] > gCostArray[open_node[0]][open_node[1]]:
                    continue

            # Add the child to the open list
            open_list.append(child)



# Main Function: compute the shortest Hamiltonian path to traverse all vertex
def mainAlgo():

    # 1.   Get info from other modules

    # The bottom left coordinates and image facing direction of 5 obstacles
    # Format = [x coordinate, y coordinate, direction-North West South East]

    ############ TODO: Need to figure out how to get info from Android #############
    obstacles = [[5, 7, "S"], [15, 4, "N"], [12, 9, "E"], [5, 13, "W"], [15, 15, "S"]]


    # 2.   Pre-processing data
    global ROBOT


    # 3.  Mark obstacles coordinate on the console MAP as 0
    # MAP representation:
    # 1 = accessible, 0 = non-accesible
    markObstaclesOnMAP(obstacles)
    # Print the MAP
    printMAP()



    # 4.  Calculate ROBOT's desired location and angle in order to scan 5 images
    vertex = findVertex(obstacles)
    # Print
    print ("ROBOT should go to ", vertex)





    #ROBOT = [4, 3, "N"]
    #testMovement()
    #print("ROBOT is ", ROBOT)





    # 5. Check if all the desired vertex are accessible,
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
    #plannedTrip = []

    #for vertex in plannedPath:
        #print("vertex is ", vertex)
        #trip = planTripAlgo(vertex)
        #print("Planned Trip to reach vertex ", vertex, " is ", plannedTrip)


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
