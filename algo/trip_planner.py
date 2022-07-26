# TITLE: trip_planner.py
# AUTHOR: Xiong Ying
# CREATE DATE: 13/07/2022
# UPDATE DATE: 17/7/2022
# PURPOSE: To compute a trip between 2 vertex with obstacle avoidance and turn area clearance checking


from map import *



# CLASS: A node class for A* Pathfinding
class Node():

    def __init__(self, parent=None, position=None, direction=None):

        self.parent = parent
        self.position = position

        # 0 = East, 1 = North, 2 = West, 3 = South
        self.direction = direction

        self.g = 0
        self.h = 0
        self.f = 0
        #self.visited = 0

        
    def __eq__(self, other):
        return self.position == other.position, self.direction == other.direction

    # End of CLASS Node()



# FUNCTION: from the current node.position and the new posititon offset, generate the new node.direction
# RETURN: new node's direction: int
def getNextNodeDirection(current_node, new_position):


    # check current node.direction 
    if current_node.direction == 0 : # current node facing east

        # check new node's position
        
        if new_position[1] == 0: # new node y axis didn't change 
            turn_dir = 0 #stay same direction

        elif new_position[1] > 0 and checkTurnClearance(current_node.position, 0, 0) == True: # if y axis +, if have turn clearance
            turn_dir = 1 # turn left

        elif new_position[1] < 0 and checkTurnClearance(current_node.position, 0, ROBOT_WIDTH-TURN_RADIUS) == True: # if y axis -, if have turn clearance
            turn_dir = -1 # turn right

        else: # all other situation is invalid, cannot make a move
            turn_dir = None

            #__print for debug
            #print("turn_dir = None: not enough clearance to turn")
            #print(" ")
                    

    # check current node.direction 
    elif current_node.direction == 2: # current node facing west

        # check new node's position

        if new_position[1] == 0: # y didn't change 
            turn_dir = 0 # stay same direction

        elif new_position[1] < 0 and checkTurnClearance(current_node.position, ROBOT_WIDTH-TURN_RADIUS, ROBOT_WIDTH-TURN_RADIUS) == True: # if y axis -, if have turn clearance
            turn_dir = 1 # turn left

        elif new_position[1] > 0 and checkTurnClearance(current_node.position, ROBOT_WIDTH-TURN_RADIUS, 0) == True: # if y axis +, if have turn clearance
            turn_dir = -1  # turn right

        else: # all other situation is invalid, cannot make a move
            turn_dir = None

            #__print for debug
            #print("turn_dir = None: not enough clearance to turn")
            #print(" ")


    # check current node.direction 
    elif current_node.direction ==  1: # current node facing north

        # check new node's position

        if new_position[0] == 0: # x didn't change 
            turn_dir = 0 # stay same direction

        elif new_position[0] < 0 and checkTurnClearance(current_node.position, ROBOT_WIDTH-TURN_RADIUS, 0) == True: # if x axis -, if have turn clearance
            turn_dir = 1 # turn left

        elif new_position[0] > 0 and checkTurnClearance(current_node.position, 0, 0) == True: # if x axis +, if have turn clearance
            turn_dir = -1  # turn right

        else: # all other situation is invalid, cannot make a move
            turn_dir = None

            #__print for debug
            #print("turn_dir = None: not enough clearance to turn")
            #print(" ")


    # check current node.direction 
    elif current_node.direction ==  3: # current node facing south

        # check new node's position

        if new_position[0] == 0: # x didn't change 
            turn_dir = 0 # stay same direction

        elif new_position[0] > 0 and checkTurnClearance(current_node.position, 0, ROBOT_WIDTH-TURN_RADIUS) == True: # if x axis +, if have turn clearance
            turn_dir = 1 # turn left

        elif new_position[0] < 0 and checkTurnClearance(current_node.position, ROBOT_WIDTH-TURN_RADIUS, ROBOT_WIDTH-TURN_RADIUS) == True: # if x axis -, if have turn clearance
            turn_dir = -1 # turn right

        else: # all other situation is invalid, cannot make a move
            turn_dir = None

            #__print for debug
            #print("turn_dir = None: not enough clearance to turn")
            #print(" ")


    else: # something wrong with direction
        turn_dir = None


    # if new position is invalid
    if turn_dir == None:

        # RETURN: empty
        return None

    # if new direction is valid
    else:

        # calculate the new node's direction
        node_direction = (current_node.direction + turn_dir) % 4


    # RETURN: the new node's direction
    return node_direction

    # End of function getNextNodeDirection(current_node, new_position)     



# FUNCTION: astar algo to compute a trip from start node to end node
# RETURN: a list of tuples as a path from the given start to the given end in the given maze: [()]
# RETURN: total cost of the trip: int
def astar(maze, start, end):


    ''' 1. Initializing variables'''

    # Create start and end node
    start_node = Node(None, (start[0], start[1]), start[2])
    start_node.g = start_node.h = start_node.f = 0
    #start_node.visited = 0

    #__ print start node
    #print("start node is", start_node.position, "and facing direction", start_node.direction)


    end_node = Node(None, (end[0], end[1]), end[2])
    end_node.g = end_node.h = end_node.f = 0
    #end_node.visited = 0

    #__ print end node
    #print("end node is", end_node.position, "and facing direction", end_node.direction)

    
    # Initialize an array to store vistied flag for node, range is x (-6, 26), y (-6, 26), direction (0, 4)
    # Even the obstacle at the border of map, facing outside of map, also is in range of visited array
    visited = [[[0 for i in range(4)] for i in range(0-TURN_RADIUS-CLEARANCE, 20+TURN_RADIUS+CLEARANCE)] for i in range(0-TURN_RADIUS-CLEARANCE, 20+TURN_RADIUS+CLEARANCE)]
    
    #__ print visited array
    #visited [0][0][0] = 1
    #print(visited)
    #print("visited:", len(visited), len(visited[0]), len(visited[0][0]), visited[0][0][0])


    # Initialize both open and closed list
    open_list = []
    closed_list = []


    # Add the start node to open list
    open_list.append(start_node)

    #__ print open_list
    #print("open_list.append(start_node):", start_node.position)


    # while Loop the open list until find the end node
    counter = 0  # counter to control infinite loop
    while len(open_list) > 0:

        #print("")
        #print("")

        counter += 1

        #__print for debug
        #print("while loop counter: ", counter)

        # loop 4096 （32 * 32 map * 4 direction）times, still cannot reach the end, probably stuck, kill the function
        if counter > 4096: 
            
            #__print for debug
            #print("too many loops")

            # RETURN: trip = None, total cost = super big number
            return None, 9999


        #__ print message to inform successfully enter the while loop
        #print("Enter while loop:")
        #print("")


        # Find the node in open list with smallest f cost, set it as the current node
        current_node = open_list[0]
        current_index = 0

        for index, item in enumerate(open_list): 
            if item.f < current_node.f:
                current_node = item
                current_index = index


        # Pop current node from open list, add it to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        #__print current node and its h cost
        #print("current_node : ", current_node.position, current_node.direction)
        #print("with a smallest h.cost: ", current_node.h)


        # set the current_node visited status to 1 = visited
        visited[current_node.position[0]][current_node.position[1]][current_node.direction] = 1

        #__print visited status
        #print("current node vistied status:", visited[current_node.position[0]][current_node.position[1]][current_node.direction] )



        # Exit: if found the goal！
        if current_node.position == end_node.position and current_node.direction == end_node.direction:
            
            #__print finishing message
            #print("")
            #print("Found the goal!")

            # initialize empty variable 
            path = []
            total_cost = 0
            current = current_node

            #__print the target node
            #print("current = current_node: ", current.position)

            # backtrack starts
            while current is not None:
                
                # append the current node to the trip
                path.append((current.position[0], current.position[1], current.direction))
                
                #__print backtracked node
                #print("Append node to trip:", current.position, current.direction)

                # add the g cost of current node to total cost of the trip
                total_cost += current.g

                # find current node's parent to backtrack
                current = current.parent

            #__ print total cost of the trip
            #print("Total cost of the trip is",total_cost)


            # RETURN: reversed backtracked path, the total cost of this trip
            return path[::-1], total_cost 



        # If didn't find target, generate children for current node
        children = []

        # for 4 directions moving
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:

            # Get new node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            
            #__ print new node position
            #print("new node : ", node_position)


            # Make sure walkable terrain
            # check if it collide with virtual obstacle area
            if checkAccessible(node_position) == False:
                
                #__print for debug
                #print("node not accessible")

                # if not accessible, skip this child node
                continue


            #__print for debug
            #print("valid node position")

            
            # get direction of the new node
            node_direction = getNextNodeDirection(current_node, new_position)

            #__print direction of new node
            #print("node_direction : ", node_direction)


            # if no direction is return, means the move is not valid
            if node_direction == None:

                # if the move is invalid, skip this child node
                continue
            

            # check if the child node was visited:
            try:
                if visited[node_position[0]][node_position[1]][node_direction] == 1:

                    # if the child node is visited, skip this child node
                    continue
            except:
                print("node is way outside of map")


            # If the child node passed all 3 initial checks (accessibility, direction turn clearance, visited) 
            # create new node
            new_node = Node(current_node, node_position, node_direction)


            # Append the new node to children list for further checks
            children.append(new_node)

            #__print for debug
            #print("children.append(new_node)")
            #print("")


        # Loop through children to further check if they can be add to open list
        for child in children:

            # If Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:

                    # skip
                    continue


            # If Child is NOT on closed list
            # find its f, g, and h cost

            # g cost

            # check turn cost & rev cost
            if child.direction != current_node.direction: # different direction

                # add turn cost
                turn_cost = 2
                # no reverse cost
                rev_cost = 0

            else: # same direction
                # no turn cost
                turn_cost = 0

                # if facing east, child.x < current.x
                if child.direction == 0 and child.position[0] < current_node.position[0]:
                    rev_cost = 1

                # if facing north, child.y < current.y
                elif child.direction == 1 and child.position[1] < current_node.position[1]:
                    rev_cost = 1

                # if facing west, child.x > current.x
                elif child.direction == 2 and child.position[0] > current_node.position[0]:
                    rev_cost = 1

                # if facing south, child.y > current.y
                elif child.direction == 3 and child.position[1] > current_node.position[1]:
                    rev_cost = 1

                # all other situation, no reverse cost
                else:
                    rev_cost = 0

            # add all moving cost together
            child.g = current_node.g + ROBOT_STEP + turn_cost + rev_cost

            #__print g cost
            #print("child.g = ", child.g)


            # h cost

            child.h = (abs(child.position[0] - end_node.position[0])) + (abs(child.position[1] - end_node.position[1]))
            
            #__print h cost
            #print("child.h = ", child.h)


            # f cost
            # f = g + h
            child.f = child.g + child.h

            #__ print f cost
            #print("child.f = ", child.f)
            #print(" ")


            # if Child is already in the open list, 
            for open_node in open_list:
                if child == open_node and child.g > open_node.g: # if child.g > opennode.g

                    #skip it
                    continue

            # if Child not in open list :
            # if Child in open list & child.g is < opennode.g :

            # Add the child to the open list
            open_list.append(child)

    # End of function astar(maze, start, end)



# FUNCTION: it's just a sample of obstacle data to test the trip_planner
def testing():

    obstacles_from_app = [(3, 5, 7, 3), (8, 5, 12, 2), (6, 12, 9, 0), (0, 15, 4, 0), (2, 15, 15, 3)]

    obstacles = []
    for i in range(len(obstacles_from_app)):
        obstacles.append(obstacles_from_app[i][1:])

    #__print for debug
    #print("Obstacles are", obstacles)
    
    markObstaclesOnMAP(obstacles)
    markAccessOnMAP(obstacles)


    start = (0, 11, 0)
    end = (15, 8, 2)

    #start = (4, 2, 1)
    #end = (0, 12, 0)

    print("Planing trip from", start, "to", end, "...")
    print("")


    path, cost = astar(MAP, start, end)
    print("Path is", path)

    #__print for debug
    #print("Cost is", cost)

    # End of function testing()



def main():
    print("trip_planner.py")

    testing()



if __name__ == '__main__':
    main()
