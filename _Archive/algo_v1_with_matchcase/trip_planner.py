# TITLE: trip_planner.py
# AUTHOR: Xiong Ying
# DATE: 09/13/2022 Wednesday
# PURPOSE: To compute a trip between 2 vertex with obstacle avoidance and turn area clearance checking


from map import *


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None, direction=None):
        self.parent = parent
        self.position = position
        # 0 = East, 1 = North, 2 = West, 3 = South
        self.direction = direction

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position, self.direction == other.direction


# Function: astar algo to compute a trip from start node to end node
# Return: a list of tuples as a path from the given start to the given end in the given maze
# Return: total cost of the trip
def astar(maze, start, end):


    # Create start and end node
    start_node = Node(None, (start[0], start[1]), start[2])
    start_node.g = start_node.h = start_node.f = 0
    #print("start node is", start_node.position, "and facing direction", start_node.direction)

    end_node = Node(None, (end[0], end[1]), end[2])
    end_node.g = end_node.h = end_node.f = 0
    #print("end node is", end_node.position, "and facing direction", end_node.direction)

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)
    #print("open_list.append(start_node):", start_node.position)

    # Loop until you find the end
    counter = 0
    while len(open_list) > 0:

        counter += 1
        if counter > 999: # probably stuck
            #print("too many")
            return None, 9999

        #print("")
        #print("Enter while loop:")

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        #print("open_list.pop(current_index):", open_list)
        #print("closed_list.append(current_node):", closed_list)
        #print("current_node = ", current_node.position)

        # Found the goal！
        if current_node.position == end_node.position and current_node.direction == end_node.direction:
            #print(end_node)
            #print("found the goal!")

            path = []
            total_cost = 0
            current = current_node
            #print("current = current_node: ", current.position)
            while current is not None:
                #print("while current is not None: ")
                path.append((current.position[0], current.position[1], current.direction))
                total_cost += current.f
                #print("total_cost is ",total_cost)
                #print("path.append(current.position): ", current.position)
                current = current.parent
                #print("current = current.parent: ", current)
            return path[::-1], total_cost # Return reversed path and the total cost of this trip

        # Generate children
        children = []
        #for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            #print("for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: ", new_position)

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            #print("node_position : ", node_position)

            # Make sure within range, put in checkAccessible already
            #if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                #continue

            # Make sure walkable terrain
            # check the 3 by 3 area
            if checkAccessible(node_position) == False:
            #if maze[node_position[0]][node_position[1]] != 0:
                #print("node not accessible")
                continue

            #print("valid node position")


            # get direction of the new node
            match current_node.direction:
                # east
                case 0 :
                    if new_position[1] == 0: # same direction
                        turn_dir = 0
                    elif new_position[1] > 0 and checkTurnClearance(current_node.position, 0, 0) == True: # turn left if have 4 by 4 clearance
                        turn_dir = 1
                    elif new_position[1] < 0 and checkTurnClearance(current_node.position, 0, ROBOT_WIDTH-TURN_RADIUS) == True: # turn right if have 4 by 4 clearance
                        turn_dir = -1
                    else:
                        turn_dir = None


                case 2: # west
                    if new_position[1] == 0: # same direction
                        turn_dir = 0
                    elif new_position[1] < 0 and checkTurnClearance(current_node.position, ROBOT_WIDTH-TURN_RADIUS, ROBOT_WIDTH-TURN_RADIUS) == True: # turn left if have 4 by 4 clearance
                        turn_dir = 1
                    elif new_position[1] > 0 and checkTurnClearance(current_node.position, ROBOT_WIDTH-TURN_RADIUS, 0) == True: # turn right if have 4 by 4 clearance
                        turn_dir = -1
                    else:
                        turn_dir = None

                # north
                case 1:
                    if new_position[0] == 0: # same direction
                        turn_dir = 0
                    elif new_position[0] < 0 and checkTurnClearance(current_node.position, ROBOT_WIDTH-TURN_RADIUS, 0) == True: # turn left if have 4 by 4 clearance
                        turn_dir = 1
                    elif new_position[0] > 0 and checkTurnClearance(current_node.position, 0, 0) == True: # turn right if have 4 by 4 clearance
                        turn_dir = -1
                    else:
                        turn_dir = None



                case 3: # south
                    if new_position[0] == 0: # same direction
                        turn_dir = 0
                    elif new_position[0] > 0 and checkTurnClearance(current_node.position, 0, ROBOT_WIDTH-TURN_RADIUS) == True: # turn left
                        turn_dir = 1
                    elif new_position[0] < 0 and checkTurnClearance(current_node.position, ROBOT_WIDTH-TURN_RADIUS, ROBOT_WIDTH-TURN_RADIUS) == True: # turn right
                        turn_dir = -1
                    else:
                        turn_dir = None


            if turn_dir == None:
                continue

            node_direction = (current_node.direction + turn_dir) % 4

            #print("node_direction : ", node_direction)


            # Create new node
            new_node = Node(current_node, node_position, node_direction)
            #print("new node is ",new_node)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            if child.direction != current_node.direction:
                turn_cost = 2
            else:
                turn_cost = 0

            child.g = current_node.g + 1 + turn_cost
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)



def main():
    print("trip_planner.py")



if __name__ == '__main__':
    main()
