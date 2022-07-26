# TITLE: main.py
# AUTHOR: Xiong Ying
# CREATE DATE: 14/07/2022
# UPDATE DATE: 26/7/2022
# PURPOSE: To invoke the algo to find a Hamiltonian Path for given obstacles


# import modules

from path_finder import *

from map import*
from trip_planner import*


# main() function
def main():

    # Get info from Bluetooth



    # obstacles_from_app FORMAT:
    # tuple = (obstacle_id, x coordinate, y coordinate, direction)
    # a tuple with the id, the position of the obstacle and image facing direction

    # obstacle_id = the id assigned to the obstacle by the app
    # x coordinate = x coordinate of the bottom left corner of obstacle
    # y coordinate = y coordinate of the bottom left corner of obstacle
    # direction = { 0:East, 1:North, 2:West, 3:South }

    # Sample obstacles for testing
    obstacles_from_app = [(1, 5, 7, 3), (2, 5, 13, 2), (3, 12, 9, 0), (4, 15, 4, 1), (5, 15, 15, 3)]


    # Be Cautious! If on permutation, computational time might explode

    permutation = False # By default, set to False

    # invoke path finder
    # RETURN:
    # movement = the instructions to move : list of list [[]]
    # obstacles_seq = sequence of obstacles to scan along the path : list of tuple [()]

    if permutation == True:
        movement, obstacles_id = planShortestPath(obstacles_from_app)
    else:
        movement, obstacles_id = planPath(obstacles_from_app)




if __name__ == '__main__':
    main()
