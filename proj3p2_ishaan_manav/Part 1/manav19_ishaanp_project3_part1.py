import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq
from math import dist
import matplotlib.patches as patches

# Importing matplotlib and turning on interactive mode
plt.ion()

# Defining a node class with following functionalities
'''initializing different attributes with self to append them easily
    this consists of c2c,c2g high and low RPMs, parent, c_t = current node
    , n_t  = new node.
    then comparing the total costs for Astar algorithm'''
class Node:
    def __init__(self, x, y, parent, c_t, n_t, U_L, U_R, cost_to_come, cost_to_goal, cost):
        # Initializing the attributes of the Node class
        self.x = x
        self.y = y
        self.parent = parent
        self.c_t = c_t
        self.n_t = n_t
        self.U_L = U_L
        self.U_R = U_R
        self.cost_to_come = cost_to_come
        self.cost_to_goal = cost_to_goal
        self.cost = cost

    def __lt__(self, other):
        # Implementing comparison based on total cost, i.e., cost_to_come + cost_to_goal
        return (self.cost_to_come + self.cost_to_goal) < (other.cost_to_come + other.cost_to_goal)


'''This function plots the curves that a turtle bot might take
    the variables are:
     x_i, y_i, Thetai: Input point's coordinates
     U_L, U_R: inputs for left and right wheels
     c: control actions execute_action() function
     plot: Plotting flag (returns a bool value)
     node_lists, find_path_list: stores nodes as lists for plotting.
     '''

def curves(x_i, y_i, Thetai, U_L, U_R, c, plot, node_lists, find_path_list):
    #difining bot inputs taking vaalues from the turtlebot3 burger    
    t = 0
    r = 0.033
    L = 0.160
    dt = 0.8
    cost = 0
    x_g = x_i
    y_g = y_i
    Theta_end = math.radians(Thetai)  # Convert to radians
    #finding new goals and theta values for each time stamp
    while t < 1:
        t += dt
        x_s = x_g
        y_s = y_g
        x_g += r * 0.5 * (U_L + U_R) * math.cos(Theta_end) * dt
        y_g += r * 0.5 * (U_L + U_R) * math.sin(Theta_end) * dt
        Theta_end += (r / L) * (U_R - U_L) * dt
        #if the goal lies in plot, calculate cost and append and then plot the path
        if execute_action(x_g, y_g, r, c):
            if plot == 0:
                cost_to_goal = dist((x_s, y_s), (x_g, y_g))
                cost += cost_to_goal
                node_lists.append((x_g, y_g))
                find_path_list.append((x_s, y_s))
            elif plot == 1:
                plt.plot([x_s, x_g], [y_s, y_g], color="red")
        else:
            return None
    # Convert theta back to degrees
    Theta_end = math.degrees(Theta_end)  
    return [x_g, y_g, Theta_end, cost, node_lists, find_path_list]


# Generateing unique key for each node 
def key(node):
    key = 1000*node.x + 111*node.y 
    return key

# Function to implement A star 
'''this is the function to impelement Astar,
    the new code merges the previus Astar and nodes class,
    to make the method as close to requirements in proj 3 phase 1.
    
    go to nodes in open list, append closed list and change nodes based on total costs
    '''
def Astar(s_node, g_node, rpm1, rpm2, radius, clearance):

    if check_goal(s_node, g_node):
        return 1, None, None
    #define nodes, path, open list and closed lists
    node_lists = []
    find_path_list = []
    closed_list = {}
    open_list = {}
    open_list[key(s_node)] = s_node
    prior_list = []
    #defining 8 set action set
    moves = [[rpm1, 0], [0, rpm1], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]
    heapq.heappush(prior_list, [s_node.cost, s_node])
    #append current nodes, get current ID
    while len(prior_list) != 0:

        c_nodes = heapq.heappop(prior_list)[1]
        c_id = key(c_nodes)
        
        #if goal is reached return the path and set current nodes as goal nodes 
        if check_goal(c_nodes, g_node):
            g_node.parent = c_nodes.parent
            g_node.cost = c_nodes.cost
            print("Goal Node found")
            return 1, node_lists, find_path_list
        #append closed list and delete from open list
        if c_id in closed_list:  
            continue
        else:
            closed_list[c_id] = c_nodes
        
        del open_list[c_id]
        #define action set for current nodes and plot the curves
        for move in moves:
            action = curves(c_nodes.x, c_nodes.y, c_nodes.c_t, move[0], move[1],
                            clearance, 0, node_lists, find_path_list)
            
            #round off the x,y to nearest integar and round off theta to
            #nearest value of 30 degs
            if action is not None:
                angle = action[2]
                x = round(action[0] * 10) / 10
                y = round(action[1] * 10) / 10
                theta = round(angle / 15) * 15
                
                #calculate cost to goal, total cost and make new nodes
                c_t = c_nodes.n_t - theta
                cost_to_goal = dist((x,y), (g_node.x, g_node.y))
                new_node = Node(x, y, c_nodes, theta, c_t, move[0], move[1], c_nodes.cost_to_come+action[3], cost_to_goal, c_nodes.cost_to_come+action[3]+cost_to_goal)
                new_node_id = key(new_node)
                #if the new node is a valid node, add the node in the closed list 
                if not execute_action(new_node.x, new_node.y, radius, clearance):
                    continue
                elif new_node_id in closed_list:
                    continue
                #if in open list, new cost is less then old cost then append the parent and the cost
                if new_node_id in open_list:
                    if new_node.cost < open_list[new_node_id].cost:
                        open_list[new_node_id].cost = new_node.cost
                        open_list[new_node_id].parent = new_node

                else:
                    open_list[new_node_id] = new_node
                    heapq.heappush(prior_list, [ open_list[new_node_id].cost, open_list[new_node_id]])
            
    return 0, node_lists, find_path_list

# This function checks if the node is obstacle or not 
'''this function sets the boundries for the map
    this uses the half plane method and returns a bool value'''
def half_plane_obstcles(x, y, radius, clearance):
    #clearance value 
    c = radius + clearance 

    # Circular obstacle in the map
    circle = (x - 4)**2 + (y - 1.1)**2 <= (0.5 + c)**2 
    
    # Rectangle obstacles in the map
    rectangle2 = (1.5 - c <= x <= 1.625 + c) and (0.75 - c <= y)   
    rectangle3 = (2.5 - c <= x <= 2.625 + c) and (y <= 1.25 + c)   
 
    # setting borders based on clearence
    border1 = x <= 0 + c     
    border2 = x >= 5.99 - c
    border3 = y <= 0 + c
    border4 = y >= 1.99 - c

    # Check the presence of obstacle and border and returns a bool value
    if circle or rectangle2 or rectangle3 or border1 or border2 or border3 or border4:
        return True
    else:
        return False

    
#This fucntion checks if the move is valid or not and returns a bool value 
def execute_action(x,y, r,c):
    
    if half_plane_obstcles(x, y, r, c):
        return False
    else:
        return True

#this function checks weather the goal node is reached
#this is done based on the distance bw current and goal node
def check_goal(c, g):

    dt = dist((c.x, c.y), (g.x, g.y))

    if dt < 0.15:
        return True
    else:
        return False
    


'''this is the function for back tracking the path 
    this will return x,y,theta for the path'''
def backtracker(g_node): 
    x_path, y_path, theta_path = [g_node.x], [g_node.y], [g_node.c_t]
    U_L_list,U_R_list = [g_node.U_L], [g_node.U_R]
    parent_node = g_node.parent

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.c_t)
        U_L_list.append(parent_node.U_L)
        U_R_list.append(parent_node.U_R)
        parent_node = parent_node.parent

    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()
    U_L_list.reverse()
    U_R_list.reverse()
    print(U_L_list)
    print(U_R_list)

    x, y, theta = np.asarray(x_path), np.asarray(y_path), np.array(theta_path)
    return x, y, theta



if __name__ == '__main__':
    
    #define the workspace and robot dimensions
    width = 6
    height = 2
    robot_radius = 0.038
    #define the clearence
    clearance = int(input("Enter clearance of robot in mm: "))/1000
    #define the high and low rpms
    RPM1 = int(input("Enter the Low RPM: "))
    RPM2 = int(input("Enter the High RPM: "))
    #take start and goal coordinates
    start_coordinates = input("Enter starting x coordintes, y coordinates and orientation seperated by spaces in mm ")
    start_x, start_y,start_theta = start_coordinates.split()
    start_x = int(start_x)/1000
    start_x = float(start_x)
    start_y = int(start_y)/1000
    start_y = float(start_y)
    start_theta = int(start_theta)

    num = int(start_theta)
    remainder = num % 30
    if remainder < 15:
        start_theta = num - remainder
    else:
        start_theta = num + (30 - remainder)
                      
    goal_coordinates = input("Enter Goal node x coordinte, y coordinate seperated by spaces in mm ")
    goal_x,goal_y = goal_coordinates.split()
    goal_x = int(goal_x)/1000
    goal_x = float(goal_x)
    goal_y = int(goal_y)/1000
    goal_y = float(goal_y)

    #checking start and goal nodes
    if not execute_action(start_x, start_y, robot_radius, clearance):
        print("Start node is either invalid or in obstacle space")
        exit(-1)

    if not execute_action(goal_x, goal_y, robot_radius, clearance):
        print("Goal node is either invalid or in obstacle space")
        exit(-1)
    #starting timer 
    timer_start = time.time()
    #taking c2g
    cost_to_goal = dist((start_x, start_y), (goal_x, goal_y))
    #adding total cost
    cost = cost_to_goal
    #append start node
    s_node = Node(start_x, start_y, -1, start_theta, 0, 0, 0, 0, cost_to_goal, cost)
    #append goal node
    g_node = Node(goal_x, goal_y, -1, 0, 0, 0, 0, cost_to_goal, 0, cost)
    #start Astar
    flag, node_lists, find_path_list = Astar(s_node, g_node, RPM1, RPM2, robot_radius, clearance)
    #start backtracking
    x_path, y_path, theta_path = backtracker(g_node)
    
    #plotting figures, axes for final plot
    fig, axes = plt.subplots()
    axes.set(xlim=(0, 6), ylim=(0, 2))

    circle = plt.Circle((4, 1.1), 0.5, fill='True', color='blue')

    rectangle2 = patches.Rectangle((1.5, 0.75), 0.15, 1.25, color= 'blue')
    rectangle3 = patches.Rectangle((2.5, 0), 0.15, 1.25, color='blue')

    axes.set_aspect('equal')
    axes.add_artist(circle)
    axes.add_artist(rectangle2)
    axes.add_patch(rectangle3)

    plt.plot(s_node.x, s_node.y, "Dw")
    plt.plot(g_node.x, g_node.y, "Dg")

    for l in range(len(node_lists)):
        plt.plot([find_path_list[l][0], node_lists[l][0]], [find_path_list[l][1], node_lists[l][1]], color="red")
    plt.plot(x_path, y_path, ':r')
    #stopping final timer
    timer_stop = time.time()

    count_time = timer_stop - timer_start
    print("The Total Runtime is: ", count_time)

    plt.show()
    plt.close('all')
