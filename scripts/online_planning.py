#!/usr/bin/python

import numpy as np
import math as m
import numpy as np
from Node import Node
import copy
import random 
import matplotlib.pyplot as plt
import matplotlib.patches as patches


def wrap_angle(angle): 
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )


def dist_between_points (p1, p2):
    return m.sqrt((p2.coord[0] - p1.coord[0])**2 + (p2.coord[1] - p1.coord[1])**2)


class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1, is_unknown_valid=True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    

    def map_set(self, map):
        bin_map = np.zeros_like(map)
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i,j] == 100:
                    bin_map[i,j] = 1
        """    plt.imshow(bin_map, cmap='gray', interpolation='nearest')
        plt.colorbar()  # Optional: to show the color bar
        plt.title("Grayscale Image")
        plt.show() """
        flip_map = self.rotate_and_flip(bin_map)

        return np.array(flip_map)
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = self.map_set(data)
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
    
    # Given a pose, returs true if the pose is not in collision and false othewise.
    #def is_valid(self, pose): 

    def is_valid(self, p, distance, map, resolution, orig, shape):
        
        # convert world robot position to map coordinates using method __position_to_map__
        # check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.
        print ("the robot is at ", p)

        # robot area in world coordinates
        pfrom = np.array((p[0] - distance, p[1] + distance)) 
        pto = np.array((p[0] + distance, p[1] - distance))
        print ("top left corner: ", pfrom)
        print ("bottom right corner: ", pto)

        fromx, fromy = self.position_to_map__(pfrom,orig,resolution,shape) # get x,y pixels from top left corner of the robot threshold
        tox, toy = self.position_to_map__(pto,orig, resolution, shape) # get x,y pixels from bottom right corner of the robot threshold

        print ("top left corner pixels:", fromx,fromy)
        print ("botom right corner: ", tox,toy)
        print (map[fromx:tox,fromy:toy])
        for i in range(fromx, tox):
            for j in range(fromy, toy):
                if map[i,j] == 0: return False
        return True

    # Given a path, returs true if thaose path is not in collision and false othewise.
    def check_path(self, path):
        pass
        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 

    # Transform position with respect the map origin to cell coordinates
    def position_to_map(self, point):
        origin = self.origin
        resolution = self.resolution
        shape = self.map.shape

        # this function variates from the requiered, since it asumes that the reference from the grid map is in the top left corner 
        x,y = point-origin #x,y are already in map (occupancy grid) coordinates
        xmap, ymap =  int(x/resolution), shape[1] - int(y/resolution)
        if x < 0 or y < 0 or x > shape[0] or y > shape[1]: return None
        else: return np.array([xmap,ymap])

    def map_to_position(self, m):
        origin = self.origin
        resolution = self.resolution
        shape = self.map.shape
        if m[0] < 0 or m[1] < 0 or m[0] > shape[0] or m[1] > shape[1]: return None
        x,y = m[0]*resolution, (-m[1] + shape[1] )*resolution 
        x,y = x + origin[0],y+ origin[1]
        return np.array((x,y))
    

    def rotate_and_flip(self, matrix):
        # Rotate the matrix 90 clockwise (Transpose and reverse each row)
        rotated_matrix = np.array(matrix).T[::-1]
        
        # Flip the matrix horizontally (reverse the columns)
        flipped_matrix = rotated_matrix[:, ::-1]
        
        return flipped_matrix

        
# Define Planner class (you can take code from Autonopmous Systems course!)
class Planner:
    def  __init__(self, state_validity_checker, max_iterations, dominion=[-10, 10, -10, 10], goal=None, start=None, step_size=0.5, goal_threshold=0.5):
        # define constructor ...
        self.max_iterations = max_iterations
        self.goal = Node(goal[0], goal[1])##
        self.start = Node(start[0], start[1])
        self.tree = [self.start]
        self.robot_distance = state_validity_checker.distance
        self.map_resolution = state_validity_checker.resolution
        self.goal_threshold = goal_threshold
 
        self.step_size = 10
        self.map = state_validity_checker.map
        pass
    
    def compute_path(self):
        for k in range(self.max_iterations):
            #print("tree: ", self.tree)
            # 1. Sample a random node
            random_node = self.get_random_node(0.2) 
            #print("random node: ", random_node)

            # 2. Find the nearest node in the tree
            nearest = self.nearest_node(random_node)
            print("nearest node: ", nearest)
            
            # 3. New configuration
            qnew = self.new_config(nearest, random_node, self.step_size)
            print("new config: ", qnew)

            # 4. Check if the new configuration is collision free
            if self.collision_free((nearest.x,nearest.y), (qnew.x,qnew.y)):
                qnew.parent = nearest

                # 5. Check if goal is reached
                self.tree.append(qnew)
                
                if dist_between_points(qnew, self.goal) <= self.goal_threshold:
                    #print(f"Goal reached in {k} iterations")
                    """ 
                    if self.pth_found_after == None: 
                        self.pth_found_after=k
                     """
                    path = self.build_path(qnew)
                    print("path found: ", path)
                    return self.tree, path
                    
        raise AssertionError("Path not found\n")
        pass

    def get_random_node(self,p):
        if random.random() < p:
            return self.goal
        # Get all free cells (where grid is 0)
        dist = int(m.ceil(self.robot_distance/self.map_resolution))
        free_positions = np.argwhere(self.map[dist:-dist,dist:-dist] == 0)
        # Randomly choose one of the free cells 
        random_index = np.random.choice(len(free_positions))
        node = Node(free_positions[random_index][0],free_positions[random_index][1])

        return node
    
    def nearest_node(self, random_node):
        return min(self.tree, key=lambda node: dist_between_points(node, random_node))
    
    def new_config(self, nearest, random_node, step_size):
        distance = dist_between_points(nearest, random_node)
        if distance <= step_size:
            return random_node
            
        else:
            theta = m.atan2(random_node.coord[1] - nearest.coord[1], random_node.coord[0] - nearest.coord[0])
            return Node(int(nearest.coord[0] + step_size * m.cos(theta)), int(nearest.coord[1] + step_size * m.sin(theta)))
        
    def collision_free(self, point1, point2):
        num_steps = max(abs(point2[0] - point1[0]), abs(point2[1] - point1[1])) + 1
        # Interpolate x and y coordinates
        x_values = np.linspace(point1[0], point2[0], num_steps)
        y_values = np.linspace(point1[1], point2[1], num_steps)
        
        # Combine x and y into pixel coordinates
        pixels = list(zip((np.floor(x_values)).astype(int), (np.floor(y_values)).astype(int)))

        for i in range(len(pixels)):
            return self.is_valid_pixel(pixels[i])
        
    def is_valid_pixel (self, pixel):

        # y is row, x is column

        robot_area_p = self.robot_distance / self.map_resolution
        fromx, fromy = pixel[0]-robot_area_p, pixel[1]-robot_area_p
        tox, toy = pixel[0] + robot_area_p, pixel[1]+robot_area_p, # get x,y pixels from bottom right corner of the robot threshold
        
        """
        fig, ax = plt.subplots()
        plt.imshow(self.map, cmap='viridis', interpolation='nearest')
        width = tox - fromx
        height = toy - fromy
        square = patches.Rectangle((fromx, fromy), width, height, linewidth=2, edgecolor='r', facecolor='none')
    
        # Add the rectangle to the plot
        ax.add_patch(square)
        plt.colorbar()
        plt.show() """

        if int(m.floor(fromx)) < 0 and int(m.floor(fromy)) < 0 and int(m.ceil(tox)) > self.map.shape[0] and int(m.ceil(toy)) > self.map.shape[1]: return False
        print(range(int(m.floor(fromy)), int(m.ceil(toy))))
        print(range(int(m.floor(fromx)), int(m.ceil(tox))))
        print(self.robot_distance)
        for i in range(int(m.floor(fromy)), int(m.ceil(toy))-1):
            for j in range(int(m.floor(fromx)), int(m.ceil(tox))-1):
                if self.map[i,j] == 1: 
                    print ("pixel is invalid")
                    return False
        print ("pixel is valid")
        return True
    
    def get_edges(self,nodes):
        edges = []
        for node in nodes:
            if node.parent:
                edges.append((nodes.index(node.parent), nodes.index(node)))
        return edges
    
    
    def smooth_path(self):
        # Optionally, you can implement a finction to smooth the RRT path.
        pass

    def build_path(self, node):
        path = []
        while node.parent is not None:
            path.append(node.coord)
            node = node.parent
        path.append(node.coord)
        return path[::-1]



# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker, dominion, max_iterations=5000):


    # TODO: Plan a path from start_p to goal_p inside dominiont using a Planner Object and the 
    # StateValidityChecker Object previously defined.
    # TODO: if solved, return a list with the [x, y] points in the solution path.
    # example: [[x1, y1], [x2, y2], ...]
    # TODO: Ensure that the path brings the robot to the goal (with a small tolerance)!

    start_p, goal_p = state_validity_checker.position_to_map(start_p), state_validity_checker.position_to_map(goal_p)
    planner = Planner(state_validity_checker, max_iterations, dominion, goal_p, start_p)
    tree, path = planner.compute_path()
    print(path)
    for i in range(len(path)):
        path[i] = [state_validity_checker.map_to_position(path[i])[:]]
    print (path)
    return path





# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(current, current_yaw, goal, Kv=0.5, Kw=0.5):
    
    # Use a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # To avoid strange curves, first correct the orientation and then the distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    # This function should return only  linear velocity (v) and angular velocity (w)

    angle_error = current_yaw - m.atan2( goal[1]- current[1],  goal[0]-current[0]) 
    if abs(angle_error) > 0.0873 :# anglular error is greater than anglular threshold 0.0873 rad = 5 deg
        w = angle_error * Kw
        v = 0
    else:
        w = 0
        v = Kv * dist_between_points(current,goal)
    
    return  v, w
