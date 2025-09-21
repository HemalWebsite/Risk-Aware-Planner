

import typing
#Implements a priority queue using a binary heap, used for efficiently getting the node with the lowest cost.
import heapq
import math
from map_info import Coordinate, Destination, MapInfo
#Used for handling the risk map as a 2D array.
import numpy as np

# ----------------- Node -----------------
class Node:
    def __init__(self, position, g=0, length=0, risk_cost=0, h=0, parent=None):
        self.position = position  # (e, n)
        self.g = g                # total cost = alpha*length + beta*risk
        self.length = length      # actual distance traveled
        self.risk_cost = risk_cost # How much risk has been accumlated
        self.h = h # Estimated distnace to the goal
        self.f = g + h # Total estimated cost
        self.parent = parent # Pointer to the previous node in the path (to reconstruct the path).

    def __lt__(self, other):
        # Tells Python how to compare two Node objects (by f value)
        return self.f < other.f

# ----------------- Heuristic -----------------
def heuristic(a, b):
    #Euclidean distance heuristic: Returns the straight line distance between 2 points
    return math.hypot(a[0] - b[0], a[1] - b[1])

# ----------------- A* Search -----------------
def astar_search(start, goal, max_range, risk_map, alpha=10000000.0, beta=100.0):
    open_set = [] # Priority queue (nodes to explore next).
    closed_set = set() # Already visited coordinates.
    start_node = Node(start, g=0, length=0, risk_cost=0, h=heuristic(start, goal))
    heapq.heappush(open_set, start_node) # Create the start node and push it into the open set

    while open_set: # While they are still nodes left to explore
        current = heapq.heappop(open_set) # Pop the node from the stack with the lowest f value
        pos_int = (int(round(current.position[0])), int(round(current.position[1]))) 

        # Skip if already visited else mark as visitied
        if pos_int in closed_set:
            continue
        closed_set.add(pos_int)

        # Goal reached if we are within 1 unit from the goal nod 
        if heuristic(current.position, goal) < 1.0:
            # Empty list that will eventually hold the coordinates of the path
            path = []
            # Loop until there are no more parents.
            node = current
            while node:
                path.insert(0, (int(round(node.position[0])), int(round(node.position[1]))))
                # Increment/Decrement Statement move one step backward
                node = node.parent
            return path

        # 8-connected neighbors: Diagonalls are sqrt 2 because of pythogarours equation
        for dx, dy, step_cost in [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),
                                  (-1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),
                                  (1,-1,math.sqrt(2)),(1,1,math.sqrt(2))]:
            nx, ny = pos_int[0] + dx, pos_int[1] + dy

            # Check bounds -- skip if its outside map boundaries
            if nx < 0 or nx >= risk_map.shape[0] or ny < 0 or ny >= risk_map.shape[1]:
                continue
            
            # Skip if cell is catagories as "KEEP_OUT"
            risk_val = risk_map[nx, ny]
            if risk_val == MapInfo.KEEP_OUT_VALUE:
                continue
            
            # Skips if over max range
            new_length = current.length + step_cost
            if new_length > max_range:
                continue

            new_risk = current.risk_cost + (1.0 if risk_val == MapInfo.HIGH_RISK_VALUE else 0.0)
            new_g = alpha * new_length + beta * new_risk
            h = heuristic((nx, ny), goal)

            # Create a new node and push to the open set.
            new_node = Node((nx, ny), g=new_g, length=new_length, risk_cost=new_risk, h=h, parent=current)
            heapq.heappush(open_set, new_node)

    return None  # no path found

# ----------------- PathPlanner -----------------
class PathPlanner:
    def __init__(self, map_info: MapInfo, destinations: typing.List[Destination], alpha=1.0, beta=5.0):
        self.map_info = map_info
        self.destinations = destinations
        self.risk_map = map_info.risk_zones
        self.alpha = alpha
        self.beta = beta

    def plan_paths(self):
        # Gets starting coordinate from map, converts it to (int e, int n)
        start_coord = self.map_info.start_coord
        start_tuple = (int(round(start_coord.e)), int(round(start_coord.n)))

        # Convert to integer grid value 
        for site in self.destinations:
            goal_coord = site.coord
            goal_tuple = (int(round(goal_coord.e)), int(round(goal_coord.n)))

            path_tuples = astar_search(
                start_tuple,
                goal_tuple,
                self.map_info.maximum_range,
                self.risk_map,
                alpha=self.alpha,
                beta=self.beta
            )

            if path_tuples:
                path_coords = [Coordinate(e, n) for e, n in path_tuples]
                site.set_path(path_coords)
            else:
                site.set_path([])
