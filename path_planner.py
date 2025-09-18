
# """
#     This module is your primary workspace. Add whatever helper functions, classes, data structures, imports... etc here.

#     We expect most results will utilize more than just dumping code into the plan_paths()
#         function, that just serves as a meaningful entry point.

#     In order for the rest of the scoring to work, you need to make sure you have correctly
#         populated the Destination.path for each result you produce.
# """
# import typing
# from queue import PriorityQueue

# import numpy as np
# from typing import Dict

# from map_info import Coordinate, Destination, MapInfo


# class PathPlanner:
#     def __init__(self, map_info: MapInfo, destinations: typing.List["Destination"]):
#         self.map_info: MapInfo = map_info
#         self.destinations: typing.List["Destination"] = destinations

#     def plan_paths(self):
#         """
#         This is the function you should re-write. It is expected to mutate the list of
#         destinations by calling each Destination's set_path() with the resulting
#         path as an argument.

#         The default construction shows this format, and should produce 10 invalid paths.
#         """
#         for site in self.destinations:
#             # YOUR CODE REPLACES THIS / WILL PLUG IN HERE
#             path_array = np.linspace(self.map_info.start_coord, site.coord, 10)
#             path_coords = [Coordinate(arr[0], arr[1]) for arr in path_array]

#             # Once you have a solution for the site - populate it like this:
#             site.set_path(path_coords)











"""
    This module is your primary workspace. Add whatever helper functions, classes, data structures, imports... etc here.

    We expect most results will utilize more than just dumping code into the plan_paths()
        function, that just serves as a meaningful entry point.

    In order for the rest of the scoring to work, you need to make sure you have correctly
        populated the Destination.path for each result you produce.
"""
import typing
import heapq
import math
from typing import Dict

# import numpy as np
import numpy as np
risk_zones = np.load('risk_zones.npy')
print(risk_zones[26, 50])
print([risk_zones[dest.n_coord, dest.e_coord] for dest in destinations])


from map_info import Coordinate, Destination, MapInfo


class Node:
   
    def __init__(self, position: Coordinate, g=0, length=0, risk_cost=0, h=0, parent=None):
        self.position = position
        self.g = g                 # combined cost used for A* (depends on weights)
        self.length = length       # actual distance traveled
        self.risk_cost = risk_cost # accumulated risk
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        
        return self.f < other.f

def heuristic(node: Coordinate, goal: Coordinate) -> float:
    return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)

def create_grid_graph(map_info: MapInfo, risk_zones: np.ndarray):
    width = risk_zones.shape[1]  # number of columns
    depth = risk_zones.shape[0]  # number of rows
    graph = {}

    for i in range(width):
        for j in range(depth):
            current_coord = (i, j)
            if risk_zones[j][i] == MapInfo.KEEP_OUT_VALUE:
                continue

            neighbors = {}
            directions = [
                (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))
            ]

            for dx, dy, step_cost in directions:
                nx, ny = i + dx, j + dy
                if 0 <= nx < width and 0 <= ny < depth and risk_zones[ny][nx] != MapInfo.KEEP_OUT_VALUE:
                    neighbors[(nx, ny)] = step_cost

            if neighbors:
                graph[current_coord] = neighbors

    return graph


def astar_search(graph: Dict, start: Coordinate, goal: Coordinate, max_range: float, risk_map: np.ndarray) -> typing.List[Coordinate]:
    open_set = []
    closed_set = set()

    start_node = Node(start, 0, heuristic(start, goal))
    heapq.heappush(open_set, start_node)

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node.position == goal:
            path = []
            while current_node:
                path.insert(0, current_node.position)
                current_node = current_node.parent
            return path

        closed_set.add(current_node.position)

        for neighbor in graph.get(current_node.position, {}):
            if neighbor in closed_set:
                continue
            
            nx, ny = neighbor
            risk_val = risk_map[ny][nx]

            step_length = graph[current_node.position][neighbor]
            new_length = current_node.length + step_length
            new_risk = current_node.risk_cost + risk_val

            if new_length > max_range:
                continue

            # Tuned weights for balancing length and risk
            alpha = 1.0  
            beta = 100.0   

            new_g = alpha * new_length + beta * new_risk
            heuristic_val = heuristic(neighbor, goal)
            new_node = Node(neighbor, g=new_g, length=new_length, risk_cost=new_risk, h=heuristic_val, parent=current_node)

            existing_node = next((node for node in open_set if node.position == neighbor), None)
            if existing_node and existing_node.g <= new_g:
                continue

            heapq.heappush(open_set, new_node)

    return None

class PathPlanner:
    def __init__(self, map_info: MapInfo, destinations: typing.List["Destination"]):
        self.map_info: MapInfo = map_info
        self.destinations: typing.List["Destination"] = destinations
        self.graph = create_grid_graph(self.map_info, self.map_info.risk_zones)  # <-- updated here

    def plan_paths(self):
        # start_coord = (int(self.map_info.start_coord.e), int(self.map_info.start_coord.n))
        # start_coord = (min(start_coord[0], risk_zones.shape[1]-1),
        #        start_coord[1])
        start_coord = (int(self.map_info.start_coord.e), int(self.map_info.start_coord.n))

        # clip to stay inside the map bounds
        risk_zones = self.map_info.risk_zones
        #getting out of bounds error
        start_coord = (
            min(max(int(self.map_info.start_coord.e), 0), risk_zones.shape[1]-1),
            min(max(int(self.map_info.start_coord.n), 0), risk_zones.shape[0]-1)
        )

        for site in self.destinations:
            goal_coord = (int(site.coord.e), int(site.coord.n))

            if start_coord not in self.graph:
                print(f"Start coordinate {start_coord} is invalid (keep-out or out of bounds).")
                site.set_path([])
                continue

            if goal_coord not in self.graph:
                print(f"Goal coordinate {goal_coord} is invalid (keep-out or out of bounds).")
                site.set_path([])
                continue

            
            path_coords_tuple = astar_search(
                self.graph,
                start_coord,
                goal_coord,
                self.map_info.maximum_range,
                self.map_info.risk_zones  # <-- updated here
            )
            
            if path_coords_tuple:
                path_coords = [Coordinate(float(x), float(y)) for x, y in path_coords_tuple]
                site.set_path(path_coords)
            else:
                site.set_path([])



