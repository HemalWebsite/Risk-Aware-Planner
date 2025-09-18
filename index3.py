# #This 

# import heapq
# import math

# class Node:
#     def __init__(self, position, g=0, length=0, risk_cost=0, h=0, parent=None):
#         self.position = position
#         self.g = g                 # combined cost used for A* (depends on weights)
#         self.length = length       # actual distance traveled
#         self.risk_cost = risk_cost # accumulated risk
#         self.h = h
#         self.f = g + h
#         self.parent = parent

#     def __lt__(self, other):
#         return self.f < other.f


# #main function contains the core logic of the A* algorithmn
# def astar_search(graph, start, goal, max_range, risk_map):
#     open_set = []
#     closed_set = set()

#     start_node = Node(start, 0, heuristic(start, goal))
#     heapq.heappush(open_set, start_node)

#     while open_set:
#         current_node = heapq.heappop(open_set)

#         if current_node.position == goal:
#             # store the total length first
#             total_length = current_node.length
            
#             if total_length > max_range:
#                 return None  # path exceeds max range

#             # Reconstruct path
#             path = []
#             while current_node:
#                 path.insert(0, current_node.position)
#                 current_node = current_node.parent
#             return path


#         closed_set.add(current_node.position)

#         for neighbor in graph[current_node.position]:
#             if neighbor not in closed_set:
#                 nx, ny = neighbor
#                 risk_val = risk_map[ny][nx]
#                 if risk_val == 2:
#                     continue

#                 step_length = graph[current_node.position][neighbor]  # 1 or √2
#                 new_length = current_node.length + step_length
#                 new_risk = current_node.risk_cost + risk_val

#                 # Maximum distance constraint
#                 if new_length > max_range:
#                     continue
                

#                 #values can be tuned to optimse length over cost or visa versa
#                 alpha = 1.0  # weight for length, higher value prioritizes shortest distance
#                 beta = 1.0   # weight for risk, higher value prioritizes least risk


#                 new_g = alpha * new_length + beta * new_risk

#                 heuristic_val = heuristic(neighbor, goal)
#                 new_node = Node(neighbor, g=new_g, length=new_length, risk_cost=new_risk, h=heuristic_val, parent=current_node)

#                 existing_node = next((node for node in open_set if node.position == neighbor), None)
#                 if existing_node and existing_node.g <= new_g:
#                     continue

#                 heapq.heappush(open_set, new_node)


#     return None  # No path exists







# #Euclidean Heuristic. Because in this problem set the diagonal distance was asked to be root 2
# def heuristic(node, goal):
#     return math.sqrt((node[0] - goal[0])**2 + (node[1] - goal[1])**2)


# def create_grid(width, depth, blocked_nodes):
#     graph = {}
#     for i in range(width):
#         for j in range(depth):
#             if (i, j) not in blocked_nodes:
#                 neighbors = {}
#                 directions = [
#                     (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),  # straight
#                     (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), # diagonals
#                     (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))
#                 ]
#                 for dx, dy, step_cost in directions:
#                     nx, ny = i + dx, j + dy
#                     if 0 <= nx < width and 0 <= ny < depth and (nx, ny) not in blocked_nodes:
#                         neighbors[(nx, ny)] = step_cost
#                 graph[(i, j)] = neighbors
#     return graph

# # Create the graph with a grid and specified blocked nodes
# width = 8
# depth = 8
# blocked_nodes = {(0, 4),(1,0), (1, 2), (2, 1), (2, 2), (2, 3), (3, 2), (4, 0)}
# graph = create_grid(width, depth, blocked_nodes)

# #risk map simulation
# risk_map = [
#     [0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 1, 1, 0, 0],
#     [0, 1, 2, 2, 1, 0, 0, 0],
#     [0, 0, 2, 2, 1, 0, 0, 0],
#     [0, 0, 0, 1, 1, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0],
#     [0, 0, 0, 0, 0, 0, 0, 0]
# ]
# #
# # Example usage with the created graph
# start_node = (0, 0)
# goal_node = (6, 6)

# max_range = 20  # or whatever value from map_info.maximum_range
# path = astar_search(graph, start_node, goal_node, max_range, risk_map)


# if path:
#     print("Path found:", path)
# else:
#     print("No path found")