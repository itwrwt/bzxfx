'''

'''


# 第五版 
'''
为了进一步优化算法，使其在尽量不与障碍物接触的前提下路径最短，需要在权衡路径距离和避障惩罚之间找到一个平衡。通过以下方法优化：
    启发式成本动态调整：将惩罚因子动态调整，使其在接近障碍物时增加，但不至于导致路径极端绕远。
    优化路径选择逻辑：在选择路径时，更加智能地评估路径的总成本，结合距离和避障成本。

    修改参数 说明  线性版本
    penalty 为 惩罚因子  越高 就越容易走远路 避开障碍物
                        越低 就越容易走进路 挨着障碍物走

     penalty一般一般 20以下 低 40-50高
'''


import numpy as np
import heapq
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y, width, height, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.g = g  # cost from start node to current node
        self.h = h  # heuristic cost from current node to goal node
        self.parent = parent
    
    def f(self):
        return self.g + self.h
    
    def __lt__(self, other):
        return self.f() < other.f()

def heuristic_cost(node: Node, goal: Node, map_grid) -> float:
    manhattan_distance = abs(node.x + node.width // 2 - goal.x - goal.width // 2) + abs(node.y + node.height // 2 - goal.y - goal.height // 2)
    penalty = 0

    # Check proximity to obstacles
    for i in range(max(0, node.x - 1), min(node.x + node.width + 1, len(map_grid))):
        for j in range(max(0, node.y - 1), min(node.y + node.height + 1, len(map_grid[0]))):
            if map_grid[i][j] != 0:  # assuming 0 means clear space
                penalty += 20  # moderate penalty for being close to obstacles

    return manhattan_distance + penalty

def is_valid(x, y, map_grid, car_width, car_height):
    n = len(map_grid)
    m = len(map_grid[0])
    
    if x < 0 or x + car_width > n or y < 0 or y + car_height > m:
        return False
    
    # Check if the area is clear for the car to fit
    for i in range(x, x + car_width):
        for j in range(y, y + car_height):
            if map_grid[i][j] != 0:  # assuming 0 means clear space
                return False
    return True

def overlaps_obstacle(x, y, width, height, map_grid) -> bool:
    for i in range(x, x + width):
        for j in range(y, y + height):
            if map_grid[i][j] != 0:  # assuming 0 means clear space
                return True
    return False

def get_neighbors(node: Node, map_grid, car_width, car_height) -> list:
    neighbors = []
    x, y = node.x, node.y
    possible_moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # possible movements: right, left, down, up
    
    for dx, dy in possible_moves:
        nx, ny = x + dx, y + dy
        if is_valid(nx, ny, map_grid, car_width, car_height):
            neighbors.append(Node(nx, ny, car_width, car_height))
    
    return neighbors

def a_star_search(map_grid, start: Node, goal: Node, car_width: int, car_height: int):
    open_list = []
    closed_list = set()
    
    heapq.heappush(open_list, start)
    
    while open_list:
        current = heapq.heappop(open_list)
        
        if (current.x, current.y, current.width, current.height) == (goal.x, goal.y, goal.width, goal.height):
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]
        
        closed_list.add((current.x, current.y, current.width, current.height))
        
        for neighbor in get_neighbors(current, map_grid, car_width, car_height):
            if (neighbor.x, neighbor.y, neighbor.width, neighbor.height) in closed_list:
                continue
            
            tentative_g = current.g + 1  # assuming uniform cost
            if tentative_g < neighbor.g or (neighbor.x, neighbor.y, neighbor.width, neighbor.height) not in [(n.x, n.y, n.width, n.height) for n in open_list]:
                neighbor.g = tentative_g
                neighbor.h = heuristic_cost(neighbor, goal, map_grid)
                neighbor.parent = current
                
                heapq.heappush(open_list, neighbor)
    
    return None  # no path found

def plot_path_with_nodes(path: list, start: Node, goal: Node, obstacles: list, save_path: str = None):
    x_path = [point[0] for point in path]
    y_path = [point[1] for point in path]
    
    # Extract start node coordinates
    start_x = start.x
    start_y = start.y
    
    # Extract goal node coordinates
    goal_x = goal.x
    goal_y = goal.y
    
    # Extract obstacle coordinates
    plt.figure(figsize=(10, 10))

    for obstacle in obstacles:
        obstacle_x = obstacle.x
        obstacle_y = obstacle.y
        obstacle_width = obstacle.width
        obstacle_height = obstacle.height
        rect = plt.Rectangle((obstacle_x, obstacle_y), obstacle_width, obstacle_height, facecolor='gray')
        plt.gca().add_patch(rect)
    
    # Plot path
    plt.plot(x_path, y_path, marker='o', linestyle='-', color='b', label='Path')
    
    # Plot start node
    rect_start = plt.Rectangle((start_x, start_y), start.width, start.height, facecolor='green', label='Start')
    plt.gca().add_patch(rect_start)
    
    # Plot goal node
    rect_goal = plt.Rectangle((goal_x, goal_y), goal.width, goal.height, facecolor='red', label='Goal')
    plt.gca().add_patch(rect_goal)
    
    plt.title('Path with Nodes and Obstacles')
    plt.xlabel('X (units)')
    plt.ylabel('Y (units)')
    plt.legend()
    plt.grid(True)
    plt.gca().invert_yaxis()  # invert y-axis to match typical grid orientation
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()

def draw_test():
    start = Node(0, 0, 2, 2)  # start position of the car with width 1 and height 1
    goal = Node(7, 7, 2, 2)   # goal position of the car with width 1 and height 1

    # Adding obstacles
    obstacles = [
        Node(2, 2, 1, 1),  # obstacle with width 1 and height 1 at position (2, 2)
        Node(5, 5, 2, 1),  # obstacle with width 2 and height 1 at position (5, 5)
        Node(6, 2, 2, 2),
        Node(2, 6, 1, 1),
        Node(8, 2, 1, 1),
        Node(0, 4, 1, 3),
        Node(4, 4, 1, 3),
        Node(10, 10, 2, 2),
        Node(15, 15, 6, 5),
        Node(10, 5, 3, 3),
        Node(12, 0, 5, 1),
        Node(12, 0, 5, 1),
        Node(14, 5, 2, 2),
        Node(10, 13, 1, 1),
        Node(5, 12, 3, 3),
    ]
    map_ondraw = [[0] * 32 for _ in range(32)]  # create a n x m grid, where 0 means clear space
    for obstacle in obstacles:
        for i in range(obstacle.x, obstacle.x + obstacle.width):
            for j in range(obstacle.y, obstacle.y + obstacle.height):
                if i >= len(map_ondraw) or j >= len(map_ondraw[i]):
                    print(f"Index out of range: i={i}, j={j}")
                else:
                    map_ondraw[i][j] = 1
    path = a_star_search(map_ondraw, start, goal, 2, 2)
    plot_path_with_nodes(path, start, goal, obstacles, save_path='path_with_nodes2.png')

# Call the draw_test function to see the results
draw_test()
