'''
    提供 一个 路径规划算法 A*
    
    包括一个路径 节点类
        检测合法函数等等

'''
import heapq
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

def heuristic_cost(node:Node, goal:Node):
    return abs(node.x + node.width // 2 - goal.x - goal.width // 2) + abs(node.y + node.height // 2 - goal.y - goal.height // 2)

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

def overlaps_obstacle(x, y, width, height, map_grid)->bool:
    for i in range(x, x + width):
        for j in range(y, y + height):
            if map_grid[i][j] != 0:  # assuming 0 means clear space
                return True
    return False

def get_neighbors(node:Node, map_grid, car_width, car_height)->list:
    neighbors = []
    x, y = node.x, node.y
    possible_moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # possible movements: right, left, down, up
    
    for dx, dy in possible_moves:
        nx, ny = x + dx, y + dy
        if is_valid(nx, ny, map_grid, car_width, car_height):
            if not overlaps_obstacle(nx, ny, car_width, car_height, map_grid):
                neighbors.append(Node(nx, ny, car_width, car_height))
    
    return neighbors

def a_star_search(map_grid, start, goal, car_width, car_height):
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
            
            neighbor.g = current.g + 1  # assuming uniform cost
            neighbor.h = heuristic_cost(neighbor, goal)
            neighbor.parent = current
            
            heapq.heappush(open_list, neighbor)
    
    return None  # no path found

def algorithm_test():
    print("这是 写代码测试的地方")