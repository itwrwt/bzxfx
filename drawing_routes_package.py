'''
    提供一个绘画函数 用于绘制 路径 
        方便肉眼观察 路径的正确性
'''

import matplotlib.pyplot as plt
from routes_algorithm_package import Node
from routes_algorithm_package import *
'''
start = Node(0, 0, 1, 1)  # start position of the car with width 2 and height 1
goal = Node(8, 8, 1, 1)   # goal position of the car with width 1 and height 1

# Adding obstacles
obstacles = [
    (2, 2, 1, 1),  # obstacle with width 1 and height 1 at position (2, 2)
    (5, 5, 1, 1)   # obstacle with width 2 and height 2 at position (5, 5)
]
path = a_star_search(map_grid, start, goal)
path = [(0, 0), (1, 0), (2, 0), (2, 1), (3, 1), (4, 1), (5, 1), (5, 2), (5, 3), (6, 3), (7, 3), (7, 4), (7, 5), (7, 6), (8, 6), (8, 7), (8, 8)]
'''
def plot_path_with_nodes(path:list, start:Node, goal:Node, obstacles:list, save_path:str=None):
    x_path = [point[0] for point in path]
    y_path = [point[1] for point in path]
    
    # Extract start node coordinates
    start_x = start.x
    start_y = start.y
    
    # Extract goal node coordinates
    goal_x = goal.x
    goal_y = goal.y
    
    # Extract obstacle coordinates
    plt.figure(figsize=(8, 8))

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
    start = Node(0, 0, 1, 1)  # start position of the car with width 2 and height 1
    goal = Node(8, 8, 1, 1)   # goal position of the car with width 1 and height 1

    # Adding obstacles
    obstacles = [
        Node(2, 2, 1, 1),  # obstacle with width 1 and height 1 at position (2, 2)
        Node(5, 5, 2, 1),   # obstacle with width 2 and height 2 at position (5, 5)
        Node(6, 2, 1, 1),
        Node(2, 6, 1, 1),
    ]
    map_ondraw = [[0] * 32 for _ in range(32)]  # create a n x m grid, where 0 means clear space
    for obstacle in obstacles:
        for i in range(obstacle.x, obstacle.x + obstacle.width):
            for j in range(obstacle.y, obstacle.y + obstacle.height):
                if i >= len(map_ondraw) or j >= len(map_ondraw[i]):
                    print(f"Index out of range: i={i}, j={j}")
                else:
                    map_ondraw[i][j] = 1
    path = a_star_search(map_ondraw, start, goal, 1,1)
    plot_path_with_nodes(path, start, goal, obstacles, save_path='path_with_nodes2.png')

draw_test()