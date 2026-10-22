import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from routes_algorithm_package import *
from action_sequence_package import actionSegment,action_to_move
from constants import speed,text_prompt,position
from extract_network import *
from drawing_routes_package import plot_path_with_nodes

def segment_to_node(image_path:str,text_prompt:str,grid_size:int,width:int,height:int,message:str,flag:bool=True):
    '''
        切割节点
    '''
    print("segment_to_node")
    if flag:
        image,node  = get_node(image_path,text_prompt,width,height)
    else:
        image,node_list = get_node_list(image_path, text_prompt, width, height)
    image_to_gird = image.astype(np.uint8)
    image_grid = grid_map(image_to_gird,grid_size)
    save_image(image_grid,message)
    if flag:
        return image_grid,node
    else:
        return image_grid,node_list

def starter(image_path:str,grid_size:int=32):  #启动函数
    
    #用于 构图的三个参数  car_node obstacle_node_list goal_node
    image_pil = Image.open(image_path).convert("RGB")
    width, height = image_pil.size   
    cell_height = height // grid_size
    cell_width = width // grid_size
    print(image_pil)
    print(f"原来的图片宽度{width},高度{height}")
    print(f"切割后的图片宽度{cell_width},{cell_height}")
    car_image_grid,car_node= segment_to_node(image_path,text_prompt[0],grid_size,cell_width,cell_width,position[0])
    car_node = find_center_contours(car_image_grid)[0]
    print(f"car_node:{car_node}")
    print(f"start:{car_node.x},{car_node.y},{car_node.width},{car_node.height}")


    obstacle_map_grid,obs_node_list= segment_to_node(image_path,text_prompt[1],grid_size,cell_width,cell_width,position[1],False)
    print(f"obs_node_list length:{len(obs_node_list)}")
    obs_node_list = find_center_contours(obstacle_map_grid)
    print(f"obs_node_list length:{len(obs_node_list)}")
    for idx, node in enumerate(obs_node_list):
        print(f"obstacle {idx} 节点:{node.x},{node.y},{node.width},{node.height}")
    
    map_ondraw = [[0] * grid_size for _ in range(grid_size)]  # create a n x m grid, where 0 means clear space
    for obstacle in obs_node_list:
        for i in range(obstacle.x, obstacle.x + obstacle.width):
            for j in range(obstacle.y, obstacle.y + obstacle.height):
                if i >= len(map_ondraw) or j >= len(map_ondraw[i]):
                    print(f"Index out of range: i={i}, j={j}")
                else:
                    map_ondraw[i][j] = 1

    target_image_grid,target_node= segment_to_node(image_path,text_prompt[2],grid_size,cell_width,cell_height,position[2])

    target_node = find_center_contours(target_image_grid)[0]
    target_node.width = car_node.width
    target_node.height = car_node.height

    # 防止出错
    if(target_node.x + target_node.width > grid_size):
        target_node.x = grid_size - target_node.width
    if(target_node.y + target_node.height > grid_size):
        target_node.y = grid_size - target_node.height


    path = a_star_search(map_ondraw, car_node, target_node, car_node.width, car_node.height)
    
    print(f"final:{target_node.x},{target_node.y},{target_node.width},{target_node.height}")

    accumulated_mask_grid = None #3副图都是一样大
    accumulated_mask_grid = np.zeros(obstacle_map_grid.shape, dtype=bool)
    accumulated_mask_grid |= obstacle_map_grid
    accumulated_mask_grid |= car_image_grid
    accumulated_mask_grid |= target_image_grid
     # print_mask(accumulated_mask_grid,"Final Map")
    save_image(accumulated_mask_grid,"final_map")

    if path:
        print("Path found:", path)
        for p in path:
            accumulated_mask_grid[p[1]][p[0]] = 1
        save_image(accumulated_mask_grid,"path_on_map")
        plot_path_with_nodes(path,car_node,target_node,obs_node_list,"path_with_nodes.png")
        # print_mask(accumulated_mask_grid,"Path on Map")

        # 动作分解
        #以300码 的速度 行动
        action_list = actionSegment(path, car_node, speed.get('300码'))
        for action in action_list:
            print(f"car action:{action.character},car action's time:{action.time}")
        # 控制小车行走
        print("move on")
        action_to_move(action_list)

    else:
        print("No path found")

if __name__ == "__main__":
    image_path = './code/6.20 测试代码/obstacles10.jpg'   #记得修改图片地址
    starter(image_path)