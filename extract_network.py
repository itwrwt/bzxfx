'''
    用于提取网络的 信息的代码
    划分成 n-m 的网格 

    包括 起点 1 , 终点 1 , 障碍物 x  这些信息 

'''
import cv2
from routes_algorithm_package import *
import numpy as np
from segment_api import segment
from PIL import Image

def find_center_contours(binary_image:any)-> list:
    if binary_image.dtype != np.uint8:
        binary_image = binary_image.astype(np.uint8)
    
    # 使用 cv2.findContours 找到图像的轮廓
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center_contours = []
    node_list = []

    # 对于每一个轮廓，使用 cv2.moments 计算矩
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] > 0:
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])
            # 计算中心轮廓
            # if 0.25 * binary_image.shape[1] < cX < 0.75 * binary_image.shape[1] and 0.25 * binary_image.shape[0] < cY < 0.75 * binary_image.shape[0]:
                # 使用 cv2.boundingRect 找出轮廓的起始位置及其宽度和高度
            x, y, w, h = cv2.boundingRect(contour)
            node = Node(x,y,w,h)
            node_list.append(node)
            center_contours.append((contour, (cX, cY), (x, y, w, h)))
            print(f"center (X,Y): ({cX}, {cY})，起始位置和宽高为 ({x}, {y}, {w}, {h})")
    
    return node_list

#画图
def grid_map(image,grid_size:int=16)-> np.ndarray:   

    height, width = image.shape

    # 计算每个网格单元的大小
    cell_height = height // grid_size
    cell_width = width // grid_size

    # 创建新的网格结果
    grid_result = np.zeros((grid_size, grid_size), dtype=bool)

    # 分割张量并检查每个网格单元
    for i in range(grid_size):
        for j in range(grid_size):
            grid_cell = image[i*cell_height:(i+1)*cell_height, j*cell_width:(j+1)*cell_width]
            grid_result[i, j] = np.any(grid_cell)

    return grid_result 


def get_node_list(image_path:str, prompt:str,cell_width:int,cell_height:int):
    node_list = []
    masks, boxes, phrases, logits = segment(image_path, prompt)
    for box in boxes:
        x = round(box[0] // cell_width)
        y = round(box[1] // cell_height)
        w = round((box[2]-box[0]) // cell_width)
        h = round((box[3]-box[1]) // cell_height)

        node = Node(x,y,w,h)
        node_list.append(node)
    
    # 障碍物图合成
    accumulated_mask = None
    if accumulated_mask is None:
        accumulated_mask = np.zeros(masks[0].shape, dtype=bool)
    for mask in masks:
        accumulated_mask |= mask

    return accumulated_mask,node_list

def get_node(image_path:str, prompt:str,cell_width:int,cell_height:int):
    mask_image = None
    masks, boxes, phrases, logits = segment(image_path, prompt)
    
    x = round(boxes[0][0] // cell_width)
    y = round(boxes[0][1] // cell_height)
    w = round((boxes[0][2]-boxes[0][0]) // cell_width)
    h = round((boxes[0][3]-boxes[0][1]) // cell_height)

    node = Node(x,y,w,h)

    return masks[0], node

def save_image(np_array:np.ndarray,filename:str):
    # 将 NumPy 数组转换为 PIL 图像对象
    image = Image.fromarray(np_array)

    # 保存图像到本地文件
    image.save(f'./{filename}.png')