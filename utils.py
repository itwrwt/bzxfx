import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

def calculate_reasonable_small_value(data):
    
    # 计算Q1（第一个四分位数）和Q3（第三个四分位数）
    Q1 = np.percentile(data, 25)
    Q3 = np.percentile(data, 75)
    
    # 计算四分位距（IQR）
    IQR = Q3 - Q1
    
    # 计算下界限（Lower Bound）
    lower_bound = Q1 - 1.5 * IQR
    
    # 识别非异常值
    non_outliers = data[data >= lower_bound]
    
    # 合理较小值可以定义为非异常值中的最小值
    reasonable_small_value = non_outliers.min()
    
    return reasonable_small_value
