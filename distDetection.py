#!/usr/bin/env python3

import cv2
import depthai as dai
import math
import numpy as np
import heapq
from move import *
from utils import *

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs

## 两个单声道摄像头用于计算立体深度，左右
monoLeft = pipeline.create(dai.node.MonoCamera) 
monoRight = pipeline.create(dai.node.MonoCamera)

## 立体深度节点 计算立体相机对（2x MonoCamera/ColorCamera）的视差和/或深度
stereo = pipeline.create(dai.node.StereoDepth)

## 空间位置计算器节点根据 depth，inputDepth .它将对 ROI 中的深度值进行平均，并删除超出范围的深度值
spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

# XLinkOut 节点用于通过 XLink 将数据从设备发送到主机
## 深度信息
xoutDepth = pipeline.create(dai.node.XLinkOut)
## 空间信息
xoutSpatialData = pipeline.create(dai.node.XLinkOut)

# XLinkIn 节点用于通过 XLink 将数据从主机发送到设备。
## 计算的空间信息
xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

## 设置输入输出流名
xoutDepth.setStreamName("depth")
xoutSpatialData.setStreamName("spatialData")
xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

# Properties
## 设置分辨率 400P
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

## 会以高分辨率和密度来计算深度信息
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
## 检查左右单声道摄像头的一致性
stereo.setLeftRightCheck(True)
## 亚像素级别的计算允许更精确地确定物体的边界和深度信息，尤其是在物体边缘模糊或有纹理不清晰的情况下
stereo.setSubpixel(True)
## 它不会等待接收到消息才开始处理数据，而是使用当前可用的数据进行计算和分析
spatialLocationCalculator.inputConfig.setWaitForMessage(False)

# Create 10 ROIs
for i in range(10):
    config = dai.SpatialLocationCalculatorConfigData()
    ## 深度阈值 200~10000被考虑在其中
    config.depthThresholds.lowerThreshold = 200
    config.depthThresholds.upperThreshold = 10000

    ## 设置ROI框,高度在45%-55%,10个框（0.1)
    config.roi = dai.Rect(dai.Point2f(i*0.1, 0.45), dai.Point2f((i+1)*0.1, 0.55))
    # config.roi = dai.Rect(dai.Point2f(i*0.1, j*0.1), dai.Point2f((i+1)*0.1, (j+1)*0.1))
        
    ## 将该ROI框的config设置添加到配置列表中
    spatialLocationCalculator.initialConfig.addROI(config)

# Linking
## 连接左右单目相机输出到立体深度模块
## 将左右单目相机的输出连接到立体深度模块，用于生成立体深度图像
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

## 空间位置计算器模块的深度数据传递输出
## 将立体深度模块的深度输出传递给空间位置计算器模块，用于空间位置计算器的深度数据分析和处理
spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
stereo.depth.link(spatialLocationCalculator.inputDepth)

## 将空间位置计算器模块的输出连接到空间数据输出模块，用于将处理后的空间位置数据输出
spatialLocationCalculator.out.link(xoutSpatialData.input)

xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    ## 设置设备的红外感知的亮度
    device.setIrLaserDotProjectorBrightness(1000)

    # Output queue will be used to get the depth frames from the outputs defined above
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)

    count_warn = 0
    is_warn_flag = True
    move_forward()
    action = ['w', 'l', 'w','r','w']
    now = 0
    # while True:
    while(now < len(action)):
        inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived

        depthFrame = inDepth.getFrame() # depthFrame values are in millimeters

        spatialData = spatialCalcQueue.get().getSpatialLocations()

        # 每一轮的初始最小距离为15m
        depth_arr = np.array([])
        for depthData in spatialData:
            
            coords = depthData.spatialCoordinates
            distance = (math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2))/1000
            
            depth_arr = np.append(depth_arr, distance)
        
        # 去除数组中的0错误值
        depth_arr = depth_arr[depth_arr != 0]
        depth_arr = np.sort(depth_arr)
        
        min_dis = calculate_reasonable_small_value(depth_arr)
        
        print("当前最小距离为%.2f"%(min_dis))

        if(action[now] == 'w'): # 前进
            if(min_dis > DISTANCELIMIT):
                move_forward()
                count_warn = 0
            else:
                if(is_warn_flag):
                    suspend()
                    now += 1
                    is_warn_flag = False
                    count_warn = 0
                else:
                    count_warn += 1
                    if(count_warn >= 5):
                        is_warn_flag = True
        elif(action[now] == 'l' or action[now] == 'r'):
            if(min_dis <= DISTANCELIMIT):
                if(action[now] == 'l'):
                    left()
                else:
                    right()
                count_warn = 0
            else:
                if(is_warn_flag):
                    suspend()
                    now +=1
                    is_warn_flag = False
                    count_warn = 0
                else:
                    count_warn +=1
                    if(count_warn >= 5):
                        is_warn_flag = True
