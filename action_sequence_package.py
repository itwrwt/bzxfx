
'''
    提供一个 动作序列  包
    包内包含 动作节点类，分割函数

'''
from action_package import *
from routes_algorithm_package import Node
from constants import car

class ActionNode: #动作序列类 包含 动作和运行时间
    def __init__(self, character, time):
        self.character = character
        self.time = time
    @property
    def character(self):
        return self._character
    @character.setter
    def character(self, value):
        self._character = value
    @property
    def time(self):
        return self._time
    @time.setter
    def time(self, value):
        self._time = value

def actionSegment(routes:list,node:Node,speed:float):
    '''
        params:
            routes : <type:'list'>
                example:  path=[(7, 11), (7, 12), (8, 12)]
                means: 路径
            node : <type:'dict'>
                example: node= {'x':1,'y':2,'width':2,'height':2}
                means: 小车节点
            v    : <type:'float'>
                example: v = 5.0
                means: 小车现实速度
    '''

    if not node:
        print("车子节点为空")
    elif(speed<0):
        print("小车速度小于0")
    elif not routes:
        print("没有路径,动作序列无法分割")
    else:
        print("存在路径,正在分割中")   #请务必保持图片为横版 车头从左向右朝向
        action_list = []
        flag = routes[0]

        #接下来做 小车实际挪动时间  
        #小车挪动一格的距离大概为：
        cell_width = int (car.get('width')/node.width)
        cell_height = int (car.get('height')/node.height)
        time_to_x = cell_width/speed
        time_to_y = cell_height/speed

        for route in routes:
            # print(f"点为:{str(p)}")
            if(route[0]>flag[0] and route[1]==flag[1]):
                action = ActionNode("w",time_to_x)
                action_list.append(action) #前进
            elif(route[0]<flag[0] and route[1]==flag[1]):
                action = ActionNode("x",time_to_x)
                action_list.append(action) #后退
            elif(route[0]==flag[0] and route[1]>flag[1]):
                action = ActionNode("e",time_to_y)
                action_list.append(action)   #右移
            elif(route[0]==flag[0] and route[1]<flag[1]):
                action = ActionNode("q",time_to_x)
                action_list.append(action) #左移
            elif(route[0]==flag[0] and route[1]==flag[1]):
                action = ActionNode("s",0)
                action_list.append(action) #停止
            else:
                print(f"存在不合适的动作：上一个点坐标{str(flag)}，下一个点坐标{str(route)}")
            flag = route
        #跳出循环后，加入最后一条停车命令:
        action = ActionNode("s",0)
        action_list.append(action) #停止

        ''' 测试输出
                for i in action_list:
            print(f"动作指令为:{i.character} + {i.time}")
        '''

        return action_list


def action_to_move(action_list:list):
    '''
        提供一个动作序列转换成 指令运动函数
    '''
    for action in action_list:
        if(action.character == 'w'):
            move_forward()
            time.sleep(action.time)
        elif(action.character == 'x'):
            backward()
            time.sleep(action.time)
        elif(action.character == 'q'):
            left()
            time.sleep(action.time)
        elif(action.character == 'e'):
            right()
            time.sleep(action.time)
        elif(action.character == 's'):
            suspend()

def list_to_Actionlist(list,speed=28.2)->list:
    print('list_to_actionList')
    if not list:
        return None
    action_list = []
    for item in list:
       action = ActionNode(item.get('action'),item.get('distance')/speed)
       action_list.append(action)
    print(action_list)
    return action_list


def action_sequence_package_test():
    print("提供写测试  的地方")