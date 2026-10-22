'''
    该包 用于存放 常量值的地方

'''

## Distance limit
DISTANCELIMIT = 0.5

car = {
    'width' : 52,
    'height' : 35
}

#指令集合
Instruction_set = {   
    'w' : bytes([0x55,0x55,0x03,0x09,0x80]), #前进
    'x' : bytes([0x55,0x55,0x03,0x0A,0x80]), #后退
    'q' : bytes([0x55,0x55,0x03,0x0B,0x80]), #左移
    'e' : bytes([0x55,0x55,0x03,0x0C,0x80]), #右移
    'a' : bytes([0x55,0x55,0x03,0x0F,0x80]), #加速
    'd' : bytes([0x55,0x55,0x03,0x10,0x80]), #减速
    's' : bytes([0x55,0x55,0x03,0x11,0x80])  #停止
}

# Local network endpoint omitted from public backup.
URL = 'http://120.55.127.129:13016/api/sam'

#速度
speed={
    '300码':28.2,
    '250码':23.6,
    '200码':19.65, 
    '150码':14.63, 
    '100码':9.8,
}

text_prompt = [
    "truck",
    "white bags",
    "red bottle"
]

position = [
    "car_position",
    "obstacles_position",
    "target_position"
]