import os
# 业务空间模型调用请参考文档传入workspace信息: https://help.aliyun.com/document_detail/2746874.html    
        
import random
import json
from http import HTTPStatus
import dashscope
from dashscope import Generation

dashscope.api_key = os.getenv("DASHSCOPE_API_KEY", "")
def call_with_messages(prompt):
    messages = [
        {'role': 'user', 'content': prompt}]
    responses = Generation.call(
        #'qwen2-72b-instruct',
        'qwen-max',
        messages=messages,
        seed=random.randint(1, 10000),  # set the random seed, optional, default to 1234 if not set
        result_format='message',  # set the result to be "message"  format.
    )
    str_data = responses.output.choices[0].message.content
    str_data = str_data.strip('`')
    str_data = str_data.strip('json')
    str_data = str_data.strip('\n')
    try:
        json_data = json.loads(str_data)
        print("\nJSON data:")
        print(json_data)
        print(type(json_data))
        if(len(json_data)!=0):
            for i in json_data:
                distance_value = i['distance']
                print(f"Type of distance value '{distance_value}': {type(distance_value)}")
        return json_data
    except json.JSONDecodeError as e:
        print(f"\nError decoding JSON: {e}")
        return "Error decoding JSON"


if __name__ == '__main__':
    prompt = "假如你是台智能小车，已知 你的速度28.2cm/s,你要向前运动20m,再向左移动30m,我希望你给我输出像下面例子的信息{'action': 'w', //'w'表示前进 'q'表示左移 'e'表示右移 's'表示停止,'distance': //表示距离，以厘米为单位} 请不要输出推理过程，直接给出json数组即可"
    call_with_messages(prompt)