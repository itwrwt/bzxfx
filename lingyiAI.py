import os
import openai
import re
import json
from openai import OpenAI
API_BASE = "https://api.lingyiwanwu.com/v1"
# 填写你的API密钥
API_KEY = os.getenv("LINGYI_API_KEY", "")

def big_model_chat(prompt):
    client = OpenAI(
        api_key=API_KEY,
        base_url=API_BASE
    )
    completion = client.chat.completions.create(
        model="yi-large",
        messages=[{"role": "user", "content": prompt}]
    )
    json_data = extract_json_from_text(completion.choices[0].message.content)
    print(json_data)
    #return completion

def extract_json_from_text(text):
    # 正则表达式匹配 JSON 字符串
    json_pattern = re.compile(r'\{.*?\}', re.DOTALL)
    match = json_pattern.search(text)
    
    if match:
        json_str = match.group()
        # 解析 JSON 字符串为 Python 对象
        try:
            json_data = json.loads(json_str)
            return json_data
        except json.JSONDecodeError as e:
            print(f"JSON解析错误: {e}")
            return None
    else:
        return None

big_model_chat("假如你是台智能小车，已知 你的速度28.2cm/s,你要向前运动50m,再向左移动10m,我希望你给我输出像下面例子的信息{'action': 'w', //'w'表示前进 'q'表示左移 'e'表示右移 's'表示停止,'time': } 你可以有计算推理过程，但是需要最快推理，你最终只能给我一份json数组")