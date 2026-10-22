import re
import json

def extract_json_from_text(text):
    # 正则表达式匹配 JSON 字符串
    json_pattern = re.compile(r'\[\{.*?\}\]', re.DOTALL)
    match = json_pattern.search(text)
    
    if match:
        json_str = match.group()
        # 将单引号替换为双引号
        json_str = json_str.replace("'", '"')
        # 解析 JSON 字符串为 Python 对象
        json_data = json.loads(json_str)
        return json_data
    else:
        return None

# 示例文本
text = """
要计算智能小车向前移动20米所需的时间，我们可以使用以下公式：
时间 = 距离 / 速度
首先，我们需要将距离和速度的单位统一。小车的速度是28.2厘米/秒，而我们要移动的距离是20米。我们需要将距离从米转换为厘米，因为速度的单位是厘米/秒。
1米 = 100厘米
20米 = 20 * 100厘米 = 2000厘米
现在我们可以计算时间了：
时间 = 2000厘米 / 28.2厘米/秒 = 709.22秒（四舍五入到小数点后两位）
根据题目要求，我们需要输出一个JSON数组，其中包含动作'w'（表示前进）和计算出的小车前进20米所需的时间。下面是符合要求的JSON数组：
```json
[{'action': 'w', 'time': 709.22}]
"""

json_data = extract_json_from_text(text)
if json_data:
    print(json.dumps(json_data, ensure_ascii=False, indent=2))
else:
    print("没有找到有效的JSON信息")