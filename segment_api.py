import requests
import base64
import json
import numpy as np
from PIL import Image, ImageDraw
from io import BytesIO
import matplotlib.pyplot as plt
import os
from constants import URL

def segment(img_path:str, prompt:str):

    with open(img_path,'rb') as f:
        img_base64 = base64.b64encode(f.read())
        base64_str = str(img_base64,encoding="utf-8")
    
    # message package
    d = {'img': base64_str, 'prompt': prompt}
    headers = {'Content-Type': 'application/json'}

    r = requests.post(url=URL, headers=headers, data=json.dumps(d))
    
    res = json.loads(r.text)
    
    res_msg = res['msg']
    res_code = res['code']
    res_data = res['data']

    if(res_msg != "" or res_code < 0):
        print(f"code:{res_code},msg:{res_msg}")

    boxes = res_data['boxes']
    phrases = res_data['phrases']
    confidence = res_data['confidence']
    masks = res_data['masks']

    deal_masks = []
    for mask in masks:
        mask = base64.b64decode(mask)
        bytes_io = BytesIO(mask)
        mask_image = Image.open(bytes_io)
        
        mask_array = np.asarray(mask_image,dtype=bool)
        
        deal_masks.append(mask_array)

    return deal_masks,boxes,phrases,confidence

def segment_test():
    #print("这是 写代码测试地方")
    print('ss')
    #segment('./code/6.20 测试代码/obstacles10.jpg',"car")