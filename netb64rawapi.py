import http.server
import urllib
import json

import warnings
from lang_sam import LangSAM
import os
from io import BytesIO
import imghdr
import requests
import torch
from PIL import Image, ImageDraw
import numpy as np
import base64
import socket
import re



class HTTPServerV6(http.server.HTTPServer):
    address_family = socket.AF_INET6

torch.cuda.set_device(2)
model = LangSAM(sam_type="vit_h", ckpt_path="./pretrained/sam_vit_h_4b8939.pth",device="cuda:2")
############## Test ##############
# image = "./assets/physed-cycle-walk-videoSixteenByNineJumbo1600-v2.jpg"
# image_p = Image.open(image).convert("RGB")
# t_prompt = "person on the bicycle"
# masks, boxes, phrases, logits = model.predict(image_p, t_prompt)
# print(f"[*] test:{boxes}")
##################################
def is_json(jsonstr):
    try:
        json.loads(jsonstr)
    except ValueError:
        return False
    return True
def is_legal_url(path):
    if path in ['/api/sam']:
        return True
    return False
def get_img(img_b64): # 接收整个datas，方便异常处理
    #try:
    #image = BytesIO(requests.get(img_url).content)
    #base64_data = re.sub('^data:image/.+;base64,', '', base64_str)
    #tmp_base = img_b64
    print(f"[*] img len: {len(img_b64)}")
    
    #base64_data = bytes(img_b64,"utf8")  ##### tmp_base 为接收到的base64字符串
    byte_data = base64.b64decode(img_b64)
    image_type = imghdr.what(None,h=byte_data)
    print(image_type)
    image_data = BytesIO(byte_data)
    #img = Image.open(image_data)
    #img.show()

    image = Image.open(image_data).convert("RGB") 
    print(type(image))
    return image,image_type
    #except:
    #    print(f"[-] image convert failed")
    #    return False,False
def plot_boxes(image,boxes):
    #image_pil = Image.open("your_image_path.jpg")  # 替换成你的图片路径
    
    # 定义矩形的坐标
    
    # 创建一个绘图对象
    draw = ImageDraw.Draw(image)
    
    # 绘制矩形
    for boxe in boxes:
        draw.rectangle(boxe, outline="red", width=3)  # 可以设置矩形边框颜色和宽度
    
    # 删除绘图对象
    del draw
    
    # 保存绘制后的图片
    #image_pil.save("drawn_image.jpg")  
    # 转换为 NumPy 数组进行进一步处理：
    #image_array = np.array(image) # Image对象转Numpy

# 若img.save()报错 cannot write mode RGBA as JPEG
# 则img = Image.open(image_path).convert('RGB')
    image_buffer = BytesIO()
    image.save(image_buffer, format='png')
    byte_data = image_buffer.getvalue()
    base64_bytes = base64.b64encode(byte_data)
    base64_str = str(base64_bytes,encoding="utf-8")
    print(f"[*] b64 len:{len(base64_str)}")
    #print(image_array.shape)
    #Image.fromarray(image_array)
    #return image_array.tolist()
    return base64_str

def masks2b64(masks):
    masks_base64_str = []
    for mask in masks:
        image_buffer = BytesIO()
        mask_image = Image.fromarray(mask.numpy())
        mask_image.save(image_buffer, format='png')
        mask_byte_data = image_buffer.getvalue()
        mask_base64_bytes = base64.b64encode(mask_byte_data)
        mask_base64_str = str(mask_base64_bytes,encoding="utf-8")
        masks_base64_str.append(mask_base64_str)
    return masks_base64_str

def model_run(datas):
    msg = "" # 状态消息
    code = 0 # 状态码, 0:正常、-1:传入参数不全、-2:文件不存在、-3:文件类型异常、-4:未检测出任何物体
    data = {}
    if 'img' not in datas.keys() or 'prompt' not in datas.keys(): # 传入参数不全
        msg = "Please pass the 'img' and 'prompt' parameters."
        code = -1
        data = {}
        return {'msg': msg, 'code': code, 'data': data}
    
    image_pil,image_type = get_img(datas['img'])
    if not image_pil: # 文件下载失败
        msg = "file download failed."
        code = -2
        data = {}
    elif not image_type in ['png','jpeg','bmp']: # 文件类型异常（非图片）
        msg = "Only support png,jpeg,bmp."
        code = -3
        data = {}
    else:
        text_prompt = datas['prompt']
        # 开始检测
        print(text_prompt)
        masks, boxes, phrases, logits = model.predict(image_pil, text_prompt)
    
        if len(masks) == 0:
            msg = f"No objects of the '{text_prompt}' prompt detected in the image."
            code = -4
            data = {}
            
        boxes = boxes.tolist()
        logits = logits.tolist()
        images_boxes = plot_boxes(image_pil,boxes) 

        masks_base64_str = masks2b64(masks)
            
        #data = {"boxes":boxes, "phrases":phrases, "confidence":logits, "images":images_boxes}
        data = {"boxes":boxes, "phrases":phrases, "confidence":logits, "masks":masks_base64_str}
        print(f"[*] data: {data['boxes']},{data['phrases']},{data['confidence']}")
    
    return {'msg': msg, 'code': code, 'data': data}


class TestHTTPHandle(http.server.BaseHTTPRequestHandler):  
    def setup(self):
        self.request.settimeout(10) #设定超时时间10秒
        http.server.BaseHTTPRequestHandler.setup(self)
    
    def _set_response(self):
        self.send_response(200) #设定HTTP服务器请求状态200
        self.send_header('Content-type', 'application/json') #设定HTTP服务器请求内容格式
        self.end_headers()
    def _set_response404(self):
        self.send_response(404) #设定HTTP服务器请求状态404
        self.send_header('Content-type', 'application/json') #设定HTTP服务器请求内容格式
        self.end_headers()
    def do_GET(self):  
        self.protocal_version = 'HTTP/1.1'
        self._set_response()
        buf = 'SAM API服务'  
        buf = bytes(buf,encoding="utf-8") #编码转换
        self.wfile.write(buf) #写出返回内容
        
    def do_POST(self):  
        '''
        处理通过POST方式传递过来并接收的输入数据并计算和返回结果
        '''
        path = self.path  #获取请求的URL路径
        print()
        print(path)
        # if not is_legal_url(path): # 请求的URI不存在
        #     self._set_response404()
        #     buf = bytes("The requested URI does not exist. ['/api/sam']",encoding="utf-8")
        #     print(f"[-] The requested URI:{path} does not exist.")
        #     self.wfile.write(buf)  #向客户端写出返回结果
        #     return
        #获取post提交的数据
        datas = self.rfile.read(int(self.headers['content-length']))  
        #datas = urllib.unquote(datas).decode("utf-8", 'ignore')
        if not is_json(datas.decode('utf-8')): # 非JSON字符串
            self._set_response()
            buf = bytes("Please pass the JSON string, the parameters are 'img' and 'prompt'",encoding="utf-8")
            print(f"[-] Not Json str")
            self.wfile.write(buf)  #向客户端写出返回结果
            return
            
        datas = json.loads(datas.decode('utf-8'))
        #print(datas)
        '''
        计算, 结果存储于变量“buf”里
        '''
        buf = model_run(datas)

        buf = json.dumps(buf) # 转为str
        
        self._set_response()
        buf = bytes(buf,encoding="utf-8")
        self.wfile.write(buf)  #向客户端写出返回结果

def start_server(ip, port):  
    if(':' in ip):
        http_server = HTTPServerV6((ip, port), TestHTTPHandle)
    else:
          http_server = http.server.HTTPServer((ip, int(port)), TestHTTPHandle)
    print('[*] HTTP server start')
    try:
        http_server.serve_forever()   
    except KeyboardInterrupt:
        pass
    http_server.server_close()
    print('[*] HTTP server closed')

if __name__ == '__main__':
    start_server('0.0.0.0', 11451) #
    #start_server('::', 20000) # IPv6 
