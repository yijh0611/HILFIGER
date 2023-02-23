# -*- coding: utf-8 -*-
"""
Created on Wed Feb 22 19:12:12 2023

@author: User

@이미지를 전부 불러와서 동영상으로 만드는 코드이다.
"""

import os

import cv2

# 사진 파일 경로
path_dir = 'D:\\icuas\\flight_1'
# 파일 리스트 - 경로는 없고 파일 이름만 있음
file_list = os.listdir(path_dir)
# 코덱
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
# 영상 저장
out = cv2.VideoWriter('D:\\icuas\\flight_1\\SaveVideo.mp4',fourcc,30.0,(640, 480))

for i in file_list:
    name = f'{path_dir}\{i}'
    
    if 'mp4' in name:
        break
    try:
        img = cv2.imread(name, cv2.IMREAD_COLOR)
        # height, width, _ = np.shape(img)
        out.write(img)
        
    except:
        break
    
out.release()