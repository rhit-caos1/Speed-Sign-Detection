import cv2
import numpy as np

import modern_robotics as mr
# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import math


import argparse

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)


img = cv2.imread("input.jpg")

segments = dict({
    tuple([1, 0, 1, 1, 1, 1, 1]):0, # 0
    tuple([0, 0, 0, 0, 1, 0, 1]):1, # 1
    tuple([1, 1, 1, 0, 1, 1, 0]):2, # 2
    tuple([1, 1, 1, 0, 1, 0, 1]):3, # 3
    tuple([0, 1, 0, 1, 1, 0, 1]):4, # 4
    tuple([1, 1, 1, 1, 0, 0, 1]):5, # 5
    tuple([1, 1, 1, 1, 0, 1, 1]):6, # 6
    tuple([1, 0, 0, 0, 1, 0, 1]):7, # 7
    tuple([1, 1, 1, 1, 1, 1, 1]):8, # 8
    tuple([1, 1, 1, 1, 1, 0, 1]):9  # 9
})

img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# lower mask (0-10)
lower_red = np.array([0,120,120])
upper_red = np.array([10,255,255])
mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

# upper mask (170-180)
lower_red = np.array([170,120,120])
upper_red = np.array([180,255,255])
mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

# join my masks
mask = mask0+mask1

#setupslidebar
minH=10
maxH=170
minS=105
maxS=41
minV=98
maxV=236
alpha_slider_max = 359
def nothing(x):
    pass
cv2.namedWindow('controls')
cv2.createTrackbar('max','controls',170,alpha_slider_max,nothing)
cv2.createTrackbar('min','controls',10,alpha_slider_max,nothing)
cv2.createTrackbar('maxs','controls',41,alpha_slider_max,nothing)
cv2.createTrackbar('mins','controls',105,alpha_slider_max,nothing)
cv2.createTrackbar('maxv','controls',236,alpha_slider_max,nothing)
cv2.createTrackbar('minv','controls',98,alpha_slider_max,nothing)

def my_line(img, start, end):


    thickness = 1
    line_type = 8
    cv2.line(img,
             start,
             end,
             (255, 255, 255),
             thickness,
             line_type)

upper_conner = (20,150)
lower_conner = (210,240)
up1 = upper_conner
up2 = (lower_conner[0],upper_conner[1])
lp1 = lower_conner
lp2 = (upper_conner[0],lower_conner[1])
length = lower_conner[0]-upper_conner[0]
width = lower_conner[1]-upper_conner[1]

secw = int(length*0.275)
sec1 = int(up1[0]+secw)
sec2 = int(sec1+length*0.1)
sec3 = int(lp1[0]-secw)

sec_lmid = int(secw/2)
sec_wmid = int(width/2)

def my_line_b(img, start, end):


    thickness = 1
    line_type = 8
    cv2.line(img,
             start,
             end,
             (0, 0, 0),
             thickness,
             line_type)

def detect_num(img,secw,width,seg_dic):
    segs = [0,0,0,0,0,0,0]

    sec1_1 = (secw/2,int(width*0.08))
    sec1_2 = (secw/2,int(width/2))
    sec1_3 = (secw/2,int(width*0.94))

    sec1_4 = (int(secw*0.20),int(width/4))
    sec1_5 = (int(secw*0.90),int(width/4))
    sec1_6 = (int(secw*0.15),int(width*0.75))
    sec1_7 = (int(secw*0.85),int(width*0.75))

    seg1_1 = img[int(sec1_1[1]-5):int(sec1_1[1]+5), int(sec1_1[0]-5):int(sec1_1[0]+5)]
    if(seg1_1.mean() >= 100):
        segs[0] = 1
    seg1_2 = img[int(sec1_2[1]-5):int(sec1_2[1]+5), int(sec1_2[0]-5):int(sec1_2[0]+5)]
    if(seg1_2.mean() >= 100):
        segs[1] = 1
    seg1_3 = img[int(sec1_3[1]-5):int(sec1_3[1]+5), int(sec1_3[0]-5):int(sec1_3[0]+5)]
    if(seg1_3.mean() >= 100):
        segs[2] = 1
    seg1_4 = img[int(sec1_4[1]-5):int(sec1_4[1]+5), int(sec1_4[0]-5):int(sec1_4[0]+5)]
    if(seg1_4.mean() >= 100):
        segs[3] = 1
    seg1_5 = img[int(sec1_5[1]-5):int(sec1_5[1]+5), int(sec1_5[0]-5):int(sec1_5[0]+5)]
    if(seg1_5.mean() >= 100):
        segs[4] = 1
    seg1_6 = img[int(sec1_6[1]-5):int(sec1_6[1]+5), int(sec1_6[0]-5):int(sec1_6[0]+5)]
    if(seg1_6.mean() >= 100):
        segs[5] = 1
    seg1_7 = img[int(sec1_7[1]-5):int(sec1_7[1]+5), int(sec1_7[0]-5):int(sec1_7[0]+5)]
    if(seg1_7.mean() >= 100):
        segs[6] = 1

    #show image section for varify
    cv2.imshow('seg1_1',seg1_1)
    cv2.imshow('seg1_2',seg1_2)
    cv2.imshow('seg1_3',seg1_3)
    cv2.imshow('seg1_4',seg1_4)
    cv2.imshow('seg1_5',seg1_5)
    cv2.imshow('seg1_6',seg1_6)
    cv2.imshow('seg1_7',seg1_7)

    print(segs)

    if tuple(segs) in seg_dic.keys():
        return seg_dic[tuple(segs)]
    else:
        return None


while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image =cv2.rotate(np.asanyarray(color_frame.get_data()),cv2.ROTATE_180)

    img_hsv=cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # minH=10
    # maxH=170
    # minS=105
    # maxS=41
    # minV=98
    # maxV=236

    minH = int(cv2.getTrackbarPos('min','controls'))
    maxH = int(cv2.getTrackbarPos('max','controls'))
    minS = int(cv2.getTrackbarPos('mins','controls'))
    maxS = int(cv2.getTrackbarPos('maxs','controls'))
    minV = int(cv2.getTrackbarPos('minv','controls'))
    maxV = int(cv2.getTrackbarPos('maxv','controls'))

    lower_red = np.array([0,minS,minV])
    upper_red = np.array([minH,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red = np.array([maxH,maxS,maxV])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

    # join my masks
    mask = mask0+mask1

    # # set my output img to zero everywhere except my mask
    # output_img = img.copy()
    # output_img[np.where(mask==0)] = 0


    my_line(mask,up1,up2)
    my_line(mask,up2,lp1)
    my_line(mask,lp1,lp2)
    my_line(mask,lp2,up1)

    my_line(mask,(sec1,up1[1]),(sec1,lp1[1]))
    my_line(mask,(sec2,up1[1]),(sec2,lp1[1]))
    my_line(mask,(sec2+secw,up1[1]),(sec2+secw,lp1[1]))
    my_line(mask,(sec3,up1[1]),(sec3,lp1[1]))

    seg1 = mask[up1[1]:lp1[1], up1[0]:sec1]
    seg2 = mask[up1[1]:lp1[1], sec2:sec2+secw]
    seg3 = mask[up1[1]:lp1[1], sec3:lp1[0]]

    # result = detect_num(seg1,secw,width,segments)
    # print(result)
    # result = detect_num(seg2,secw,width,segments)
    # print(result)
    result = detect_num(seg3,secw,width,segments)
    print(result)

    cv2.imshow('seg1',seg1)
    cv2.imshow('seg2',seg2)
    cv2.imshow('seg3',seg3)
    cv2.imshow('img', color_image)
    cv2.imshow('mask', mask)
    key=cv2.waitKey(1)
    

    if key & 0xFF == ord('q') or key == 27:
        print(length)
        print(seg1.shape)
        cv2.destroyAllWindows()
        break