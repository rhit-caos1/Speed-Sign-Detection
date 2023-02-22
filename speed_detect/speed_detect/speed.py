import cv2
import numpy as np
# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class camera_read():
    def __init__(self,upper_corner, lower_corner):
    # Create a pipeline
        self.Vresult = 0
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)


        # img = cv2.imread("input.jpg")

        self.segments = dict({
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

        #setupslidebar
        self.minH=166
        self.maxH=0
        self.minS=152
        self.maxS=155
        self.minV=232
        self.maxV=214

        # upper_conner = (215,215)
        # lower_conner = (405,295)
        self.up1 = upper_corner
        self.up2 = (lower_corner[0],upper_corner[1])
        self.lp1 = lower_corner
        self.lp2 = (upper_corner[0],lower_corner[1])
        length = lower_corner[0]-upper_corner[0]
        self.width = lower_corner[1]-upper_corner[1]

        self.secw = int(length*0.275)
        self.sec1 = int(self.up1[0]+self.secw)
        self.sec2 = int(self.sec1+length*0.1)
        self.sec3 = int(self.lp1[0]-self.secw)

        # sec_lmid = int(self.secw/2)
        # sec_wmid = int(self.width/2)

        self.kernel = np.ones((11,11),np.uint8)


    def my_line(self, img, start, end):


        thickness = 1
        line_type = 8
        cv2.line(img,
                start,
                end,
                (255, 255, 255),
                thickness,
                line_type)

    

    def my_line_b(self, img, start, end):


        thickness = 1
        line_type = 8
        cv2.line(img,
                start,
                end,
                (0, 0, 0),
                thickness,
                line_type)

    def detect_num(self,img,secw,width,seg_dic):
        segs = [0,0,0,0,0,0,0]

        sec1_1 = (secw/2,int(width*0.08))
        sec1_2 = (secw/2,int(width/2))
        sec1_3 = (secw/2,int(width*0.94))

        sec1_4 = (int(secw*0.20),int(width/4))
        sec1_5 = (int(secw*0.90),int(width/4))
        sec1_6 = (int(secw*0.15),int(width*0.75))
        sec1_7 = (int(secw*0.85),int(width*0.75))

        seg1_1 = img[int(sec1_1[1]-6):int(sec1_1[1]+6), int(sec1_1[0]-6):int(sec1_1[0]+6)]
        if(seg1_1.mean() >= 100):
            segs[0] = 1
        seg1_2 = img[int(sec1_2[1]-6):int(sec1_2[1]+6), int(sec1_2[0]-6):int(sec1_2[0]+6)]
        if(seg1_2.mean() >= 100):
            segs[1] = 1
        seg1_3 = img[int(sec1_3[1]-6):int(sec1_3[1]+6), int(sec1_3[0]-6):int(sec1_3[0]+6)]
        if(seg1_3.mean() >= 100):
            segs[2] = 1
        seg1_4 = img[int(sec1_4[1]-6):int(sec1_4[1]+6), int(sec1_4[0]-6):int(sec1_4[0]+6)]
        if(seg1_4.mean() >= 100):
            segs[3] = 1
        seg1_5 = img[int(sec1_5[1]-6):int(sec1_5[1]+6), int(sec1_5[0]-6):int(sec1_5[0]+6)]
        if(seg1_5.mean() >= 100):
            segs[4] = 1
        seg1_6 = img[int(sec1_6[1]-6):int(sec1_6[1]+6), int(sec1_6[0]-6):int(sec1_6[0]+6)]
        if(seg1_6.mean() >= 100):
            segs[5] = 1
        seg1_7 = img[int(sec1_7[1]-5):int(sec1_7[1]+5), int(sec1_7[0]-5):int(sec1_7[0]+5)]
        if(seg1_7.mean() >= 100):
            segs[6] = 1

        #show image section for varify
        # cv2.imshow('seg1_1',seg1_1)
        # cv2.imshow('seg1_2',seg1_2)
        # cv2.imshow('seg1_3',seg1_3)
        # cv2.imshow('seg1_4',seg1_4)
        # cv2.imshow('seg1_5',seg1_5)
        # cv2.imshow('seg1_6',seg1_6)
        # cv2.imshow('seg1_7',seg1_7)

        # print(segs)

        if tuple(segs) in seg_dic.keys():
            return seg_dic[tuple(segs)]
        else:
            return None

    def decimal(self,img,secw,width):
        seg = img[int(width-13):int(width-3), int(secw-10):int(secw)]
        # cv2.imshow('seg1_7',seg)
        if(seg.mean() >= 100):
            i = 1
        else:
            i = 10

        return i




    def detection(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image =cv2.rotate(np.asanyarray(color_frame.get_data()),cv2.ROTATE_180)

        img_hsv=cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # minH=10
        # maxH=170
        # minS=105
        # maxS=41
        # minV=98
        # maxV=236

        lower_red = np.array([0,self.minS,self.minV])
        upper_red = np.array([self.minH,255,255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        # upper mask (170-180)
        lower_red = np.array([self.maxH,self.maxS,self.maxV])
        upper_red = np.array([180,255,255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

        # join my masks
        mask_1 = mask0+mask1

        mask = cv2.morphologyEx(mask_1, cv2.MORPH_CLOSE, self.kernel)


        # # set my output img to zero everywhere except my mask
        # output_img = img.copy()
        # output_img[np.where(mask==0)] = 0


        self.my_line(mask,self.up1,self.up2)
        self.my_line(mask,self.up2,self.lp1)
        self.my_line(mask,self.lp1,self.lp2)
        self.my_line(mask,self.lp2,self.up1)

        self.my_line(color_image,self.up1,self.up2)
        self.my_line(color_image,self.up2,self.lp1)
        self.my_line(color_image,self.lp1,self.lp2)
        self.my_line(color_image,self.lp2,self.up1)

        self.my_line(mask,(self.sec1,self.up1[1]),(self.sec1,self.lp1[1]))
        self.my_line(mask,(self.sec2,self.up1[1]),(self.sec2,self.lp1[1]))
        self.my_line(mask,(self.sec2+self.secw,self.up1[1]),(self.sec2+self.secw,self.lp1[1]))
        self.my_line(mask,(self.sec3,self.up1[1]),(self.sec3,self.lp1[1]))

        seg1 = mask[self.up1[1]:self.lp1[1], self.up1[0]:self.sec1]
        seg2 = mask[self.up1[1]:self.lp1[1], self.sec2:self.sec2+self.secw]
        seg3 = mask[self.up1[1]:self.lp1[1], self.sec3:self.lp1[0]]

        result0 = self.detect_num(seg1,self.secw,self.width,self.segments)
        result1 = self.detect_num(seg2,self.secw,self.width,self.segments)
        result2 = self.detect_num(seg3,self.secw,self.width,self.segments)
        d = self.decimal(seg2,self.secw,self.width)
        if ((result1 is not None) and (result2 is not None)):
            if(result0 is not None):
                result = (result0*100+result1*10+result2)*d
                print(result)
                self.Vresult = result
            else:
                result = (result1*10+result2)*d
                print(result)
                self.Vresult = result

        # cv2.imshow('seg1',seg1)
        # cv2.imshow('seg2',seg2)
        # cv2.imshow('seg3',seg3)
        cv2.imshow('img', color_image)
        # cv2.imshow('mask', mask)
        key=cv2.waitKey(1)
        

        # if key & 0xFF == ord('q') or key == 27:
        #     print(seg1.shape)
        #     cv2.destroyAllWindows()
        
        return self.Vresult

class Speed(Node):
    def __init__(self):
        super().__init__('speed')
        self.publisher_ = self.create_publisher(Int16, 'speed', 10)


def main():
    upper_corner = (215,215)
    lower_corner = (405,295)
    detect = camera_read(upper_corner,lower_corner)
    msg = Int16()
    rclpy.init(args=None)    
    speed_pub = Speed()

    while True:
        msg.data = detect.detection()
        speed_pub.publisher_.publish(msg)
        # speed_pub.get_logger().info(f'detect speed {msg.data}')
    speed_pub.pipeline.stop()
    detect_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()