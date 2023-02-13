# import argparse
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
import torch.nn as nn
from numpy import random

from yolo_pkg.models.experimental import attempt_load

# from pathlib import Path
# from yolo_pkg.models.experimental import Ensemble
# from yolo_pkg.models.common import Conv
# from yolo_pkg.utils.google_utils import attempt_download

from yolo_pkg.utils.datasets import LoadStreams, LoadImages
from yolo_pkg.utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from yolo_pkg.utils.plots import plot_one_box
from yolo_pkg.utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray

# def attempt_load(weights, map_location=None):
#     # Loads an ensemble of models weights=[a,b,c] or a single model weights=[a] or weights=a
#     model = Ensemble()
#     for w in weights if isinstance(weights, list) else [weights]:
#         #unable to detect file !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#         file = Path(str('best.pt').strip().replace("'", '').lower())
#         print(file)
#         # attempt_download(w)
#         torch.load('/home/scg1224/Individual_project/ros_ws/src/Speed-Sign-Detection/detection/detection/best.pt')
#         ckpt = torch.load(w, map_location=map_location)  # load
#         model.append(ckpt['ema' if ckpt.get('ema') else 'model'].float().fuse().eval())  # FP32 model
    
#     # Compatibility updates
#     for m in model.modules():
#         if type(m) in [nn.Hardswish, nn.LeakyReLU, nn.ReLU, nn.ReLU6, nn.SiLU]:
#             m.inplace = True  # pytorch 1.7.0 compatibility
#         elif type(m) is nn.Upsample:
#             m.recompute_scale_factor = None  # torch 1.11.0 compatibility
#         elif type(m) is Conv:
#             m._non_persistent_buffers_set = set()  # pytorch 1.6.0 compatibility
    
#     if len(model) == 1:
#         return model[-1]  # return model
#     else:
#         print('Ensemble created with %s\n' % weights)
#         for k in ['names', 'stride']:
#             setattr(model, k, getattr(model[-1], k))
#         return model  # return ensemble

class detect_sign:
    def __init__(self,source, weights, device, img_size, iou_thres, conf_thres):
        self.webcam = source.isnumeric()

        # Directories
        # save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))  # increment run
        # (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        # Initialize
        #device 0 for GPU
        self.iou_thres = iou_thres
        self.conf_thres = conf_thres
        set_logging()
        self.device = select_device(device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(weights, map_location=self.device)  # load FP32 model
        stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(img_size, s=stride)  # check img_size


        if self.half:
            self.model.half()  # to FP16

        if self.webcam:
            view_img = check_imshow()
            cudnn.benchmark = True  # set True to speed up constant image size inference
            self.dataset = LoadStreams(source, img_size=imgsz, stride=stride)

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(self.model.parameters())))  # run once
        old_img_w = old_img_h = imgsz
        old_img_b = 1

        t0 = time.perf_counter()
        self.obj = iter(self.dataset)

    def detection(self):
        detections = []
        path, img, im0s, vid_cap = next(self.obj)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if self.device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                self.model(img)[0]

        # Inference
        t1 = time_synchronized()
        with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
            pred = self.model(img)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)
        t3 = time_synchronized()

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if self.webcam:  # batch_size >= 1
                p, s, im0, frame = path[i], '%g: ' % i, im0s[i].copy(), self.dataset.count
            else:
                p, s, im0, frame = path, '', im0s, getattr(self.dataset, 'frame', 0)

            p = Path(p)  # to Path

            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    detections.append(int(cls))

                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=1)

            # Print time (inference + NMS)
            print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')

            # Stream results
            
            cv2.imshow(str(p), im0)
            cv2.waitKey(1)  # 1 millisecond
            print('goof')
        print(detections)
        return detections



### ROS Node

class detect_ros(Node):
    def __init__(self, detector1):
        super().__init__('detect_ros')
        self.detector = detector1
        self.publisher_ = self.create_publisher(Int16MultiArray, 'signs', 10)
        msg = Int16MultiArray()
        while True:
            msg.data = detector1.detection()
            print('doof')
            self.publisher_.publish(msg)
            print('toof')

# def main(args=None):
#     """ The main() function. """
#     rclpy.init(args=args)
#     mywaypoint = waypoint()
#     rclpy.spin(mywaypoint)
#     rclpy.shutdown()
def main(args=None):
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(device)
    # file = Path('.')
    # ckpt = torch.load("best.pt", map_location=None)
    rclpy.init(args=None)


    with torch.no_grad():
        detect_s = detect_sign("0","best.pt",device,img_size=1920,iou_thres=0.45,conf_thres=0.5)

        detect_pub = detect_ros(detect_s)
        while True:
            rclpy.spin_once(detect_pub)
            # detect_pub.destroy_node()
            # rclpy.shutdown()
            print('oof')

        detect_pub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    # device = 'cuda' if torch.cuda.is_available() else 'cpu'
    # print(device)
    # #check_requirements(exclude=('pycocotools', 'thop'))
    # rclpy.init(args=None)


    # with torch.no_grad():
    #     detect_s = detect_sign("0","best.pt",device,img_size=1920,iou_thres=0.45,conf_thres=0.5)

    #     detect_pub = detect_ros(detect_s)
    #     while True:
    #         rclpy.spin_once(detect_pub)

    #     detect_pub.destroy_node()
    #     rclpy.shutdown()
        # while True:
        #     detects = detect_s.detection()
        # detect("0","best.pt",device,img_size=1920,iou_thres=0.45,conf_thres=0.5)
        # print('goof1')
    # print('goof2')

