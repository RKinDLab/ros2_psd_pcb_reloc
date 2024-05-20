"""

Contains variable and function definitions for all functions related to the YOLOv5 bounding box regressor.

Author: Azmyin Md. Kamal
Date : 06/09/2022
Version: 1.0

"""

# Import necessary modules and definitions
from typing import Tuple
import numpy as np
import math
import time
import copy
import cv2
import torch
import torchvision 
# from py_parameters import Parameters # TODO need to come up with one utils file to merge all helpful funcitons
# from py_utils import debug_lock, show_image
from .utils.general import non_max_suppression

wait_key_val = 5

# ------------------------------------------------------------------------------------------
def show_image(img, win_name, win_size, win_move_loc,wait_key):
    # HELPER FUCTION
    # Display this image

    """
    Parameters
    img --> image, numpy array
    win_name --> window name, string
    win_pos --> size of window of window, list in form [width,height]
    win_move_loc --> position of window, list in form [X_pixel_cord, Y_pixel_cord]
    """

    try:
        cv2.imshow(win_name,img)
        cv2.waitKey(wait_key) # Minimum delay needed??
        if win_size is not None:
            cv2.resizeWindow(win_name, win_size[0],win_size[1]) # Resize window 
        if win_move_loc is not None:
            cv2.moveWindow(win_name,win_move_loc[0],win_move_loc[1]) # Move window
    except:
        print(f"Error in displaying image")
# ------------------------------------------------------------------------------------------



# ------------------------------------------------------------------------------------------
class Neural_Network_Engine(object):
    # Core class to run YOLOv5 bounding box regressor
    # ------------------------------------------------------------------------------------------
    def __init__(self, nn_engine, colors, static_object_database):
        # Initialize class and instance variables
        """
        Parameters 
        nn_engine = [network_dim, device, half, stride, 
            pt, model, imgsz, conf_thres, nms_thres, augment, visualize]
        # TODO declare some work variable so that we don't have to use nn_e[0] variables
        """
        self.nn_e = nn_engine
        self.colors = colors
        self.stat_obj_db = static_object_database # List of static object known to the neural network

        # Variables related to semantic extraction
        self.semantic_matrix = []
    # ------------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------------------------
    # PRIMARY METHOD
    def get_semantic_knowledge(self, img0, win_name, debug_view = True)->Tuple[bool, np.ndarray]:
        """
        Return semantic matrix as defined in paper.
        
        Parameters
            Inputs:
            img0 -- Numpy, unaltered copy of RGB image
            debug_view -- Boolean, if True, show a separete cv2.imagewindow showing
                        class label and bounding boxes of detected objects
            
        Outputs
            stat_flg -- Boolean, 
            semantic_matrix -- Python list in list, [1 x M], 2D array flattened into row vector where each element is a row of the 2D matrix
            ixx -- Numpy, altered copy showing bounding boxes of images
        
        Notes
            * Function called by python node
            * each row [[cls_id, x1, y1, x2, y2],.....]
            * M -- Number of objects detected in a particular image
            * self.nn_e = [network_dim, device, half, stride, pt, model, imgsz, conf_thres, nms_thres, augment, visualize]
        
        """
        # One liner master function that executes the neural network and returns class ids and bound box
        # Does not separate between static and dynamic objects
        # Initialize variables
        img = None
        original_img = None
        work_img = None
        det = []
        ixx = []
        stat_flg = False 
        self.semantic_matrix = [] 

        # Copy for drawing assests on
        if (debug_view):
            original_img = copy.deepcopy(img0) 
        work_img = copy.deepcopy(img0)

        try:
            # Prepare image for YOLOv5 network, convert image to a letterbox image
            img = self.prepare_imz_yolov5(img0, self.nn_e[0], self.nn_e[1], self.nn_e[2], self.nn_e[3], self.nn_e[4], self.nn_e[5], self.nn_e[6]) # Shape [batch,ch,w,h]

            # Run YOLOv5 on GPU
            det = self.dnn_detection(img, self.nn_e[5], self.nn_e[7], self.nn_e[8], self.nn_e[9], self.nn_e[10]) 
        except:
            print("ERROR in detecting objects for this image!")
            return stat_flg, self.semantic_matrix
        # print(f"How many elements does the pytorch tensor have?")
        # print(f"No of tensors --{det.nelement()}")
        # Extract bounding box coordinates and draw asset
        if (det is not None):
            # extract_bbox --> [x1,x2,y1,y2]
            #self.semantic_matrix = self.form_semantic_matrix(det, img, work_img) # [[class_id, x1,y1,x2,y2],.....]
            self.semantic_matrix = self.form_semantic_matrix(det, img, work_img) # [[class_id, x1,y1,x2,y2],.....]
            stat_flg = True
            if (debug_view):
                ixx = self.draw_bbox_on_image(work_img, self.semantic_matrix, self.colors) # Human understandable output of neural network
                # Show image here
                # show_image(self.f_cur.ixx, "output", [self.f_cur.ixx.shape[0], self.f_cur.ixx.shape[1]], None, Parameters.wait_key_val)
                #show_image(ixx, "py_node_cv2_image", [ixx.shape[0], ixx.shape[1]], None, Parameters.wait_key_val)
                show_image(ixx, win_name, [ixx.shape[0], ixx.shape[1]], None, wait_key_val)
        else:
            stat_flg = False 
            print(f"No detection from neural network!")
            # Show blank image
            if (debug_view):
                #show_image(original_img, "py_node_cv2_image", [original_img.shape[0], original_img.shape[1]], None, Parameters.wait_key_val)
                show_image(original_img, win_name, [original_img.shape[0], original_img.shape[1]], None, wait_key_val)
        return stat_flg, self.semantic_matrix

    #  -------------------------------------------------------------------------------------------
    
    # ------------------------------------------------------------------------------------------
    # HELPER METHOD
    @staticmethod
    def getList(dict):
    # Extract all keys from a dictionary and convert them into a list
    # Author: GeekforGeeks
        list = []
        for key in dict.keys():
            list.append(key)
        return list
    # ------------------------------------------------------------------------------------------

    # ---------------------------------------------------------------------------------------
    # CORE METHOD
    def prepare_imz_yolov5(self, img0, network_dim, device, half, stride, pt, model, imgsz):
        # Resizes input image in to letterbox format with padding and moves tensor to CUDA
        """
        img0 --> numpy array of RGB image
        network_dim --> edge length of letterbox image
        device --> cuda:0 if 1 gpu present
        half --> boolean, usually True
        stride --> int, usually 32
        pt --> Boolean, usually True
        """
    
        # Work variable
        img = []

        # Convert image to letterbox format
        img = self.letterbox(img0, network_dim, stride = stride, auto = pt)[0]
    
        # Convert BGR to RGB, 
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        #model.warmup(imgsz=(1, 3, *imgsz), half=half) # Warmup   
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.0, normalize
    
        # expand for batch dim, i.e from [ch,w,h] to [batch,ch,w,h]
        if len(img.shape) == 3:
            img = img[None]  

        return img
    # ---------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------------------------
    # CORE METHOD
    @classmethod
    def dnn_detection(cls,img, model, conf_thres, nms_thres, augment, visualize): 
        # Executes YOLOv5 object detection engine
        
        # nn_engine = [network_dim (0), device (1), half (2), stride (3), pt (4), model (5), imgsz (6), conf_thres (7), nms_thres (8), 
        # augment (9), visualize (10)]

        # Initialize working variables
        pred = []
        det = []
        t1,t2,inference_time = 0,0,0
    
        # Object detection
        with torch.no_grad():
            t1 = cls.time_synchronized()
            pred = model(img, augment=augment, visualize=visualize) # Output is a python list
            
            #det = cls.non_max_suppression(pred[0], conf_thres, nms_thres)[0] # In GPU, works with the pred[0] modification
            det = non_max_suppression(pred, conf_thres, nms_thres)[0] # This version works, uses YOLOV5's NMS
            
            t2 = cls.time_synchronized()
            inference_time = (t2 - t1) # Time to make a prediction
            inference_time = round(inference_time * 1000)
            
            # print()
            # print(f"In dnn_detection function")
            # print(f"Inference time --> {inference_time} in ms")
            # print()

            # If valid detection, transfer to CPU
            if det is not None and len(det) > 0:
                det = det.detach().cpu() # Move tensor out of GPU memory
            else:
                det = None
        return det
    # ------------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------------------------
    # CORE METHOD
    @classmethod
    def form_semantic_matrix(cls, det, im, im0):
        # Modified to use new methods from Ultralytics YOLOv5
        """
        
        Parameters
            Inputs:
                det --> Output tensor from pytorch
                im --> letterbox image prepared for cnn network
                im0 --> unaltered copy of the input image
            
            Outputs:
                semantic_matrix --> Python list in list, [Mx5]
        Notes:
            cls_conf: Available and may be used in future
        """
        
        # Initialize variables
        semantic_matrix = []
        
        if (det is not None and len(det) > 0):
            # Rescale values to width and height of the actual image
            det[:, :4] = cls.scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Loop through each detection and print a bounding box
            for x1, y1, x2, y2, cls_conf, cls_id in det:
                # NOTE: cls_conf is really not used
                lss_tempo = [] # Temporary list
                x1 = abs(int(x1))
                x2 = abs(int(x2))
                y1 = abs(int(y1))
                y2 = abs(int(y2))
                cls_id = int(cls_id)
                lss_tempo = [cls_id,x1,y1,x2,y2] # Convert to correct box shape
                semantic_matrix.append(lss_tempo)

        return semantic_matrix
    # ------------------------------------------------------------------------------------------

    # -------------------------------------------------------------------------------------------------------------
    # CORE METHOD
    @classmethod
    def draw_bbox_on_image(cls,work_img, semantic_matrix, colors):
        # Draws bounding box on detected objects preceeding neural network detections  

        """
        Parameters:
            Inputs:
                work_img -- Numpy, Size, Unaltered copy of images
                semantic_matrix -- Python list in list, [Mx5]
            Outputs:
                iox -- Numpy, Size, pointer to modified work_img
        Notes:
            * Each element in semantic matrix -- [[cls_id, x1, y1, x2, y2]]

        """

        # Define work variables
        iox = None
    
        for ext in semantic_matrix:
            # Define variables
            bbox = []
            cls_id = []
            # Variable to hold the bounding box coordinates
            # bbox = [ext[0],ext[1],ext[2],ext[3]] # [x1,y1,x2,y2]
            # cls_id = ext[4] # Variable to hold class label

            cls_id = ext[0] # Variable to hold class label
            bbox = [ext[1],ext[2],ext[3],ext[4]] # [x1,y1,x2,y2]
            
            iox = cls.draw_bbox(work_img, bbox, cls_id, colors[0])

        return iox
    # -------------------------------------------------------------------------------------------------------------
    
    # -------------------------------------------------------------------------------------------------------------
    @staticmethod
    def draw_bbox(img, bbox, cls_id, color):
        # AUX function, draws the bounding box rectangle
        # Define output variable
        ixx_ret = None

        col_text = (0,0,102)

        # The rectangle value must be passed in x1, y1, x2, y2
        x1 = bbox[0]
        y1 = bbox[1]
        x2 = bbox[2]
        y2 = bbox[3]

        box_w = x2 - x1 # Box width
        box_h = y2 - y1 # Box height
    
        # This line will be changed later on 
        text_string = "[{}]".format(cls_id)
    
        # Draw rectangle
        # https://stackoverflow.com/questions/23720875/how-to-draw-a-rectangle-around-a-region-of-interest-in-python
        ixx_ret = cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        #cv2.imshow("box_check",ixx_ret)

        # Put string
        text_offset_x = - 2 * (box_w / 2)
        text_offset_y = - 7.0
    
        # Coordinates must be all integers
        text_offset_x = int(text_offset_x)
        text_offset_y = int(text_offset_y)
        cx = int(x1 + 10)

        # print(f"cx -> {cx}")
        # print(f"cy -> {cy}")
    
        #coord_text = (x2+text_offset_x, y2 - text_offset_y) # Print at lower bottom
        #coord_text = (x1, y1 + text_offset_y) # Print at upper left corner
        coord_text = (cx, y1 + text_offset_y) # Print at upper middle
        ixx_ret = cv2.putText(img, text_string, coord_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, col_text, 2)
                
        return ixx_ret
    # ------------------------------------------------------------------------------------------------------------- 

    # -------------------------------------------------------------------------------------------------------------
    @staticmethod
    def calculate_bbox_center(cord_pack):
        x1 = cord_pack[0]
        x2 = cord_pack[1]
        y1 = cord_pack[2]
        y2 = cord_pack[3]
        xcent = int((x1+x2)/2)
        ycent = int((y1+y2)/2)
    
        return xcent, ycent
    # -------------------------------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------------------------
    @staticmethod
    def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        # CORE Method
        # Author: Ultralytics
        # New version specific to yolov5
        # Resize and pad image while meeting stride-multiple constraints
        shape = im.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better val mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return im, ratio, (dw, dh)
    # ------------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @staticmethod
    def correct_hypo_box(sort_box):
        # Method to convert object hypothesis box to correct shape
        # ? Maybe redundant
        # Define working variables
        hypo_cent_pack, hypo_bbox_wh, hypo_bbox = [],[],[]
        obj_id = 0
    
        x11 = sort_box[0]
        x22 = sort_box[1]
        y11 = sort_box[2]
        y22 = sort_box[3]
        obj_id = sort_box[4]
    
        # hypo_cent_pack = [int((x11+x22)/2),int((y11+y22)/2)] # centroid
        # hypo_bbox_wh = [int(x22 - x11),int(y22 - y11)] # width, height
        hypo_bbox = [[x11,y11,x22,y22], obj_id]  # Match vector format with BLE IPS

        return hypo_bbox
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @classmethod
    def scale_coords(cls,img1_shape, coords, img0_shape, ratio_pad=None):
        # CORE FUCNTION
        # Author: Ultralytics
        # Rescale coords (xyxy) from img1_shape to img0_shape
        if ratio_pad is None:  # calculate from img0_shape
            gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
            pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
        else:
            gain = ratio_pad[0][0]
            pad = ratio_pad[1]

        coords[:, [0, 2]] -= pad[0]  # x padding
        coords[:, [1, 3]] -= pad[1]  # y padding
        coords[:, :4] /= gain
        cls.clip_coords(coords, img0_shape)
        return coords    
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @staticmethod
    def clip_coords(boxes, shape):
        # Author: Ultralytics
        # Clip bounding xyxy bounding boxes to image shape (height, width)
        if isinstance(boxes, torch.Tensor):  # faster individually
            boxes[:, 0].clamp_(0, shape[1])  # x1
            boxes[:, 1].clamp_(0, shape[0])  # y1
            boxes[:, 2].clamp_(0, shape[1])  # x2
            boxes[:, 3].clamp_(0, shape[0])  # y2
        else:  # np.array (faster grouped)
            boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, shape[1])  # x1, x2
            boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, shape[0])  # y1, y2
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @staticmethod
    def xyxy2xywh(x):
        # Transform box coordinates from [x1, y1, x2, y2] (where xy1=top-left, xy2=bottom-right) to [x, y, w, h] 
        y = torch.zeros_like(x) if isinstance(x, torch.Tensor) else np.zeros_like(x)
        y[:, 0] = (x[:, 0] + x[:, 2]) / 2  # x center
        y[:, 1] = (x[:, 1] + x[:, 3]) / 2  # y center
        y[:, 2] = x[:, 2] - x[:, 0]  # width
        y[:, 3] = x[:, 3] - x[:, 1]  # height
        return y
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @staticmethod
    def xywh2xyxy(x):
        # Transform box coordinates from [x, y, w, h] to [x1, y1, x2, y2] (where xy1=top-left, xy2=bottom-right)
        y = torch.zeros_like(x) if isinstance(x, torch.Tensor) else np.zeros_like(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
        y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
        y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
        y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
        return y
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @classmethod
    def scale_coords(cls, img1_shape, coords, img0_shape, ratio_pad=None):
        # Rescale coords (xyxy) from img1_shape to img0_shape
        if ratio_pad is None:  # calculate from img0_shape
            gain = max(img1_shape) / max(img0_shape)  # gain  = old / new
            pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
        else:
            gain = ratio_pad[0][0]
            pad = ratio_pad[1]

        coords[:, [0, 2]] -= pad[0]  # x padding
        coords[:, [1, 3]] -= pad[1]  # y padding
        coords[:, :4] /= gain
        cls.clip_coords(coords, img0_shape)
        return coords
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @staticmethod
    def clip_coords(boxes, img_shape):
        # Clip bounding xyxy bounding boxes to image shape (height, width)
        boxes[:, 0].clamp_(0, img_shape[1])  # x1
        boxes[:, 1].clamp_(0, img_shape[0])  # y1
        boxes[:, 2].clamp_(0, img_shape[1])  # x2
        boxes[:, 3].clamp_(0, img_shape[0])  # y2
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @staticmethod
    def bbox_iou(box1, box2, x1y1x2y2=True, GIoU=False, DIoU=False, CIoU=True):
        # Returns the IoU of box1 to box2. box1 is 4, box2 is nx4
        box2 = box2.t()

        # Get the coordinates of bounding boxes
        if x1y1x2y2:  # x1, y1, x2, y2 = box1
            b1_x1, b1_y1, b1_x2, b1_y2 = box1[0], box1[1], box1[2], box1[3]
            b2_x1, b2_y1, b2_x2, b2_y2 = box2[0], box2[1], box2[2], box2[3]
        else:  # transform from xywh to xyxy
            b1_x1, b1_x2 = box1[0] - box1[2] / 2, box1[0] + box1[2] / 2
            b1_y1, b1_y2 = box1[1] - box1[3] / 2, box1[1] + box1[3] / 2
            b2_x1, b2_x2 = box2[0] - box2[2] / 2, box2[0] + box2[2] / 2
            b2_y1, b2_y2 = box2[1] - box2[3] / 2, box2[1] + box2[3] / 2

        # Intersection area
        inter = (torch.min(b1_x2, b2_x2) - torch.max(b1_x1, b2_x1)).clamp(0) * \
                (torch.min(b1_y2, b2_y2) - torch.max(b1_y1, b2_y1)).clamp(0)

        # Union Area
        w1, h1 = b1_x2 - b1_x1, b1_y2 - b1_y1
        w2, h2 = b2_x2 - b2_x1, b2_y2 - b2_y1
        union = (w1 * h1 + 1e-16) + w2 * h2 - inter

        iou = inter / union  # iou
        if GIoU or DIoU or CIoU:
            cw = torch.max(b1_x2, b2_x2) - torch.min(b1_x1, b2_x1)  # convex (smallest enclosing box) width
            ch = torch.max(b1_y2, b2_y2) - torch.min(b1_y1, b2_y1)  # convex height
        if GIoU:  # Generalized IoU https://arxiv.org/pdf/1902.09630.pdf
            c_area = cw * ch + 1e-16  # convex area
            return iou - (c_area - union) / c_area  # GIoU
        if DIoU or CIoU:  # Distance or Complete IoU https://arxiv.org/abs/1911.08287v1
            # convex diagonal squared
            c2 = cw ** 2 + ch ** 2 + 1e-16
            # centerpoint distance squared
            rho2 = ((b2_x1 + b2_x2) - (b1_x1 + b1_x2)) ** 2 / 4 + ((b2_y1 + b2_y2) - (b1_y1 + b1_y2)) ** 2 / 4
            if DIoU:
                return iou - rho2 / c2  # DIoU
            elif CIoU:  # https://github.com/Zzh-tju/DIoU-SSD-pytorch/blob/master/utils/box/box_utils.py#L47
                v = (4 / math.pi ** 2) * torch.pow(torch.atan(w2 / h2) - torch.atan(w1 / h1), 2)
                with torch.no_grad():
                    alpha = v / (1 - iou + v)
                return iou - (rho2 / c2 + v * alpha)  # CIoU

        return iou
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @staticmethod
    def box_iou(box1, box2):
        # https://github.com/pytorch/vision/blob/master/torchvision/ops/boxes.py
        """
        Return intersection-over-union (Jaccard index) of boxes.
        Both sets of boxes are expected to be in (x1, y1, x2, y2) format.
        Arguments:
            box1 (Tensor[N, 4])
            box2 (Tensor[M, 4])
        Returns:
            iou (Tensor[N, M]): the NxM matrix containing the pairwise
                IoU values for every element in boxes1 and boxes2
        """

        # -----------------------------------------------
        def box_area(box):
            # Local function of box_iou
            # box = 4xn
            return (box[2] - box[0]) * (box[3] - box[1])
        # -----------------------------------------------

        area1 = box_area(box1.t())
        area2 = box_area(box2.t())

        # inter(N,M) = (rb(N,M,2) - lt(N,M,2)).clamp(0).prod(2)
        inter = (torch.min(box1[:, None, 2:], box2[:, 2:]) - torch.max(box1[:, None, :2], box2[:, :2])).clamp(0).prod(2)
        return inter / (area1[:, None] + area2 - inter)  # iou = inter / (area1 + area2 - inter)
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @staticmethod
    def wh_iou(wh1, wh2):
        # Returns the nxm IoU matrix. wh1 is nx2, wh2 is mx2
        wh1 = wh1[:, None]  # [N,1,2]
        wh2 = wh2[None]  # [1,M,2]
        inter = torch.min(wh1, wh2).prod(2)  # [N,M]
        return inter / (wh1.prod(2) + wh2.prod(2) - inter)  # iou = inter / (area1 + area2 - inter)
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @classmethod
    def non_max_suppression(cls, prediction, conf_thres=0.1, iou_thres=0.6, merge=False, classes=None, agnostic=False):
        """Performs Non-Maximum Suppression (NMS) on inference results
        Updated by Pytorch to use GPU 

        Returns:
         detections with shape: nx6 (x1, y1, x2, y2, conf, cls)
        """
        if prediction.dtype is torch.float16:
            prediction = prediction.float()  # to FP32

        nc = prediction[0].shape[1] - 5  # number of classes
        xc = prediction[..., 4] > conf_thres  # candidates

        # Settings
        min_wh, max_wh = 2, 4096  # (pixels) minimum and maximum box width and height
        max_det = 300  # maximum number of detections per image
        time_limit = 10.0  # seconds to quit after
        redundant = True  # require redundant detections
        multi_label = nc > 1  # multiple labels per box (adds 0.5ms/img)

        #t = time.time()
        output = [None] * prediction.shape[0]
        # Main loop
        for xi, x in enumerate(prediction):  # image index, image inference
            # Apply constraints
            # x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height
            x = x[xc[xi]]  # confidence

            # If none remain process next image
            if not x.shape[0]:
                continue

            # Compute conf
            x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf

            # Box (center x, center y, width, height) to (x1, y1, x2, y2)
            #box = self.xywh2xyxy(x[:, :4])
            box = cls.xywh2xyxy(x[:, :4])

            # Detections matrix nx6 (xyxy, conf, cls)
            if multi_label:
                i, j = (x[:, 5:] > conf_thres).nonzero().t()
                x = torch.cat((box[i], x[i, j + 5, None], j[:, None].float()), 1)
            else:  # best class only
                conf, j = x[:, 5:].max(1, keepdim=True)
                x = torch.cat((box, conf, j.float()), 1)[conf.view(-1) > conf_thres]

            # Filter by class
            if classes:
                x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

            # Apply finite constraint
            # if not torch.isfinite(x).all():
            #     x = x[torch.isfinite(x).all(1)]

            # If none remain process next image
            n = x.shape[0]  # number of boxes
            if not n:
                continue

            # Sort by confidence
            # x = x[x[:, 4].argsort(descending=True)]

            # Batched NMS
            c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
            boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
            i = torchvision.ops.boxes.nms(boxes, scores, iou_thres)
        
        
            if i.shape[0] > max_det:  # limit detections
                i = i[:max_det]
            if merge and (1 < n < 3E3):  # Merge NMS (boxes merged using weighted mean)
                try:  # update boxes as boxes(i,4) = weights(i,n) * boxes(n,4)
                    #iou = self.box_iou(boxes[i], boxes) > iou_thres  # iou matrix
                    iou = cls.box_iou(boxes[i], boxes) > iou_thres  # iou matrix
                    weights = iou * scores[None]  # box weights
                    x[i, :4] = torch.mm(weights, x[:, :4]).float() / weights.sum(1, keepdim=True)  # merged boxes
                    if redundant:
                        i = i[iou.sum(1) > 1]  # require redundancy
                except:  # possible CUDA error https://github.com/ultralytics/yolov3/issues/1139
                    print(x, i, x.shape, i.shape)
                    pass

            output[xi] = x[i]
            #if (time.time() - t) > time_limit:
            #    break  # time limit exceeded

        return output
    # ------------------------------------------------------------------------

    # ------------------------------------------------------------------------
    @staticmethod
    def time_synchronized():
        # Original author: WungKim
        torch.cuda.synchronize() if torch.cuda.is_available() else None
        return time.time()

    # ------------------------------------------------------------------------

    # -------------------------------------------- DO NOT DELETE UNTIL FULLY PORTED -------------------------------
    # ------------------------------------------------------------------------------------------
    # CORE METHOD
    @classmethod
    def extract_semantic_info(cls, extracted_det, static_objs_database, ixx, orb):
        # Master function that maskes and semantically computes orb points belonging to static objects
    
        # Define variables
        bbox_static_objs = []
        static_objs_in_frame = []
        dynamic_obs_in_frame = []
        kp = []
        des = []

        # Filter out bbox for static objects in the enviornment
        # patches_static_objs_in_env  -- returns return bbox_out_static, cls_static_objs, bbox_out_dyna, cls_dyna_objs  
        bbox_static_objs, static_objs_in_frame, _, dynamic_obs_in_frame= cls.patches_static_objs_in_env(extracted_det, static_objs_database)

        # Extract image patch from original image for this class
        ixx,kp,des = cls.find_keypoints_in_patches(bbox_static_objs, ixx, orb)

        # Objects in extracted_det which are not dynamic objects
        #print(extracted_det)

        return ixx, static_objs_in_frame, dynamic_obs_in_frame
    # ------------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------------------------
    @classmethod
    def find_keypoints_in_patches(cls, bbox_static_objs, iox_prev, orb):
        # CORE FUNCTION
        # Returns processed image, keypoints and description of keypoints
        # TODO Add a control to show bbox and class name on detected figures

        """
        Parameters
        bbox_static_objs -- # List in list, bbox belonging to static objects in the scene
        orig_img -- Numpy copy of the input image
        orb -- An object beloning to cv2.ORB class
        iox_prev -- An image already containing the bounding box assessts

        These few lines may be depricit and be deleted later on
        kp, des = orb.detectAndCompute(wrk_img,None)
        show_image(ixx_gray, "color_frame", None, [640,480])
        #print(f"kp type --> {type(keypoints)}")
        # ixx_gray = cv2.drawKeypoints(ixx_gray, kp, None, color=colors[1], flags=0)
        
        """
        # Define variables
        k1 = None # Keypoints
        ds1 = None # Descriptor of keypoints

        # Create a working copy to prevent orig_img from getting modified
        #wrk_img_1 = cv2.cvtColor(copy.deepcopy(iox_prev), cv2.COLOR_BGR2GRAY)
        wrk_img_1 = cv2.cvtColor(copy.deepcopy(iox_prev), cv2.COLOR_BGR2GRAY)

        # Using bbox, create maskes for this image
        roi = cls.create_rect_mask(wrk_img_1, bbox_static_objs)

        # Detect keypoints on the masked images
        kp1, ds1 = orb.detectAndCompute(roi,None)
    
        # Coordinates when keypoints are detected on whole image
        # ds1 --> numpy array of shape [num_of_keypoints * 32]
        #t0 = current_milli_time()
        #kp1, ds1 = orb.detectAndCompute(roi,None)
        #t_del = current_milli_time() - t0
        
        # print(f"Keypoints --> {len(kp1_xy_ls)}, Time --> {t_del} ms on {len(bbox_static_objs)} patches")
        # print(f"Keypoints --> {len(kp1)}, Time --> {t_del} ms on {len(bbox_static_objs)} patches")
        # print(ds1) 
        # print(ds1.shape)
        # print(ds1[0,:]) # Each row is a BREIF -32 descriptor
        
        # Draw points for human-readable outputs
        roi = cv2.drawKeypoints(wrk_img_1, kp1, None, color=(0,0,255), flags=0) # Proposed ORB, show only keypoints and bbox
        
        return roi, kp1, ds1
    # ------------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------------------------
    @staticmethod
    def create_rect_mask(image, bbox):
        # HELPER FUNCTION
        # TODO modify this to send back all the image except dynamic object patches
        # Returns masked image containing only static object images
        
        """
        Parameters
        imgz --> orig grayscaled image
        bbox --> bounding boxes to mask images
        """
        mask = np.zeros(image.shape[:2], dtype="uint8")
        masked = [] # Output image
        
        # Cycle through all bounding boxes
        for bx in bbox:
            top_left = (bx[0], bx[1])
            bottom_right = (bx[2], bx[3])
            
            # Create a white mask 
            cv2.rectangle(mask, top_left, bottom_right, 255, -1)

        # Apply mask on the work image
        masked = cv2.bitwise_and(image, image, mask=mask)
        return masked
    # ------------------------------------------------------------------------------------------

    # ------------------------------------------------------------------------------------------
    @classmethod
    def patches_static_objs_in_env(cls,extracted_det,static_objs):
        # HELPER FUNCTION
        # Extracts bbox for static objects in the enviornment
        # Also return the class labels for each object
        """
        Parameters
        extracted_det
        static_objs --> dictionary in format class_id:class_name
        bbox_out_static and bbox_out_dynamic will not have any class id
        """
        
        # Define variables
        bbox_out_static = [] # List in list
        bbox_out_dyna = [] # List in list, bbox and class labels of dynamic objects
        cls_static_objs = [] # List of class labels of static objects detected
        cls_dyna_objs = [] # List of class labels of dynamic objects detected

        # Build a list of keys
        static_keys = cls.getList(static_objs)
        #print(f"{static_keys}, {type(static_keys)}")

        # Cycle thru each row of extracted_det and find bboxes for static objects
        for det in extracted_det:
            cls_id_in_row = det[-1] # Last value in list is the class id
            in_det = False # Boolean set True if current clas_id is found in static object rows
            tempo_row = [] # List to bbox coordinates

            # Hold bounding box coordinates [x1,x2,y1,y2] in a temporary row
            for iox in det[:-1]:
                tempo_row.append(iox) # [x1,x2,y1,y2]
            
            # Check if this row's class_id is available in the static_objs dictionary
            in_det = cls_id_in_row in static_keys
            
            # Extract this object's bbox and find class name
            if (in_det == True):
                #print(f"This row class id in static keys--> {in_det}")  
                cls_static_objs.append(cls_id_in_row)
                bbox_out_static.append(tempo_row)
            else:
                # Object belongs to moving dynamic_objs group
                cls_dyna_objs.append(cls_id_in_row)
                bbox_out_dyna.append(tempo_row)
            
        # print(f"bbox_out_stat --> {bbox_out_static}")
        # print(f"dynamic objects -- {cls_dyna_objs}")
        
        return bbox_out_static, cls_static_objs, bbox_out_dyna, cls_dyna_objs               
    # ------------------------------------------------------------------------------------------
    
    
# ------------------------------------ EOF of Semantic_Extractor -----------------------------------------------
