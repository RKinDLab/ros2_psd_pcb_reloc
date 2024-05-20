"""
#* Depricated, no longer used 11/29/23 DO NOT DELETE UNTIL YAML FILE IS READY
"""

'''
An anemic class that contains all the shared parameters
https://stackoverflow.com/questions/52679693/classes-with-attributes-but-no-methods
''' 
class Parameters(object):   
    
    #* YOLOv5 parameters
    conf_thres = 0.65                                   # Class confidence threshold
    nms_thres = 0.35                                    # Min-max suppression threshold
    max_det=1000                                        # Maximum number of objects to be detected
    network_dim = 416                                   # Size of letterbox image used by the neural network
    cudnn_benchmark = True                              # An nvidia specific optimizatino technique, always kept True
    half=True                                           # True --> FP 16, False --> FP 32
                                                        # !NOTE Variables needed for V5, keep all of them as is
    dnn=False                                           # use OpenCV DNN for ONNX inference
    visualize=False                                     # visualize features, don't know its purpose at this time
    augment=False                                       # augmented inference, no idea
    colors = [(255,0,0),(0,0,255),(63,127,0)]           # HARCODED, Color values, must be defined in BGR format and in integer
                                                        # Paths to weights and dataset definition
    
    # 04/06 -- Paths are manually set in a driver script 
    path_to_weights = None
    path_to_data = None
    
    frame_warmup = 3                                    # Number of initial frames used by cnn to ``warm-up''. N+1
    wait_key_val = 5                                    # How many ms to wait to show images

    #* Python Node
    # NOTE on ROS verbosity levels http://wiki.ros.org/Verbosity%20Levels
    verbose_loginfo = True                        # Set True to see Verbose message in ROS LOGINFO level
    verbose_debuginfo = False                     # Set True to see Verbose message in ROS LOGINFO level