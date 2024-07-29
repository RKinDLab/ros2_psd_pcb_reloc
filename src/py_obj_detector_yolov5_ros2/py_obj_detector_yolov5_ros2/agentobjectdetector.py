"""AgentObjectDetector class defition file."""

#* Python / Pytorch modules
import os # Operating specific functions
import time # Python timing module
import copy # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
from pathlib import Path # To find the "home" directory location
import natsort  # Python's natural sorting module
import numpy as np # Python Linear Algebra module
import cv2 # Python OpenCV library

#* Import from submodules
from .submodules.py_utils import debug_lock
from .submodules.py_utils import get_obs_weight_config, curr_time

#* YOLOv5 imports
from .neural_net.neural_network import Neural_Network_Engine
from .neural_net.models.common import DetectMultiBackend
from .neural_net.utils.general import check_img_size

#* ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

#* ROS 2 message imports
from matimg_custom_msg_interface.msg import MatImg  # Custom message
from std_msgs.msg import String # ROS2 string message template
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension # http://docs.ros.org/en/melodic/api/std_msgs/html/msg/MultiArrayLayout.html
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array

import torch  # Pytorch
import torch.backends.cudnn as cudnn
import yaml # Python module to real YAML files
cudnn.benchmark = True # Not sure but needed

class AgentObjectDetector(Node):
    """

    Class that performs object detection and sends rgbd data to VSLAM node for a chosen agent.

    Autmoatically find image files, perform object detection, build semantic matrix 
    and pass data to appropirate VLSAM node

    """

    def __init__(self, node_name = "py_node"):
        """Class constructor."""
        super().__init__(node_name) # Initializes the rclpy.Node class with an unique name
        # ROS specific variables
        self.node_name = node_name
        if self.node_name not in ['robot0', 'robot1']:
            err_msg = "Node name must be either 'robot0' or 'robot1'!!"
            raise ValueError(err_msg)
        self.rate  = None # Update rate for this ROS node
        self.glob_yamls_dir = os.environ["ROS2_WS_PATH"] + "/global_yamls/"
        if not Path(self.glob_yamls_dir).is_dir():
            err_msg = "/global_yamls directory does not exsist in root of the workspace!"
            raise FileNotFoundError(err_msg)
        # Path to this package
        self.this_node_dir = os.environ["ROS2_WS_PATH"] + "/src/py_obj_detector_yolov5_ros2/"
        # Initialize ros2 parameters
        self.declare_parameter("experiment","NULL")   # Either "ieeeaim2024" or "other"  
        self.declare_parameter("image_sequence","NULL") # Dataset in \home\DATASETS\sub_dir director

        # Experiment related variables
        # Resolve name of experiment
        self.experiment_name = str(self.get_parameter('experiment').value)
        if self.experiment_name not in ['ieeeaim2024', 'others'] or self.experiment_name == "NULL":
            err_msg = "Experiment name must be either 'ieeeaim2024' or 'others'"
            raise ValueError(err_msg)
        # Build path to the sub directory
        # Each subdiretory contains image sequence datasets to complete one full experiment
        self.sub_dir = ""
        if self.experiment_name == "ieeeaim2024":
            self.sub_dir = "IEEEAIM2024_DATASETS/"
        else:
            self.sub_dir = "OTHERS/"
        self.path_to_subdir = str(Path.home()) + "/DATASETS/" + self.sub_dir
        # Resolve name of the particular sequence we want to run experiments on
        self.image_sequence = str(self.get_parameter('image_sequence').value) 
        if self.image_sequence == "NULL":
            err_msg = "Image sequence name not set!"
            raise ValueError(err_msg)
        # Recover static and dyanmic object database defined for this image sequence
        self.obj_database_yaml = self.glob_yamls_dir + "dataset_db.yaml"
        if not Path(self.obj_database_yaml).is_file():  # noqa: PTH113, PTH118
            err_msg = "dataset_db.yaml not found in /global_yamls directory!"
            raise FileNotFoundError(err_msg)
        # VSLAM agent configurations
        self.agent_name = self.node_name # Default for single agent scenario
        if self.agent_name == "robot0":
            self.vpr_method = "psd_pcb" # robot0 runs proposed relocalization pipeline
        elif self.agent_name == "robot1":
            self.vpr_method = "bag" # robot1 runs baseline relocalization pipeline

        """
        obj_database.yaml contains the static and dynamic object names as collection of python
        dictionaries under one query key. This top level name corresponds to the name of the 
        camera calibration yaml file that vslam node uses.

        """ 
        self.database_query_key = self.get_database_query_key()

        # # Path to image sequence may contain both rgb and depth data
        # self.dataset_dir = self.path_to_subdir + self.image_sequence
        # self.agent_dataset_dir = self.dataset_dir + "/" + self.agent_name + "/"
        if self.experiment_name == "ieeeaim2024":
            self.dataset_dir = self.path_to_subdir + self.image_sequence
            #! HARDCODED for IEEE AIM 2024 experiment both system uses robot0 data
            self.agent_dataset_dir = self.dataset_dir + "/" + "robot0" + "/"
        else:
            # Path to image sequence may contain both rgb and depth data
            self.dataset_dir = self.path_to_subdir + self.image_sequence
            self.agent_dataset_dir = self.dataset_dir + "/" + self.agent_name + "/"

        # TODO need to check what happens when datasets does not have depth data
        # Get rbg and depth image data for agent0
        self.rgb_dir = self.agent_dataset_dir + "cam0/data/"       # .../robot0/cam0/data/
        self.depth_dir = self.agent_dataset_dir + "depth0/data/"   # .../robot0/depth0/data/
        self.rgb_sequence = []     # [<1>.png, <2>.png]
        self.depth_sequence = []     # [<1>.png, <2>.png]
        self.rgb_timesteps = []    # [<long_int_1>, <long_int_2>]
        self.depth_timesteps = []  # [<long_int_1>, <long_int_2>]
        if self.experiment_name == "ieeeaim2024":
            # Path to image sequence may contain both rgb and depth data
            self.get_agent_rbg_data()
        else:
            self.get_agent_rgbd_data()

        print()
        print(f"RGB images loaded: {len(self.rgb_sequence)}")
        print(f"Depth images loaded: {len(self.depth_sequence)}")
        print()

        # Populates object database and path to weights
        self.static_db = {}
        self.dynamic_db = {}
        self.path_pw = "" # Full path to .pth weight files
        self.path_pd = "" # Full path to custom_<name>.yaml file

        """
        def obj_database_path_to_weights(query_key, file_path, this_package_path) -> Tuple[any, any, any]
        yaml files needed by yolov5 are also located inside the /global_yamls/yolov5_yamls/ directory
        """
        # self.static_obj_database, self.dynamic_obj_database, self.path_to_weights, self.path_to_data = obj_database_path_to_weights(self.database_query_key,   # noqa: E501
        #                                                  self.obj_database_yaml, self.this_node_dir)

        self.static_db, self.dynamic_db, self.path_pw, self.path_pd = get_obs_weight_config(self.database_query_key,
                                                                                             self.obj_database_yaml,
                                                                                             self.this_node_dir)
        # Define OpenCV object
        self.br = CvBridge()

        #* Configure YOLOv5 network
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.conf_thres = 0.65 # Class confidence threshold
        self.nms_thres = 0.45 # min-max suppression threshold
        self.max_det=200
        self.network_dim = 416
        self.imgsz=(self.network_dim, self.network_dim) # New in yolov5 implementation
        self.half=True 
        self.dnn=False
        self.visualize=False
        self.augment=False
        self.colors = [(255,0,0),(0,0,255),(63,127,0)]

        print("Loading YOLOv5 network ................")

        t00 = curr_time()
        #* Setup the YOLOv5 model
        self.model = DetectMultiBackend(weights = self.path_pw, device=self.device, dnn=self.dnn, data=self.path_pd)

        # Unpack
        self.stride, self.names, self.pt, self.jit = self.model.stride, self.model.names, self.model.pt, self.model.jit 
        self.onnx, self.engine = self.model.onnx, self.model.engine

        self.nn_settings = [self.network_dim, self.device, self.half, self.stride, self.pt,
                            self.model, self.imgsz, self.conf_thres, self.nms_thres,
                            self.augment, self.visualize]

        self.imgsz = check_img_size(self.imgsz, s=self.stride)

        #Activate half precision
        if self.half:
            self.model.model.half()
        else: 
            self.model.model.float()

        self.yolov5_network = Neural_Network_Engine(self.nn_settings, self.colors, self.static_db)
        print("YOLOv5 network ready")
        print(f"YOLOv5 loaded in {curr_time() - t00:.3f} ms\n")

        print("Initializing ROS2 publishers and subscribers ......\n")
        self.pub_exp_config_name = "/" + self.agent_name + "/experiment_settings" 
        self.sub_exp_ack_name = "/" + self.agent_name + "/exp_settings_ack"
        self.pub_matimg_to_agent_name = "/" + self.agent_name + "/matimg_msg"
        self.send_config = True # Set False once parameters are received

        # TODO ------------ remove in ros2_psd_pcb reloc repo
        self.run_fast_mf = ""
        if self.agent_name == "robot0":
            self.run_fast_mf = "/" + self.agent_name + "/run_fast_mf_flag_msg"
        
        # robot0 can send rendevouz event
        if self.agent_name == "robot0":
            self.send_rendevouz_event = True # Set false to prevent sending data anymore 
        else:
            self.send_rendevouz_event = False
        self.robot0_rendevouz_id = -1 # Manually select a Frame to start rendevouz event 
        # TODO ------------ remove in ros2_psd_pcb reloc repo
        
        self.show_sent_msg = True # Flag to show what config was sent only once
        self.setup_ros2_pub_sub()   # Setup ROS publishers and subscribers
        
        # Initialize work variables
        self.start_frame = 0 # Default 0
        self.end_frame = -1 # Set -1 for entire sequence
        self.frame_stop = -1 # Set -1 to use the whole sequence, some positive integer to force sequence to stop, 350 test2, 736 test3
        self.show_rgb_depth_output = False # Default, False, set True to see the output directly from this node
        self.frame_id = 0 # Global integer id of frames processed, always begins from 0
        self.frame_count = 0 # Ensure we are consistent with the count number of the frame
        self.sleep_time = 0.00 # 75ms works as of July 2023, 0.00ms works as of Sep 9th, 23
        self.warmup_frames = 5 # How many frames to use before warmup is complete
        self.warmup_msg_flag = True # Set false to view the message only once, TODO this made no sense
        
        # Global statistics
        self.inference_time = [] # List to compute average time
        
        # DEBUG
        print()
        print(f"I AM AGENT: {self.agent_name}")
        print(f"self.image_sequence: {self.image_sequence}")
        print(f"self.dataset_dir: {self.dataset_dir}")
        print(f"self.pub_exp_config_name {self.pub_exp_config_name}") # Publish configuration to VSLAM node
        print(f"self.sub_exp_ack_name {self.sub_exp_ack_name}") # Get ACK for received config by VSLAM node
        print(f"self.pub_matimg_to_agent_name: {self.pub_matimg_to_agent_name}") # Topic to send MatImg message
        print()

    def build_experiment_string(self):
        """
        Build the experiment configuration string.

        Values are set before this method.
        """
        #* IEEE AIM 2024 experiment configurations
        if self.experiment_name == "ieeeaim2024":
            self.do_reloc_experiment = True
        else:
            self.do_reloc_experiment = False
        # 05/15/2024 experiment_name /image_sequence/
        # example string : "ieeeaim2024/NSF24_COMBO/LSU_iCORE_RGBD/psd_pcb/-1/False"
        self.exp_config_msg = self.experiment_name + "/"  + self.image_sequence + "/"
        self.exp_config_msg = self.exp_config_msg + self.database_query_key + "/" + self.vpr_method + "/"
        self.exp_config_msg = self.exp_config_msg + str(self.manual_reloc) + "/" + str(self.do_reloc_experiment)
        print(f"{self.agent_name} configuration string sent: {self.exp_config_msg}")

    def get_database_query_key(self):
        """Return database query key name based on image sequence name."""
        str_to_ret = ""
        if self.image_sequence in {"NSF24_COMBO"}:
            str_to_ret = "LSU_iCORE_MONO"
        elif self.image_sequence in {"MH01", "MH02", "MH03", "MH04", "MH05"}:
            str_to_ret = "MACHINE_HALL"
        elif self.image_sequence in {"V101", "V102", "V103", "V201", "V202", "V203"}:
            str_to_ret = "VICON_ROOM"
        elif self.image_sequence in {"FR2PIONEER360", "FR2PS1", "FR2PS2"}:
            str_to_ret = "TUM_FR2"
        elif self.image_sequence in {"ROBOTICSLAB0", "ROBOTICSLAB1", "ROBOTICSLAB2"}:
            str_to_ret = "LSU_iCORE_MONO"
        else:
            err_msg = "Unknown image sequence!"
            raise NotImplementedError(err_msg)
        return str_to_ret

    def setup_ros2_pub_sub(self):
        """Configure all publishers and subscribers used by this node."""
        # TODO delete parts not related to ros2_psd_pcb reloc repo
        # Publisher to send settings to CPP node
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        # Subscriber to get acknowledgement from CPP node that it received experimetn settings
        self.subscribe_exp_ack_ = self.create_subscription(String,self.sub_exp_ack_name, self.ack_callback ,10)
        self.subscribe_exp_ack_
        self.publish_mat_img_ = self.create_publisher(MatImg, self.pub_matimg_to_agent_name, 1) # Publisher to send RGM image and semantic matrix
        # TODO -------------------------- delete from ros2_psd_pcb repo -------------------------------
        # NSF24 DEMO, only robot0, run fast_mf when the rendevouz frame is reached
        if self.send_rendevouz_event:
            self.publish_run_fast_mf_ = self.create_publisher(String, self.run_fast_mf, 1)
        # TODO -------------------------- delete from ros2_psd_pcb repo -------------------------------

    def handshake_with_vslam_node(self):
        """Handshake with VSLAM node to get ack for configuration sent."""
        if self.show_sent_msg:
            # print(f"Configuration string sent: {self.exp_config_msg}")
            self.show_sent_msg = False
        self.send_config_to_cpp_node() # Blocking
        # time.sleep(1)   # Allow C++ node to fire up

    def send_config_to_cpp_node(self):
        """Send configuration string to VSLAM node."""
        if self.send_config is True:
            # print(f"Sent mesasge: {self.exp_config_msg}")
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def get_agent_rbg_data(self):
        """
        Return RGB image paths and list of timesteps in ascending order.

        Only used for Monocular cases such as IEEE AIM 2024 experiment
        """
        self.rgb_sequence = natsort.natsorted(os.listdir(self.rgb_dir),reverse=False)
        # Extract timesteps from image names
        if not self.rgb_sequence:
            err_msg = "No RGB images found, check dataset/robot#/cam0/data directory"
            raise FileNotFoundError(err_msg)
        else:
            for iox in self.rgb_sequence:
                ttt = iox.split(".")[0]
                self.rgb_timesteps.append(ttt)
                #print(time_step)

    def get_agent_rgbd_data(self):
        """
        Return images and list of timesteps in ascending order for rgb and depth.

        Image sequences may not have depth maps but it must have RGB images
        """
        self.rgb_sequence = natsort.natsorted(os.listdir(self.rgb_dir),reverse=False)
        self.depth_sequence = natsort.natsorted(os.listdir(self.depth_dir),reverse=False)
        # Extract timesteps from image names
        if not self.rgb_sequence:
            err_msg = "No RGB images found, check dataset/robot#/cam0/data directory"
            raise FileNotFoundError(err_msg)
        else:
            for iox in self.rgb_sequence:
                ttt = iox.split(".")[0]
                self.rgb_timesteps.append(ttt)
                #print(time_step)
        if not self.depth_sequence:
            err_msg = "No Depth images found, check dataset/robot#/depth0/data directory"
            raise FileNotFoundError(err_msg)
        else:
            for iox in self.depth_sequence:
                ttt = iox.split(".")[0]
                self.depth_timesteps.append(ttt)
                #print(time_step)

    def ack_callback(self, msg):
        """Send ack for receiving configuration string."""
        if(msg.data == "ACK"):
            print(f"Got ack: {msg.data}")
            self.send_config = False
            # self.subscribe_exp_ack_.destory() # Doesn't work 

    def run_warmup_routine(self,imgz_path, tstep, debug_view)->None:
        """Execute warmup routine for YOLOv5."""
        # Kept to debug error cuz except statement is not very clear on what type of error occured
        # imgz_warm = np.asarray(cv2.imread(imgz_path)) # This copy will be used by NN and ORB
        # img_ww = copy.deepcopy(imgz_warm)    
        # _,_ = self.yolov5_network.get_semantic_knowledge(imgz_warm, self.agent_name, debug_view = debug_view)
        try:
            imgz_warm = np.asarray(cv2.imread(imgz_path)) # This copy will be used by NN and ORB
            #img_ww = copy.deepcopy(imgz_warm)
            # NOTE execution time not record time for warm up for some reason it reports wrong time
            # stat_flg, semantic_matrix, ixx
            _,_ = self.yolov5_network.get_semantic_knowledge(imgz_warm, self.agent_name, debug_view = debug_view)
        except:
            print(f"Error reading image: {format(tstep, '.0f')}.png")
        return None

    def run_detection_agent(self, imgz_path, timestep, debug_flag):
        """
        Do 2D Object detection using YOLOv5 on a RGB image.

        returns stat_flag, and the ros message to pass the semantic matrix
        """
        # Initialize work varialbes
        stat_flg_nn = False
        sem_matrix_ros_msg = None

        # Read image and convert to numpy array
        try:
            img0 = np.asarray(cv2.imread(imgz_path)) # This copy will be used by NN and ORB
            img = copy.deepcopy(img0)
        except:
            print(f"Error in reading image for timestep -- {timestep} for agent -- {self.agent_name}")
            #rospy.loginfo(f"") 
            return stat_flg_nn, sem_matrix_ros_msg
        
        # Extract semantic knowledge
        str_win = "rgb_" + self.agent_name
        stat_flg_nn,_ = self.yolov5_network.get_semantic_knowledge(img0, str_win ,debug_view = debug_flag)
        # Form ros message
        sem_matrix_ros_msg = self.prep_multiarray_message(self.yolov5_network.semantic_matrix)
        return stat_flg_nn, sem_matrix_ros_msg

    def prep_multiarray_message(self, sem_mat, debug_verbose = False):
        """
        Convert Python list in list "semantic_matrix" into a MultiDimensional Message.

        Inputs
            semantic_matrix -- Python list in list, [1 x M], 2D array flattened into row vector where each element is a row of the 2D matrix
        Outputs
            mat -- ROS Float32MultiArray message, [Mx5], semantic_matrix converted to Float32MultiArray ros message
        Notes
        For a 2D Float32MultiArray
        Useful tutorial -- https://gist.github.com/jarvisschultz/7a886ed2714fac9f5226
        stride of the zeroth dimension is indeed the total number of entries
        stride of the first dimension is the number of columns
        Why Float32MultiArray ? -- Too lazy and this works as per tutorial (FYI, Mr. Schultz took the time to update the tutorial to help me with this project)
        """  # noqa: D415
        # Initialize the 2D Float32MultiArray matrix
        height = len(sem_mat) # Number of rows
        width = 5 # number of columns, HARDCODED
        mat = Float32MultiArray()
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim.append(MultiArrayDimension())
        mat.layout.dim[0].label = "height" # Number of rows
        mat.layout.dim[1].label = "width"  # Number of columns
        mat.layout.dim[0].size = height # Number of rows
        mat.layout.dim[1].size = width # Number of columns
        mat.layout.dim[0].stride = height * width # (note dim[0] stride is just the total number of elements)
        mat.layout.dim[1].stride = width # dimp[1] is total number of columns
        mat.layout.data_offset = 0 # No offset
        mat.data = [0.0]*height*width # Populate with data, number of rows and number of columns
        # Save some dimensions
        dstride0 = mat.layout.dim[0].stride
        dstride1 = mat.layout.dim[1].stride
        offset = mat.layout.data_offset
        if (debug_verbose):
            tmpmat = np.zeros((height,width)) # To check if we are correctly building the array

        for i, sem in enumerate(sem_mat):
            for j in range(width):
                num = np.empty((1,1)) # Initialize
                num = sem[j] # Access ith row, jth columns value
                mat.data[offset + i*dstride1 + j] = num # Push(i,j)th element in mat 2D matrix
                if (debug_verbose):
                    tmpmat[i,j] = num
        if(debug_verbose):
            print(mat)
            print()
            print(tmpmat)
        return mat

    def obj_database_path_to_weights(self, query_key, file_path, this_package_path):
        """Reads the object database from obj_database.yaml file."""
        
        with open(file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
            stat_objs = data[query_key]['static_objs_names']
            dyna_objs = data[query_key]['dynamic_objs_names']
            pw = data[query_key]['path_to_weights']
            pd = data[query_key]['path_to_data']
        
        #* Update to full path address i.e. /home/ros2_ws/src/<package_name>/<package_name>/submodules/neural_net/data/...
        path_to_weights = this_package_path  + pw
        path_to_data = this_package_path + pd 

        return stat_objs, dyna_objs, path_to_weights, path_to_data

    def visualize_depthmap(self, depth_img):
        """Visualize depth map in 8bit format suitable for vieiwing by OpenCV."""
        try:
            # The depth image is a single-channel float32 image the values is the distance in mm in z axis
            # Convert the depth image to a Numpy array since most cv2 functions require Numpy arrays.
            cv_image_array = np.array(depth_img, dtype = np.dtype('f8'))
            # Normalize the depth image to fall between 0 (black) and 1 (white)
            # http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            # Resize to the desired size
            cv_image_resized = cv2.resize(cv_image_norm, (640, 480), interpolation = cv2.INTER_CUBIC)

            str_win = "depthmap_" + self.agent_name         
            cv2.imshow(str_win, cv_image_resized)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def prep_matimg_msg(self,br,rgb_path,rgb_timestep, depth_path, depth_timestep ,sem_matrix_ros_msg):
        """
        Build ROS 2 message with rgb image, depth image and semantic matrix.

        If RGB image conversion fails, system breaks completely
        """
        # Initialize variables
        img_msg = None
        depth_msg = None
        matimg = MatImg()
        img_msg = br.cv2_to_imgmsg(cv2.imread(rgb_path, cv2.IMREAD_UNCHANGED),
                            encoding="passthrough") # Already converted to 8UC3
        if img_msg is None:
            err_msg = "Unable to convert RGB image to sensors/Image message!"
            raise ValueError(err_msg)
        else:
            # Have valid RGB sensors/Image message
            matimg.rgb = copy.deepcopy(img_msg)
            matimg.mat = copy.deepcopy(sem_matrix_ros_msg)
            matimg.rgb_timestamp = rgb_timestep
        # There may not be depth images in the dataset
        if depth_path is None:
            pass
        else:
            depth_msg = br.cv2_to_imgmsg(cv2.imread(depth_path, cv2.IMREAD_UNCHANGED),
                        encoding="passthrough") # Already converted to 16UC1 data format
            if depth_msg is None:
                print("WARNING: unable to convert depth image to sensors/Image message")
            else:
                # Have valid depthmap as sensors/Image
                matimg.depth = copy.deepcopy(depth_msg)
                matimg.depth_timestamp = depth_timestep
        return matimg

    def run_py_node(self, idx):
        """

        Execute object detection, semantic matrix composition and publishing data for one image.

        idx: integer number corresponding to an a RGB and depth image 

        RGB images are guranteed to be there, depth images are optional
        """
        # Initialize variable for this image
        stat_flg_nn = False # Sets True if Neural Network
        sem_matrix = [] # Numpy, [Mx5] 
        matimg = None
        no_depth_image = True # Default, true, set false if there is depth image
        # RGB image
        rgb_image_name = self.rgb_sequence[idx]
        rgb_path = self.rgb_dir  + rgb_image_name
        rgb_timestep = float(rgb_image_name.split(".")[0])
        # Depth image, only work if there is indeed
        if not self.depth_sequence:
            no_depth_image = True
            depth_path = None
            depth_timestep = None
        else:
            depth_image_name = self.depth_sequence[idx]
            depth_path = self.depth_dir  + depth_image_name
            depth_timestep = float(depth_image_name.split(".")[0])
        
        # DEBUG visualize the raw depth map
        if self.show_rgb_depth_output and no_depth_image is False:
            self.visualize_depthmap(cv2.imread(depth_path))
        
        # self.depth_callback(cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)) # Make depth image 8bit grayscale and normalized between 0 to 1
        # Step Warmup routine, uses some frames from Agent 0
        # Cycle over the first few frames to warmup the YOLOv5 network
        #if (self.warmup_frame_id <= (self.start_frame + self.frame_warmup)):
        if (self.frame_id < self.warmup_frames):
            self.run_warmup_routine(rgb_path, rgb_timestep, debug_view=False)
            # self.warmup_frame_id =  self.warmup_frame_id + 1
            self.frame_id = self.frame_id + 1 # Increment Frame id     
            return # exit out of function
        else:
            if (self.warmup_msg_flag):
                print("Warmup routine complete")
                print()
                self.warmup_msg_flag = False # suppress this message from printing again
        #* Object detection
        tstart = curr_time()
        stat_flg_nn, sem_matrix = self.run_detection_agent(rgb_path, rgb_timestep, self.show_rgb_depth_output)
        self.inference_time.append(curr_time() - tstart)
        print(f"\nFrame ID: {self.frame_id}, Object detection -- {stat_flg_nn}")
        #print(f"semantic_matrix -- {sem_matrix}") # Debug print

        matimg = self.prep_matimg_msg(self.br,rgb_path,
                                      rgb_timestep, depth_path,
                                      depth_timestep, sem_matrix) # Prepare MatImg message for C++ nodes
        if matimg is not None:
            # pass
            self.publish_mat_img_.publish(matimg)   # Publish MatImg message in network
        else:
            print("Ill formed imgmsg, not publishing to node")

        # NSF24 robot0 sends rendevouz event only ONCE
        if (idx == self.robot0_rendevouz_id and self.send_rendevouz_event):
            print("Sending rendevouz event exactly once")
            msgs = String()
            msgs.data = "run"
            self.publish_run_fast_mf_.publish(msgs)
            self.send_rendevouz_event = False

        time.sleep(self.sleep_time) 
        self.frame_id = self.frame_id + 1 # Increment Frame id

