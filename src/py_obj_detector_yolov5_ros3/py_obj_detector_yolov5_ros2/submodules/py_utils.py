"""
Contains some utility python imports and function definitions
Author: Azmyin Md. Kamal
Date : 10/19/2022, modified 04/09/2024
Version: 1.0

"""

"""
NOTES
* Join paths to neccesary components/packages
--> sys.path.append("/home/icore_base/Documents/semantic_single_agent/thirdparty/g2opy/lib") # Point to thirdparty libraries
"""

# * Python Modules
import sys # System specific modules
import os # Operating specific functions
import glob # path operation tools
import time # Python timing module
import copy # For deepcopying arrays
import shutil # High level folder operation tool
from pathlib import Path # To find the "home" directory location
import argparse # To accept user arguments from commandline
import natsort # To ensure all images are chosen loaded in the correct order
import yaml # To manipulate YAML files for reading configuration files
import copy # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
import numpy as np # Python Linear Algebra module
import cv2 # OpenCV
import yaml # Python module to real YAML files


def debug_lock():
    """Locks system in an infinite loop for debugging."""
    print("LOCK")
    while (1):
        pass

def get_obs_weight_config(query_key, file_path, this_package_path):
    """Return the object database from obj_database.yaml file."""
    with open(file_path, 'r') as yaml_file:
        data = yaml.safe_load(yaml_file)
        stat_objs = data[query_key]['static_objs_names']
        dyna_objs = data[query_key]['dynamic_objs_names']
        pw = data[query_key]['weights'] # weights
        pd = data[query_key]['yolov5_config'] # custom_<name>.yaml
    
    #* Update to full path address i.e. /home/ros2_ws/src/<package_name>/weights_configs/
    path_to_weights = this_package_path + "weights_configs/" + pw
    path_to_data = this_package_path + "weights_configs/" + pd
    return stat_objs, dyna_objs, path_to_weights, path_to_data

def curr_time():
    """Return time tick in milliseconds."""
    return time.monotonic() * 1000

def retrieve_camera_config(yaml_file, key_ls):
    """Retrieve camera configurations from a .yml file."""

    """
    Parameters
    yaml_file -- location of the yaml file to read
    
    # key_ls = ["Camera1.width","Camera1.height","Camera1.fx", 
    "Camera1.fy","Camera1.cx","Camera1.cy","Camera1.k1",
    "Camera1.k2","Camera1.k3","Camera1.p1","Camera1.p2",
    "az_orb.nFeatures","az_orb.scoreType","az_orb.FLANN_INDEX_LSH",
    "az_orb.flann_param_table_number","az_orb.flann_param_key_size",
    "az_orb.flann_param_multi_probe_level",
    "az_orb.flann_search_params","Camera1.fps"]
    
    """
    # Define variables
    intrinsic_val = []
    dist_coef = []
    orb = None
    flann = None
    im_w, im_h = 0.0, 0.0
    fx,fy,cx,cy = 0,0,0,0
    k1,k2,p1,p2 = 0,0,0,0
    orb_nFeatures = None
    orb_scoreType = None
    FLANN_INDEX_LSH = None
    flann_table_number = None
    flann_key_size = None
    flann_multi_probe_level = None
    flann_checks = None
    cam_fps = 0.0

    # Open YAML file with a context manager
    with open(yaml_file) as f:  
        data = f.read()

    # Create a dictionary from the YAML file
    dictionary = yaml.load(data, Loader=yaml.FullLoader)
    
    # Bust out values from dictionary
    im_w = dictionary[key_ls[0]]
    im_h = dictionary[key_ls[1]]
    fx = dictionary[key_ls[2]]
    fy = dictionary[key_ls[3]]
    cx = dictionary[key_ls[4]]
    cy = dictionary[key_ls[5]]
    k1 = dictionary[key_ls[6]]
    k2 = dictionary[key_ls[7]]
    k3 = dictionary[key_ls[8]]
    p1 = dictionary[key_ls[9]]
    p2 = dictionary[key_ls[10]]
    orb_nFeatures = dictionary[key_ls[11]]
    #orb_scoreType = dictionary[key_ls[12]] # !NOTE need to fix this later
    FLANN_INDEX_LSH = dictionary[key_ls[13]]
    flann_table_number = dictionary[key_ls[14]]
    flann_key_size = dictionary[key_ls[15]]
    flann_multi_probe_level = dictionary[key_ls[16]]
    flann_checks = dictionary[key_ls[17]]
    cam_fps = dictionary[key_ls[18]]

    # Define ORB Object
    # https://github.com/methylDragon/opencv-python-reference/blob/master/02%20OpenCV%20Feature%20Detection%20and%20Description.md
    orb = cv2.ORB_create(nfeatures = orb_nFeatures, scaleFactor=1.2, nlevels=8, scoreType= cv2.ORB_FAST_SCORE)
    
    # Define FLANN object
    # TODO needs to be depricited
    index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=flann_table_number, key_size=flann_key_size, 
    multi_probe_level=flann_multi_probe_level)
    search_params = dict(checks=flann_checks)
    flann = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)

    # Pack arrays
    intrinsic_val = [im_w,im_h,fx,fy,cx,cy]
    dist_coef = [k1,k2,k3,p1,p2]

    #return orb, flann, cam_P_mat, cam_K_mat
    return orb, flann, intrinsic_val, dist_coef, cam_fps, orb_nFeatures

def show_image(img, win_name, win_size, win_move_loc,wait_key):
    """Show image in OpenCV window."""
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
        print("Error in displaying image")

def save_processed_image(imgz, imgz_name, imgz_save_loc, add_str_to_imgz_name ,frame_id):
    """Save this frame into folder."""
    """
    Parameters
    imgz_save_loc in "fldr_pth/" form, just add file name to save here
    """
    f_name = imgz_name + add_str_to_imgz_name + ".png"
    pathx = imgz_save_loc + f_name
    
    try:
        cv2.imwrite(pathx,imgz)
    except:
        print("Unable to save image!")

def show_save_image(ixx, win_name, wait_key_val, show_imgz, save_imgz, imgz_name, imgz_save_loc, add_str_to_imgz_name, frame_count):
    """Shows and/or saves a processed image."""
    # Display image on a window
    if (show_imgz):
        #show_image(ixx, "color_frame", None, [1280,1080],wait_key_val)
        show_image(ixx, win_name, [ixx.shape[0], ixx.shape[1]], None,wait_key_val)
    # Save image at last
    if (save_imgz):
        save_processed_image(ixx, imgz_name, imgz_save_loc, add_str_to_imgz_name, frame_count)
    
def mat_printer(mat):
    """Prints each row of a 2D matrix in terminal.""" 
    print()
    print("*****************************************")
    for mx in mat:
        print(mx)
    print("*****************************************")
    print()

def create_folder_in_parent_dir(folder_name_with_path):
    """Create a folder in the parent directory if it does not exsist."""
    """
    Parameters
    folder_name_with_path
    """
    # Make sure to pass the full folder name prior to executing this function
    if not os.path.exists(folder_name_with_path):
        os.makedirs(folder_name_with_path)
    else:
        pass

def delete_and_make_folder(folder_name_with_path):
    """Recreate a folder everytime this function is invoked."""
    """
    Parameters
    folder_name_with_path
    """
    # CORE FUNCTION
    # 
    if not os.path.exists(folder_name_with_path):
        os.makedirs(folder_name_with_path)
    else:
        # First delete the exsisting folder and then recreate a fresh one
        shutil.rmtree(folder_name_with_path)
        os.makedirs(folder_name_with_path)
