import numpy as np
import cv2
import os


depth_data = np.load('/home/yin/graspnet-baseline/doc/data/depth.npy') 

depth_data = depth_data.astype(np.uint16)

save_path = '/home/yin/graspnet-baseline/doc/data' 
if not os.path.exists(save_path):
    os.makedirs(save_path)

cv2.imwrite(os.path.join(save_path, 'depth.png'), depth_data)

print("depth has been saved successfully")
