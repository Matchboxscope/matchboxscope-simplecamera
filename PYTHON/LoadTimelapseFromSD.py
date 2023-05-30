#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 29 15:20:43 2023

@author: bene
"""

import glob
import cv2
import numpy as np
import matplotlib.pyplot as plt

import tifffile as tif
# Directory path
folder_path = '/Volumes/SD'

# Search for Python files in the folder
python_files = glob.glob(folder_path + '/*.jpg')


#%%
savepath = '/Users/bene/Downloads/stack.tif'
nFilesPerTime = 6

iImage = []
for mFile in python_files:
    
    iImage.append(cv2.imread(mFile))
    if len(iImage)==nFilesPerTime:
        print(mFile)
        iImage = np.array(iImage)
        iImage = np.mean(np.std(iImage, 0), -1)
        #plt.imshow(iImage, cmap='gray'), plt.show()

        tif.imwrite(savepath, iImage, append=True)
        iImage = []    
    
#%%
# Convert the input images to float32
images_float = [np.float32(image) / 255.0 for image in images]

# Merge the images using exposure fusion
merge_mertens = cv2.createMergeMertens()
result = merge_mertens.process(images_float)

# Convert the result back to uint8 format
result = np.uint8(result * 255)

# Save the merged image
cv2.imwrite('/path/to/output_image.jpg', result)