#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 27 19:15:30 2023

@author: bene
"""
#%%
import matplotlib.pyplot as plt

from skimage import data, color, img_as_ubyte
from skimage.feature import canny
from skimage.transform import hough_ellipse
from skimage.draw import ellipse_perimeter
import NanoImagingPack as nip
import tifffile as tif
import numpy as np

from skimage.transform import rescale, resize, downscale_local_mean


realData = tif.imread('/Users/bene/Dropbox/12h42m28s_rec_FocusLockCamera Bene.tif')

# cleanup
cleanData = []
frame = realData[0,]
cleanData.append(frame)

for iFrame in realData:
    if np.mean(iFrame) != np.mean(cleanData[-1]):
        cleanData.append(iFrame)

#%%
focusValues = []
focusValues2 = []
cleanData = np.array(cleanData)
cleanData = cleanData/nip.gaussf(np.mean(cleanData, 0),20)


for i in range(len(cleanData)):
    noisy_gaussian = cleanData[i,:]
    image_gray=np.array(nip.gaussf(nip.extract(noisy_gaussian, (50,50), (59,190)),1))
    
    image_gray = rescale(image_gray, 1, anti_aliasing=False)
    edges = canny(image_gray,2) 

    focusValues2.append(np.sum(np.sum(edges,1)>0)/np.sum(np.sum(edges,0)>0))
    
    #%
    # Perform a Hough Transform
    # The accuracy corresponds to the bin size of a major axis.
    # The value is chosen in order to get a single high accumulator.
    # The threshold eliminates low accumulators
    result = hough_ellipse(edges, accuracy=10,min_size=8) #, threshold=250,min_size=0, max_size=120)
    result.sort(order='accumulator')
    
    # Estimated parameters for the ellipse
    best = list(result[-1])
    yc, xc, a, b = (int(round(x)) for x in best[1:5])
    orientation = best[5]
    
    # Draw the ellipse on the original image
    cy, cx = ellipse_perimeter(yc, xc, a, b, orientation)
    image_gray[cy, cx] = 1
    # Draw the edge (white) and the resulting ellipse (red)
    edges = color.gray2rgb(img_as_ubyte(edges))
    edges[cy, cx] = 0
    
    focusValues.append(orientation )

    plt.subplot(131)
    plt.title('Original picture')
    plt.imshow(image_gray)
    plt.subplot(132)    
    plt.title('Edge (white) and result (red)')
    plt.imshow(edges)
    plt.subplot(133)
    plt.title('Edge (white) and result (red)')
    plt.plot(focusValues)
    plt.plot(focusValues2)

    #plt.savefig("astimatism_hough"+str(i)+".png")
    plt.show()
    
plt.plot(np.unwrap(np.array(focusValues), period=np.pi/2))
plt.plot(focusValues2)