#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 26 16:03:35 2023

@author: bene
"""
import NanoImagingPack as nip
import numpy as np
from importlib import reload
import matplotlib.pyplot as plt
import tifffile as tif

psfparams = nip.PSF_PARAMS()
psfparams.aberration_zernikes
obj3d = nip.readim("MITO_SIM")

aber_map = nip.xx(obj3d.shape[-2:]).normalize(1);  # Define some aberration map (x-ramp in this case)
psfparams.aberration_types = [psfparams.aberration_zernikes.astigm]  # define list of aberrations (select from choice, zernike coeffs, or aberration map
psfparams.aberration_strength = [1];



obj3d=nip.extract(obj3d,[100,100,100])
obj3d.pixelsize=[50,50,50]
h3 = nip.psf(obj3d, psfparams)

for iStack in range(h3.shape[0]):
    h3[iStack,] = (nip.rot2d(h3[iStack,], 40, padding=0))


plt.imshow(h3[40,])
tif.imsave("astigmatism.tif", h3)


#%% load stack
realData = tif.imread('/Users/bene/Dropbox/12h42m28s_rec_FocusLockCamera Bene.tif')

# cleanup
cleanData = []
frame = realData[0,]
cleanData.append(frame)

for iFrame in realData:
    if np.mean(iFrame) != np.mean(cleanData[-1]):
        cleanData.append(iFrame)
        
cleanData = np.array(cleanData)

#%% correlate data
import scipy
focusvalue=[]
for i in range(30,70,1):
    frame = nip.gaussf(cleanData[0],0)
    frame = nip.extract(frame, (100,100), (50,180))
    astigmatism = nip.gaussf(h3[i,],2)
    mCorrelation = scipy.signal.correlate(frame,astigmatism)
    
    
    focusvalue.append(np.max(mCorrelation))
    continue

    
    plt.subplot(131)
    plt.title("Realdata")
    plt.imshow(frame)
    plt.subplot(132)
    plt.title("Simulated PSF")
    plt.imshow(astigmatism)
    plt.subplot(133)
    plt.title("Calculated CC \n" + str(focusvalue[-1]))
    plt.imshow(mCorrelation)
    plt.savefig("cc"+str(i)+"png")
    plt.show()


plt.plot(focusvalue)
#%% iterate over real data
bestFocus = []

for iFrame in cleanData:
    focusvalue=[]
    frame = nip.gaussf(iFrame,0)
    frame = nip.extract(frame, (100,100), (50,180))
    for i in range(30,70,1):
        astigmatism = nip.gaussf(h3[i,],2)
        mCorrelation = scipy.signal.correlate(frame,astigmatism)
        
        focusvalue.append(np.mean(mCorrelation))
    bestFocus.append(np.max(np.array(focusvalue)))
    bestAstigmatism = h3[np.argmax(np.array(focusvalue)),:,:]
    

    
    plt.subplot(131)
    plt.title("Realdata")
    plt.imshow(frame)
    plt.subplot(132)
    plt.title("Simulated PSF")
    plt.imshow(bestAstigmatism)
    plt.subplot(133)
    plt.title("Calculated CC \n" + str(bestFocus[-1]))
    plt.imshow(mCorrelation)
    plt.savefig("cc"+str(i)+"png")
    plt.show()


plt.plot(bestFocus)

#%%
focusvalues=[]
cleanData[,10]
for iFrame in cleanData:
    iFrame = nip.gaussf(iFrame,5)
    focusValue = np.sum(np.mean(iFrame,1)/np.mean(iFrame)>1.05)/np.sum(np.mean(iFrame,0)/np.mean(iFrame)>1.05)
    #(np.mean(iFrame,0))/np.std(np.mean(iFrame, 1)))
    focusvalues.append(focusValue)
    
plt.plot(focusvalues)

#%%
import numpy as np
from astropy.modeling import models, fitting
from skimage import feature
import matplotlib.pyplot as plt
from skimage.draw import ellipse
import imageio

# Step 1: Preprocess the image to identify points that belong to the ellipse
image = cleanData[9,]
image  = nip.extract(image , (100,100), (50,180))
image = nip.gaussf(image ,5)

# Edge detection (you might need to adjust the sigma)
edges = feature.canny(image, sigma=11)
plt.imshow(edges)
# Get the x and y coordinates of the edge points
y, x = np.nonzero(edges)

# Step 2: Use these points to initialize the Ellipse2D model
# The initial parameters are the mean x and y coordinates and initial guesses for the semimajor and semiminor axes
init = models.Ellipse2D(amplitude = 1, x_0=0, y_0=0, a=1, b=1, theta=0.)

# Step 3: Perform a least squares fit to find the optimal parameters
fit = fitting.LevMarLSQFitter()

x_grid, y_grid = np.meshgrid(np.arange(image.shape[1]), np.arange(image.shape[0]))  # Create a grid of x, y coordinates
ellipse = fit(init, x_grid, y_grid, edges)  # Pass the grid of coordinates and edges into the fitter


# Print the parameters of the best-fit ellipse
print(ellipse)

# Create a new Ellipse2D model using the parameters from the fitted ellipse
fitted_ellipse = models.Ellipse2D(amplitude=ellipse.amplitude.value, x_0=ellipse.x_0.value, y_0=ellipse.y_0.value, a=ellipse.a.value, b=ellipse.b.value, theta=ellipse.theta.value)

# Evaluate the fitted ellipse over a grid of points
y_grid, x_grid = np.mgrid[:image.shape[0], :image.shape[1]]
fitted_image = fitted_ellipse(x_grid, y_grid)

# Now we can visualize the original points and the fitted ellipse
plt.imshow(image, cmap='gray')
plt.contour(fitted_image, levels=[0.5], colors='r')
plt.scatter(x, y, s=1)
plt.show()