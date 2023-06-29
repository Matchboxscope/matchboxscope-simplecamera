#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 27 11:02:02 2023

@author: bene
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import NanoImagingPack as nip
import tifffile as tif 

def gaussian_2d(xy, amplitude, xo, yo, sigma_x, sigma_y, theta):
    x, y = xy
    a = (np.cos(theta)**2) / (2 * sigma_x**2) + (np.sin(theta)**2) / (2 * sigma_y**2)
    b = -(np.sin(2 * theta)) / (4 * sigma_x**2) + (np.sin(2 * theta)) / (4 * sigma_y**2)
    c = (np.sin(theta)**2) / (2 * sigma_x**2) + (np.cos(theta)**2) / (2 * sigma_y**2)
    z = amplitude * np.exp(- (a * (x - xo)**2 + 2 * b * (x - xo) * (y - yo) + c * (y - yo)**2))
    return z.ravel()

# Generate coordinates for the grid
x, y = np.meshgrid(np.linspace(-5, 5, 100), np.linspace(-5, 5, 100))
xy = np.vstack((x.flatten(), y.flatten()))

# Define parameters for the true Gaussian
true_amplitude = 1.0
true_xo, true_yo = 0.5, -0.5
true_sigma_x, true_sigma_y = 1.5, 0.8
true_theta = np.pi / 4.0
true_gaussian = gaussian_2d(xy, true_amplitude, true_xo, true_yo, true_sigma_x, true_sigma_y, true_theta)

# Add noise to the true Gaussian
noise_std = 0.1
noisy_gaussian = true_gaussian + np.random.normal(0, noise_std, true_gaussian.shape)



realData = tif.imread('/Users/bene/Dropbox/12h42m28s_rec_FocusLockCamera Bene.tif')

# cleanup
cleanData = []
frame = realData[0,]
cleanData.append(frame)

for iFrame in realData:
    if np.mean(iFrame) != np.mean(cleanData[-1]):
        cleanData.append(iFrame)
        
cleanData = np.array(cleanData)
noisy_gaussian = cleanData/nip.gaussf(np.mean(cleanData, 0),20)
noisy_gaussian = noisy_gaussian[5,:]
noisy_gaussian=np.array(nip.gaussf(nip.extract(noisy_gaussian, (100,100), (50,180)),1))
noisy_gaussian=nip.gaussf(noisy_gaussian.ravel(), 4)

centerMax = np.unravel_index(noisy_gaussian.argmax(), cleanData.shape[1:]) 

# Fit the noisy Gaussian to a 2D Gaussian function
#xy, amplitude, xo, yo, sigma_x, sigma_y, theta
initial_guess = [1.0, 0, 1, 1.1,0.1, 0]  # Initial guess for the parameters

fit_params, _ = curve_fit(gaussian_2d, xy, noisy_gaussian, p0=initial_guess)

#%% Extract the fitted parameters
initial_guess = [1.0, 0, 1, 1.1,0.1, 0]  # Initial guess for the parameters
fit_amplitude, fit_xo, fit_yo, fit_sigma_x, fit_sigma_y, fit_theta = initial_guess

# Compute the fitted Gaussian
fitted_gaussian = gaussian_2d(xy, fit_amplitude, fit_xo, fit_yo, fit_sigma_x, fit_sigma_y, fit_theta)


fitted_gaussian = gaussian_2d(xy, fit_amplitude, fit_xo, fit_yo, fit_sigma_x, fit_sigma_y, fit_theta)

# Plot the original, noisy, and fitted Gaussian
plt.figure(figsize=(12, 4))

plt.subplot(131)
plt.title("Original Gaussian")
plt.imshow(fitted_gaussian.reshape(x.shape), extent=(-5, 5, -5, 5), origin="lower")
plt.colorbar()

plt.subplot(132)
plt.title("Noisy Gaussian")
plt.imshow(noisy_gaussian.reshape(x.shape), extent=(-5, 5, -5, 5), origin="lower")
plt.colorbar()

plt.subplot(133)
plt.title("Fitted Gaussian")
plt.imshow(fitted_gaussian.reshape(x.shape), extent=(-5, 5, -5, 5), origin="lower")
plt.colorbar()

plt.tight_layout()
plt.show()
