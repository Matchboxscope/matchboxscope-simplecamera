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
    h3[iStack,] = (nip.rot2d(h3[iStack,], 45, padding=0))


plt.imshow(h3[50,])
tif.imsave("astigmatism.tif", h3)

