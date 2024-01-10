import os
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import tifffile as tif

import csv

try:
    pixelSize = 2
    allDrifts = []
    driftFile = '/Users/bene/Downloads/2023_11_30-Anglerfish_LED_Timelapse_Decay/Drift.csv'
    
    csvFile = open(driftFile)
    csvreader = csv.reader(csvFile)
    rows = []
    for row in csvreader:
        try:allDrifts.append(float(row[1]))
        except:pass
    allDrifts = np.array(allDrifts)
    csvFile.close()
except:
    pass




def main():
    mStack = []
    # Assuming your folders are named "0", "1", "2", ...
    if 1:
        mFolder = '/Users/bene/Downloads/2023_11_30-Anglerfish_LED_Timelapse_Decay/'
        mFilename = 'data_Aug_24_2023_160719_timelapse_image_anglerfish'
        nFolders = 1929
        iterators = []  # To store iterators (i from anglerfish_i)
        # To store mean intensities
        exposures = [1, 5, 10, 50, 100, 500]  # List of your exposures
        nExposures = len(exposures)
        mValues = np.zeros((nFolders, nExposures))
        
        for folder in range(nFolders):
            # Construct the file path
            file_template = f"{mFilename}_{folder}_z0_texp_"
            
            mExposures = []
            print(file_template)
            mSubStack = []
            mSubMean = []
            for index, exp in enumerate(exposures):
                file_name = f"{folder}/{file_template}{exp}.jpg"
                file_path = os.path.join(mFolder, file_name)
            
                if os.path.exists(file_path):
                    img = Image.open(file_path).convert('L')  # Convert to grayscale
                    # Convert to numpy array and calculate mean intensity
                    mean_intensity = np.array(img).mean()
                    mSubStack.append(img)
                    mSubMean.append(mean_intensity)
                    if mean_intensity == 0:
                        break
                    mValues[folder, index]=mean_intensity

            
            mImage = mSubStack[np.argmin(np.abs(np.array(mSubMean)-127))]
            try:
                tif.imsave("stack.tif", mImage, append=True)
            except:
                pass
        np.save('dataset.npy',mValues )
    else:
        exposures = [1, 5, 10, 50, 100, 500]  # List of your exposures
        mValues = np.load('dataset.npy')
    font = {'family': 'serif',
            'color':  'darkred',
            'weight': 'normal',
            'size': 16,
            }
    

    # Creating a more refined and publication-quality plot
    mTimeStamps = np.linspace(0,mValues.shape[0]-1,mValues.shape[0])*120 #s 
    
    fig, ax1 = plt.subplots()
    for i, val in enumerate(exposures):
        print("Adding ")
        ax1.plot(mTimeStamps/60/60, mValues[:,i], alpha=0.7)
    ax1.set_xlabel('Time [h]', fontsize=14)
    ax1.set_ylabel('Mean Intensity [a.u.]', fontsize=14)
    ax1.legend(exposures, fontsize=12)
    plt.title('Mean Intensity per Image', fontsize=16)
    ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
    #ax1.xticks(fontsize=12)
    #ax1.yticks(fontsize=12)
    
    ax2 = ax1.twinx()
    mTimeStamps2 = np.linspace(0,np.max(mTimeStamps/60/60),allDrifts.shape[0])
    ax2.plot(mTimeStamps2,allDrifts, "black")
    ax2.legend("Drift [µm]", fontsize=12, loc='upper left')
    ax2.set_ylabel(("Drift"))
    plt.savefig("graph_meanintensityovertimebatteryled.png")


if __name__ == "__main__":
    main()
