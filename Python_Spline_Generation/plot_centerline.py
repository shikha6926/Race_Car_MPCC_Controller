
import yaml
import matplotlib.pyplot as plt
import sys
import os
from yaml.loader import BaseLoader
import numpy as np




if __name__ == '__main__':

    # Load plot specification file
    centerline = yaml.load(open("FREIBURG_FULL_TRACK.yaml"), Loader=BaseLoader)
    # Create figure and axes
    fig = plt.figure()
    # Plot the lines
    xCoords = np.array(centerline['track']['xCoords'])
    yCoords = np.array(centerline['track']['yCoords'])
    plt.scatter(xCoords.astype(float), yCoords.astype(float))
    plt.show()
