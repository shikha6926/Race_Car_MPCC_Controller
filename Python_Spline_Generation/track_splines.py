# -*- coding: utf-8 -*-
"""
Created on Wed May 10 14:37:57 2023

@author: Ankita
"""

# -*- coding: utf-8 -*-
"""
Created on Tue May  9 20:40:39 2023

@author: Ankita
"""

import yaml
from yaml.loader import BaseLoader
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import numpy as np


# Generate a parameterization of the full track using cubic splines using given
# horizon and returns splines and its coefficients
def fulltrack_splines(xCoords, yCoords):
    t = np.linspace(0, 2*13.2799, len(xCoords), endpoint=True)
    t = t[1::100]
    xCoords = xCoords[1::100]
    yCoords = yCoords[1::100]
    # for x in range(len(xCoords)):
    #     print(f"{xCoords[x]}, {t[x]}")
    cs_x = CubicSpline(t, xCoords, bc_type='not-a-knot')
    cs_y = CubicSpline(t, yCoords, bc_type='not-a-knot')
    # Obtain the coefficients of cubic functions used for the splines
    coeff_x = cs_x.c
    coeff_y = cs_y.c
    return cs_x, cs_y, coeff_x, coeff_y, xCoords, yCoords


# Generate a parameterization of the custom track length using cubic splines
# using given horizon and returns splines and its coefficients
def curvetrack_splines(horizon, start):
    t = np.linspace(0, 13.2799, horizon, endpoint=True)
    cs_x = CubicSpline(t, xCoords[start: start+horizon], bc_type='clamped')
    cs_y = CubicSpline(t, yCoords[start: start+horizon], bc_type='clamped')
    # Obtain the coefficients of cubic functions used for the splines
    coeff_x = cs_x.c
    coeff_y = cs_y.c
    return cs_x, cs_y, coeff_x, coeff_y


# Generate a higher-resolution parameterization for a smoother track
def smooth_track(horizon, f_x, f_y):
    t_new = np.linspace(0, 13.2799, horizon, endpoint=True)
    # Evaluate the cubic splines at the new parameterization
    x_new = f_x(t_new)
    y_new = f_y(t_new)
    return x_new, y_new

def plot_fulltrack(xCoords, yCoords, x_new, y_new):
    # Create figure and axes
    fig = plt.figure(1)
    plt.figure(figsize=(30, 15))
    plt.plot(xCoords[1::100], yCoords[1::100], 'bo-', label='Original Track')
    plt.plot(x_new, y_new, 'r-', label='Smoothed Track')
    plt.legend()
    plt.title('2D Race Track')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('fulltrack_spline.png')
    plt.show()

def plot_smoothtrack(xCoords, yCoords, x_new_c, y_new_c):
    fig2 = plt.figure(2)
    plt.figure(figsize=(30, 15))
    plt.plot(xCoords, yCoords, 'bo-', label='Original Track')
    plt.plot(x_new_c, y_new_c, 'r-', label='Smoothed Track')
    plt.legend()
    plt.title('2D Race Track')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('custom_length_track_spline.png')
    plt.show()

def store_into_yaml(theta_x, theta_y, xCoords, yCoords):
    # Store coefficients of splines in yaml file
    x_coef = {'X_a': theta_x[3].tolist(), 'X_b': theta_x[2].tolist(),
              'X_c': theta_x[1].tolist(), 'X_d': theta_x[0].tolist()}

    y_coef = {'Y_a': theta_y[3].tolist(), 'Y_b': theta_y[2].tolist(),
              'Y_c': theta_y[1].tolist(), 'Y_d': theta_y[0].tolist()}

    X_coords = {'X_coords': xCoords.tolist()}
    Y_coords = {'Y_coords': yCoords.tolist()}

    with open('fulltrack_spline.yaml', 'w') as file:
        yaml.dump(x_coef, file, sort_keys=False)
        yaml.dump(y_coef, file, sort_keys=False)
        yaml.dump(X_coords, file, sort_keys=False)
        yaml.dump(Y_coords, file, sort_keys=False)
    # with open('fulltrack_spline_ycoeff.yaml', 'w') as file:
    #     yaml.dump(y_coef, file, sort_keys=False)

def get_coords_from_yaml(yaml_file):
    # Load plot specification file
    centerline = yaml.load(open(yaml_file), Loader=BaseLoader)
    # Get the x and y coordinates of track from yaml file
    xCoords = np.array(centerline['track']['xCoords'])
    xCoords = xCoords.astype(float)
    yCoords = np.array(centerline['track']['yCoords'])
    yCoords = yCoords.astype(float)

    return xCoords, yCoords


if __name__ == '__main__':

    xCoords, yCoords = get_coords_from_yaml("FREIBURG_FULL_TRACK.yaml")
    # one_lap_index = int(len(xCoords)/2)
    # xCoords = xCoords[:one_lap_index]
    # yCoords = yCoords[:one_lap_index]
    horizon = 40
    start = 950
    
    f_x, f_y, theta_x, theta_y, xCoords_, yCoords_ = fulltrack_splines(xCoords, yCoords)
    x_new, y_new = smooth_track(len(xCoords), f_x, f_y)
    print(f"Length of coef: {len(theta_x[0])}")
    store_into_yaml(theta_x, theta_y, xCoords_, yCoords_)
    for i in range(len(theta_y[2])):
        print(f"{theta_y[3][i]},")
    # Print the cubic functions for each spline segment
    # for i, (s_x, s_y) in enumerate(zip(splines_x, splines_y),):
    # for i in range(horizon-1):
    #     print(f"Spline segment {i+1}:")
    #     print(f"Cubic function for x:{theta_x[0][i]:.3f} + {theta_x[1][i]:.3f}*(t-{i}) + {theta_x[2][i]:.3f}*(t-{i})^2 + {theta_x[3][i]:.3f}*(t-{i})^3")
    #     print(f"Cubic function for y:{theta_y[0][i]:.3f} + {theta_y[1][i]:.3f}*(t-{i}) + {theta_y[2][i]:.3f}*(t-{i})^2 + {theta_y[3][i]:.3f}*(t-{i})^3")

    # Plot the original full track and smoothed full track
    # Create figure and axes
    plot_fulltrack(xCoords, yCoords, x_new, y_new)
    
    f_x_c, f_y_c, theta_x_c, theta_y_c = curvetrack_splines(horizon, start)
    x_new_c, y_new_c = smooth_track(horizon, f_x_c, f_y_c)

    # Plot the custom length track and smoothed track
    plot_smoothtrack(xCoords, yCoords, x_new_c, y_new_c)

    