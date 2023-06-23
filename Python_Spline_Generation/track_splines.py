import yaml
from yaml.loader import BaseLoader
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import numpy as np


# Generate a parameterization of the full track using cubic splines returns splines and its coefficients
def fulltrack_splines(xCoords, yCoords, track_length, step):
    # t: variable on which the splines will be parameterized
    t = np.linspace(0, track_length, len(xCoords), endpoint=True)
    t = t[0::step]
    t = np.append(t, track_length)
    xCoords = xCoords[0::step]
    yCoords = yCoords[0::step]
    xCoords_ = np.append(xCoords, 1.0)
    yCoords_ = np.append(yCoords, 3.0)
    cs_x = CubicSpline(t, xCoords_, bc_type='periodic')
    cs_y = CubicSpline(t, yCoords_, bc_type='periodic')
    # Obtain the coefficients of cubic functions used for the splines
    coeff_x = cs_x.c
    coeff_y = cs_y.c
    return cs_x, cs_y, coeff_x, coeff_y, xCoords_, yCoords_


# Generate a parameterization of the custom track length using cubic splines
# using given horizon and returns splines and its coefficients
def curvetrack_splines(horizon_len, start, track_length):
    t = np.linspace(0, track_length, horizon_len, endpoint=True)
    cs_x = CubicSpline(t, xCoords[start: start+horizon_len], bc_type='clamped')
    cs_y = CubicSpline(t, yCoords[start: start+horizon_len], bc_type='clamped')
    # Obtain the coefficients of cubic functions used for the splines
    coeff_x = cs_x.c
    coeff_y = cs_y.c
    return cs_x, cs_y, coeff_x, coeff_y


# Generate a higher-resolution parameterization for a smoother track
def smooth_track(horizon_len, f_x, f_y, track_length):
    t_new = np.linspace(0, track_length, horizon_len, endpoint=True)
    # Evaluate the cubic splines at the new parameterization
    x_new = f_x(t_new)
    y_new = f_y(t_new)
    return x_new, y_new


# Generate the plot for the fulltrack with the points selected for splines and its smoothed version
def plot_fulltrack(xCoords, yCoords, x_new, y_new):
    # Create figure and axes
    plt.figure(1)
    plt.figure(figsize=(30, 15))
    plt.plot(xCoords, yCoords, 'bo-', label='Original Track')
    plt.plot(x_new, y_new, 'r-', label='Smoothed Track')
    plt.legend()
    plt.title('2D Race Track')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('fulltrack_spline.png')
    plt.show()


# Plot the custom length track, i.e. for a particular horizon length
def plot_smoothtrack(xCoords, yCoords, x_new_c, y_new_c):
    plt.figure(2)
    plt.figure(figsize=(30, 15))
    plt.plot(xCoords, yCoords, 'bo-', markersize=0.1, alpha=0.2, label='Original Track')
    plt.plot(x_new_c, y_new_c, 'r-', markersize=20, label='Smoothed Track')
    plt.legend()
    plt.title('2D Race Track')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    plt.savefig('custom_length_track_spline.png')
    plt.show()


# Store the respective splines related data in the yaml file
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


# Load the data from the yaml file for further use
def get_coords_from_yaml(yaml_file):
    # Load plot specification file
    centerline = yaml.load(open(yaml_file), Loader=BaseLoader)
    # Get the x and y coordinates of track from yaml file
    xCoords = np.array(centerline['track']['xCoords'])
    xCoords = xCoords.astype(float)
    yCoords = np.array(centerline['track']['yCoords'])
    yCoords = yCoords.astype(float)
    trackLength = np.array(centerline['track']['trackLength'])
    trackLength = trackLength.astype(float)
    return xCoords, yCoords, trackLength


if __name__ == '__main__':
    # Read the x and y coordinates of the reference trajectory from the yaml file
    xCoords, yCoords, track_length = get_coords_from_yaml("FREIBURG_FULL_TRACK.yaml")
    xCoords = xCoords[0:int(len(xCoords)*0.5)]
    yCoords = yCoords[0:int(len(yCoords)*0.5)]
    step_size = 100

    # Generate splines for the reference trajectory from the coordinates
    # step_size : Denotes the step of slicing operation, i.e. no. of points to be skipped between selected points
    # f_x, f_y : Represent the cubic functions of splines
    # theta_x, theta_y : Spline coefficients for x and y coordinates
    # xCoords_, yCoords_ : The sliced version of coordinates
    f_x, f_y, theta_x, theta_y, xCoords_, yCoords_ = fulltrack_splines(xCoords, yCoords, track_length, step_size)
    print(f"Total Number of coordinates points selected for generation of the splines: {len(xCoords_)}")
    print(f"Total Number of Splines generated for the coordinates: {len(theta_x[0])}")
    # Create higher resolution parameterization for the splines
    x_new, y_new = smooth_track(len(xCoords_), f_x, f_y, track_length)
    store_into_yaml(theta_x, theta_y, xCoords_, yCoords_)
    # Plot the original full track and smoothed track
    plot_fulltrack(xCoords_, yCoords_, x_new, y_new)

    # Create the splines for the shorter horizon length
    horizon = 100
    start = 950
    f_x_c, f_y_c, theta_x_c, theta_y_c = curvetrack_splines(horizon, start, track_length)
    x_new_c, y_new_c = smooth_track(horizon, f_x_c, f_y_c, track_length)

    # Plot the custom length track and smoothed track
    plot_smoothtrack(xCoords, yCoords, x_new_c, y_new_c)

    