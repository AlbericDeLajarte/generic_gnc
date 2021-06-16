#!/usr/bin/env python3

import rospkg

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

rospack = rospkg.RosPack()
raw_data = np.loadtxt(rospack.get_path('generic_gnc') + "/log/monte_carlo_results.txt", skiprows = 1)

rail_zenith = raw_data[:,0]
wind_angle_error = 180-raw_data[:,1]
wind_speed = raw_data[:,2]

acc_noise = raw_data[:, 3]
gyro_noise = raw_data[:,4]
baro_noise = raw_data[:,5]

wind_gust = np.row_stack((raw_data[:,6], raw_data[:,7]))

end_burn = raw_data[:,8]
used_propellant = raw_data[:,9]

apogee_error = raw_data[:,10]
kalman_error = np.row_stack((raw_data[:, 11], raw_data[:, 12], raw_data[:, 13]))

print(kalman_error[0].size)
fig, axe = plt.subplots(2,2, figsize=(10,5))

axe[0][0].hist(apogee_error, bins = 30)

axe[1][0].scatter(kalman_error[2], apogee_error)
ellipse = Ellipse(xy=(np.mean(kalman_error[2]), np.mean(apogee_error)), width=3*np.std(kalman_error[2]), height=3*np.std(apogee_error), edgecolor='r', fc='None', lw=2)
axe[1][0].add_patch(ellipse)
axe[1][0].plot(np.mean(kalman_error[2]), np.mean(apogee_error), "r+")

axe[1][1].scatter(kalman_error[0], kalman_error[1])
ellipse = Ellipse(xy=(np.mean(kalman_error[0]), np.mean(kalman_error[1])), width=3*np.std(kalman_error[0]), height=3*np.std(kalman_error[1]), edgecolor='r', fc='None', lw=2)
axe[1][1].add_patch(ellipse)
axe[1][1].plot(np.mean(kalman_error[0]), np.mean(kalman_error[1]), "r+")


axe[0][1].scatter(acc_noise, apogee_error)
plt.show()