#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

SAMPLE_RATIO = 400

file_teleop_data = './src/freedom_navigation/data/teleop_data.txt'
file_following_data = './src/freedom_navigation/data/following_data.txt'

length_teleop_data = round(sum(1 for _ in open(file_teleop_data)) / 50, 0)
length_following_data = sum(1 for _ in open(file_following_data))

with open(file_teleop_data, 'r') as f:
    x_teleop_data = []
    y_teleop_data = []
    length = sum(1 for _ in open(file_teleop_data))
    for index, line in enumerate(f):
        if index % SAMPLE_RATIO == 0:
            x0, y0 = line.split(',')
            x_teleop_data.append(float(x0))
            y_teleop_data.append(float(y0))
    x_teleop_data= np.array(x_teleop_data)
    y_teleop_data= np.array(y_teleop_data)
    points = np.vstack((x_teleop_data, y_teleop_data)).T
    print("Points: ", points)

ratio_lengths = round(length_following_data / length_teleop_data, 0)

with open(file_following_data, 'r') as f:    
    x_following_data = []
    y_following_data = []
    for index, line in enumerate(f):
        if index % ratio_lengths == 0:
            x0, y0 = line.split(',')
            x_following_data.append(float(x0))
            y_following_data.append(float(y0))
    x_following_data = np.array(x_following_data)
    y_following_data = np.array(y_following_data)

plt.title('Comparação entre curva gravado e curva executada')
plt.plot(x_teleop_data, y_teleop_data, label='Teleop_data')
plt.plot(x_following_data, y_following_data, label='Following_data')
plt.legend()
plt.show()