import numpy as np

def moving_average(data, window_size):
    window_size = 5
    smoothed_data = np.convolve(data, np.ones(window_size)/window_size, mode='valid')
    return smoothed_data
