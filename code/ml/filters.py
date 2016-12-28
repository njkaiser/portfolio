#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import numpy as np
from scipy import signal



def filt_bw(data, parameter = 1):
    # Create an order 3 lowpass butterworth filter:
    b, a = signal.butter(3, 0.1 / parameter)

    # Apply the filter to xn. Use lfilter_zi to choose the initial condition of the filter:
    zi = signal.lfilter_zi(b, a)
    z, _ = signal.lfilter(b, a, data, zi=zi*data[0])

    # Apply the filter again, to have a result filtered at an order the same as filtfilt:
    z2, _ = signal.lfilter(b, a, z, zi=zi*z[0])

    # Use filtfilt to apply the filter:
    filtered_data = signal.filtfilt(b, a, data)

    return filtered_data

    ### END OF FILT_BW()



if __name__ == '__main__':
  pass

### END OF SCRIPT
