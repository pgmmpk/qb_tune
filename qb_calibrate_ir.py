"""
Calibration of IR sensors.

This code just collects 100 samples (about 5 seconds) from each IR sensor and plots
these measurements on a chart.

Adding obstacles around bot at fixed distance from sensors will allow one to calibrate the
sensor readings...

"""
import socket
import re
import contextlib
import time
from qb_tune import QbMaster

if __name__ == '__main__':

    # change the following to match your setup
    ROBOT_ADDR = ('192.168.0.5', 5005)
    BASE_ADDR = ('192.168.0.6', 5005)

    WHICH = 0  # which IR sensor we calibrate

    with QbMaster.connect(ROBOT_ADDR, BASE_ADDR) as master:

        # check robot is ready
        if not master.cmd_check():
            raise Exception('Failed to connect')

        master.cmd_reset()

        measurements = []
        for _ in range(100):
            time.sleep(0.05)
            vals = master.cmd_get_irval()
            measurements.append(vals)

    import matplotlib.pyplot as plt

    plt.plot([vals[0] for vals in measurements], 'r')
    plt.plot([vals[1] for vals in measurements], 'g')
    plt.plot([vals[2] for vals in measurements], 'b')
    plt.plot([vals[3] for vals in measurements], 'm')
    plt.plot([vals[4] for vals in measurements], 'c')
    plt.ylim([0, 750])
    plt.show()
