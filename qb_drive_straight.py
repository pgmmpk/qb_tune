"""
PID controller for going STRAIGHT
(ticks-based, no use of velocity)

Observations:
t_1, t_2

Goal is to maintain t_1 - t_2 = 0 while going at constant speed

Model is: motor power is proportional to the wheel speed

t_1' = u_1
t_2' = u_2

we will be controlling the motor power differential aiming at getting zero
difference between t_1 and t_2

deltaU = C_1 * delta_t + C_2 * delta_t' + C_3 * \int delta_t

"""
import socket
import re
import contextlib
import time
from qb_tune import QbMaster


class PID:
    """
    Simple PID controller
    """

    def __init__(self, c_p, c_i, c_d):
        self.c_p = c_p
        self.c_i = c_i
        self.c_d = c_d

        self._prev = None
        self._integral = 0.0

    def feed(self, value):
        self._integral += value

        response = self.c_p * value + self.c_i * self._integral
        if self._prev is not None:
            response += self.c_d * (value - self._prev)
        self._prev = value

        return response

if __name__ == '__main__':

    # change the following to match your setup
    ROBOT_ADDR = ('192.168.0.8', 5005)
    BASE_ADDR = ('192.168.0.6', 5005)

    # these params seem to be good to make robot go straight
    pid = PID(0.5, 0.01, 0.05)
    power = 60
    turn = 0  # change this if you want robot to turn then go straight (unit is ticks)

    with QbMaster.connect(ROBOT_ADDR, BASE_ADDR) as master:

        # check robot is ready
        if not master.cmd_check():
            raise Exception('Failed to connect')

        master.cmd_reset()

        for _ in range(150):
            time.sleep(0.005)
            t1, t2 = master.cmd_get_enval()
            delta_pwm = pid.feed(t2 - t1 - turn)

            # clip pwm to its range
            pwm_1 = int(min(100, max(0, power + delta_pwm)))
            pwm_2 = int(min(100, max(0, power - delta_pwm)))

            master.cmd_set_pwm(pwm_1, pwm_2)
            print '%7d %7d %7d %7d %7d' % ( (t2-t1), t1, t2, pwm_1, pwm_2)

        # stop motors
        master.cmd_set_pwm(0, 0)

        # keep printing ticks while robot is stopping...
        for _ in range(10):
            time.sleep(0.05)
            t1, t2 = master.cmd_get_enval()
            print '%7d %7d %7d' % ( (t2-t1), t1, t2)
