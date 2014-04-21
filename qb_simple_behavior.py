"""
Simple behavior, contains three controllers:

 1. Go Straight - this controller tries to keep tick difference between right and left wheel constant, i.e. make
    bot go as straight as possible.

 2. Avoid Obstacle - this controller just stops, then backtracks a little (i.e. tries to compensate for
    bot inertia)

 3. Find New Direction - makes bot rotate on the spot, looking for a clear direction.

Bot goes straight until it hits an obstacle. It then brakes and backtracks. Then it star rotating, looking for
a direction where there is no obstacles. When found, it will run straight.
"""
import socket
import re
import contextlib
import time
from qb_tune import QbMaster
import random


class Signal:
    """
    Utility class: implements observer pattern. Interested parties can register their listeners and
    broadcast events.
    """

    def __init__(self):
        self._listeners = []

    def emit(self, *av, **kav):
        for l in self._listeners:
            l(*av, **kav)

    def connect(self, listener):
        self._listeners.append(listener)

    def disconnect(self, listener):
        self._listeners.remove(listener)


class GoStraightController:
    """
    Controller that tries to maintain a fixed difference between left wheel ticks and right wheel ticks.
    This makes bot go straight.

    Emits |obstacle| signal when detects an obstacle.
    """

    obstacle = Signal()

    def __init__(self, Kp=1.0, Ki=0.03, Kd=0.0, power=70, turn=0):
        self._pid = PID(Kp, Ki, Kd)
        self._power = power
        self._turn = turn
        self._timer = 10

    def execute(self, t1, t2, irval):
        self._timer += 1

        if irval[2] > 100.0:  # head-on obstacle!
            self.obstacle.emit()
            return 0, 0  # stop!

        if self._timer == 1:
            self._turn = t2 - t1

        delta_pwm = self._pid.feed(t2 - t1 - self._turn)
        return self._power + delta_pwm, self._power - delta_pwm

    def reset(self):
        self._timer = 0


class AvoidCollisionController:
    """
    Measures how far bot goes after it was commanded to stop, then backtracks to compensate for this
    overshoot.

    When finished, emits |backtracked| signal.
    """

    backtracked = Signal()

    def __init__(self):
        self._timer = 0
        self._t1 = 0
        self._t2 = 0

    def reset(self):
        self._timer = 0

    def execute(self, t1, t2, irval):
        # backtrack to the tick position
        self._timer += 1
        if self._timer == 1:
            self._t1 = t1
            self._t2 = t2

        if self._timer < 10:
            return 0, 0

        # measure how much we've overshoot and backtrack
        if self._timer == 10:
            delta1, delta2 = t1 - self._t1, t2 - self._t2
            print 'Overshot by:', delta1, delta2
            self._t1 = t1 + delta1
            self._t2 = t2 + delta2

        pwm_1 = -70 if t1 < self._t1 else 0
        pwm_2 = -70 if t2 < self._t2 else 0

        if pwm_1 == 0 and pwm_2 == 0:
            self.backtracked.emit()

        return pwm_1, pwm_2


class FindNewDirectionController:
    """
    Rotates bot, looking for a direction where there is no obstacles. Whn found, emits |no_obstacle| signal.
    """

    no_obstacle = Signal()

    def __init__(self):
        self._timer = 0
        self._direction = 0

    def execute(self, t1, t2, irval):
        self._timer += 1

        if self._timer < 10:
            if irval[2] < 50:
                self.no_obstacle.emit()
            return 0, 0

        if self._timer > 13:
            self._timer = 0
            #self._direction = random.random() > 0.5
            return 0, 0

        if self._direction:
            return 85, -85
        else:
            return -85, 85

    def reset(self):
        self._timer = 0


class Supervisor:
    """
    Implements behavior by utilizing three controllers and managing transitions between these controllers.
    """

    def __init__(self):

        self._go_straight = GoStraightController()
        self._avoid_collision = AvoidCollisionController()
        self._find_new_direction = FindNewDirectionController()
        self.current = self._go_straight

        self._go_straight.obstacle.connect(self.on_obstacle)
        self._avoid_collision.backtracked.connect(self.on_backtracked)
        self._find_new_direction.no_obstacle.connect(self.on_no_obstacle)

    def execute(self, t1, t2, irval):
        return self.current.execute(t1, t2, irval)

    def on_obstacle(self):
        self.current = self._avoid_collision
        self.current.reset()

    def on_backtracked(self):
        self.current = self._find_new_direction

    def on_no_obstacle(self):
        self.current = self._go_straight
        self.current.reset()


if __name__ == '__main__':

    # change the following to match your setup
    ROBOT_ADDR = ('192.168.0.5', 5005)
    BASE_ADDR = ('192.168.0.6', 5005)

    supervisor = Supervisor()

    with QbMaster.connect(ROBOT_ADDR, BASE_ADDR) as master:

        # check robot is ready
        if not master.cmd_check():
            raise Exception('Failed to connect')

        master.cmd_reset()

        for _ in range(1500):
            time.sleep(0.05)
            irval = master.cmd_get_irval()
            t1, t2 = master.cmd_get_enval()

            pwm_1, pwm_2 = supervisor.execute(t1, t2, irval)

            # clip pwm to its range
            pwm_1 = int(min(100, max(-100, pwm_1)))
            pwm_2 = int(min(100, max(-100, pwm_2)))

            print pwm_1, pwm_2
            master.cmd_set_pwm(pwm_1, pwm_2)
            print '%7d %7d %7d %7d %7d' % ( (t2-t1), t1, t2, pwm_1, pwm_2)
            print '\t\t\t', irval

        # stop motors
        master.cmd_set_pwm(0, 0)

