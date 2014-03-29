"""
Run this code on the host computer. It will send motor commands to the QuickBot and
read and display ticks registered by QuickBot.
"""
import socket
import re
import contextlib
import time


class QbMaster:
    """
    Takes care of the communication protocol details. Presents to the user the easy-to-use
    robot control interface.
    """

    BUFFER_SIZE = 1024

    def __init__(self, robot_addr, base_addr):
        self.robot_addr = robot_addr
        self.base_addr = base_addr
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__socket.bind(self.base_addr)
        self.__socket.settimeout(2.0)

    def close(self):
        self.__socket.close()

    def send_recv(self, message, expect_reply=True):
        self.__socket.sendto(message, self.robot_addr)

        if not expect_reply:
            return

        reply, _ = self.__socket.recvfrom(QbMaster.BUFFER_SIZE)
        return reply

    def cmd_reset(self):
        self.send_recv("$RESET*\n", False)

    def cmd_check(self):
        reply = self.send_recv("$CHECK*\n")
        print repr(reply)
        return  reply.startswith("Hello from QuickBot")

    def cmd_set_pwm(self, left_val, right_val):
        self.send_recv("$PWM=%s,%s*\n" % (left_val, right_val), False)

    def cmd_get_pwm(self):
        reply = self.send_recv("$PWM?*\n")
        return parse_tuple(reply)

    def cmd_get_enval(self):
        reply = self.send_recv("$ENVAL?*\n")
        return parse_tuple(reply)

    def cmd_get_envel(self):
        reply = self.send_recv("$ENVEL?*\n")
        return parse_tuple(reply)

    @classmethod
    @contextlib.contextmanager
    def connect(cls, robot_addr, base_addr):

        master = QbMaster(robot_addr, base_addr)

        try:
            yield master

        finally:
            master.close()


def parse_tuple(reply):
    m = re.match(r'\[(-?[\d\.]+)\s*,\s*(\-?[\d\.]+)\]\s*', reply)
    if not m:
        raise Exception('Unexpected reply:' + reply)

    return float(m.group(1)), float(m.group(2))


def animate(from_, to, duration, samples=10):
    dt = duration / (samples - 1)
    dx = (to - from_) / (samples - 1)

    yield from_

    for _ in range(samples):
        time.sleep(dt)
        from_ += dx
        yield from_


if __name__ == '__main__':

    # change the addresses following according to your setup
    ROBOT_ADDR = ('192.168.0.8', 5005)
    BASE_ADDR = ('192.168.0.6', 5005)

    with QbMaster.connect(ROBOT_ADDR, BASE_ADDR) as master:

        # check
        if not master.cmd_check():
            raise Exception('Failed to connect')

        master.cmd_reset()

        for power in animate(30, 100, 0.5):
            master.cmd_set_pwm(power, power)

        time.sleep(1.5)
        print master.cmd_get_envel()

        for power in animate(100, 30, 0.5):
            master.cmd_set_pwm(power, power)

        master.cmd_set_pwm(0, 0)

        time.sleep(0.5)

        print master.cmd_get_enval()
