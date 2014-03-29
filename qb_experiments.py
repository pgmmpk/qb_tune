
# run Bot along X-axis straight.
#
# each measurement is (dY, dX, leftTickCount, rightTickCount)
# where (dX, dY) are measurements taken from reference point (51.5, 0)
#
# If we model trajectory as an arc with center at (0, +-R) with angle phi, then
# R and phi can be computed from:
#
#    sin(phi) = 2*x*y / (x*x + y*y)
#    1/R = 2*y / (x*x + y*y)
#
# Distance between wheels is approx 4". This means that the distance
# each wheel is travelled can be estimated as
#
#   phi * (R - 2)    phi * (R + 2)
#
# when y is small (comparing to x)
#   phi = 2 * y / x
#   R = x * x / (2 * y)
#
# and each wheel travelling distance becomes
#
#   x - 2" * 2y/x    x + 2" * 2y / x
#
# when y is negative, left wheel path is longer than right wheel path.
#
# Lets invert and deduce x, y from lp and rp
#
#  p = 0.5 * (lp + rp) = phi * R
# dp = rp - lp         = 2L*phi
#
#  phi = dp / 2L
#  R = p / phi
#
#  x = sin(phi) * p / phi
#  y = p / phi * (1 - cos(phi))
#
#  when dp is small, phi becomes small and
#
#  x = p (1 - phi*phi / 6)
#  y = p * phi / 2
#
import math


def wheel_path(x, y, L=2):
    if abs(y) < 1.e-6 * abs(x):
        return x - 2.0 * L * y / x, x + 2.0 * L * y / x

    else:
        R = (x * x + y * y) / (2.0 * y)
        phi = math.asin(2.0 * x * y / (x * x + y * y))
        return phi * (R - L), phi * (R + L)


def position_from_odometry(lp, rp, L=2):
    p = (lp + rp) * 0.5

    phi = (rp - lp) / 2.0 / L

    if abs(phi) < 1.e-6:

        return p * (1. - phi * phi / 6.0), p * phi / 2.0

    else:
        R = p / phi
        return R * math.sin(phi), R * (1. - math.cos(phi))

# this experiments was done with the "hacked" version of quickbot_bbb package
# where i used schmitt filtering instead of thresholding
DATA_hacked = [
    (-8.5, -4.0, 46, 43),
    (-20.0, -8.5, 50, 49),
    (14.0, -3, 51, 54),
    (6, 0, 51, 48),
    (-13.5, -1.5, 56, 50),
    (-12, -3, 41, 48),
    (-4.5, -1.5, 50, 49),
    (-17, -4.5, 46, 51),
    (-11, -3, 49, 50),
    (-4.5, -1, 43, 50),
    (-16.5, -3.5, 47, 50),
    (-12, -1, 49, 50)
]

# this one uses standard software
DATA_pristine = [
    (-18.5, -7.5, 52, 54),
    (-4.5, 0, 50, 41),
    (15, -4.5, 59, 47),
    (-11.5, -1.5, 48, 41),
    (5, -2, 39, 52),
    (18, -4.5, 43, 48),
    (10, -3.5, 49, 43),
    (6, -3.5, 44, 44),
    (-9, -3.5, 49, 46),
    (19.5, -9, 42, 46),
    (18, -5, 50, 56),
    (12, -4.5, 39, 53)
]


def to_abs(data):
    for dy, dx, lcnt, rcnt in data:
        x = dx + 51.5
        y = 0.0 + dy

        lp, rp = wheel_path(x, y)
        #print x, y, position_from_odometry(lp, rp)
        yield lp, rp, lcnt, rcnt


with open('wheels_pristine.csv', 'wb') as f:
    for lp, rp, lcnt, rcnt in to_abs(DATA_pristine):

        f.write(', '.join(str(x) for x in [lp, rp, lcnt, rcnt]) + '\n')