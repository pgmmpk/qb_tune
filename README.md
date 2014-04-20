## qb_tune

Mics code from my experiments with QuickBot.

### qb_tune

This code sends "go straight" command to the robot. No controlling, no wheel encoder feedback.
It just reads and displays the encoder ticks at the end ot this (supposedly straight) move.

### qb_experiments

This is just a record of experiments taken using qb_tune code and actual robot position measured.
Then the expected number of ticks is computed (based on a simple model that assumes that any deviation
from straight move means that trajectory was an arc with constant radius). Measured number of ticks
is then compared to the expected one in a hope to find close correlation.

### qb_drive_straight

This is a simple controller that adjusts motor power according to the wheel encoder readings,
trying to balance them (i.e. making robot to go straight). This seem to work fairly well for me
when I use my version of quickbot_bbb software. Beware that I did not try the original quickbot_bbb
software, and more experiments are needed before one can make a judgement calls on the merits of my
version of quickbot_bbb as opposed to the original one.
