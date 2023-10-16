#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


MAX_OBSTACLES = 25                      # maximum number of obstacles 
num_obstacles = 13                      # number of obstacles 
obstacle = [
    [0.61, 2.743],[0.915, 2.743],[1.219, 2.743],[1.829, 1.219],
    [1.829, 1.524],[ 1.829, 1.829], [1.829, 2.134],[2.743, 0.305],
    [2.743, 0.61],[2.743, 0.915],[2.743, 2.743],[3.048, 2.743],
    [3.353, 2.743],
    [-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],
    [-1,-1],[-1,-1],[-1,-1]
]

start = [0.305, 1.219] # start location
goal = [3.658, 1.829]  # goal location

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)


# Write your program here.
# Run the motor up to 500 degrees per second. To a target angle of 90 degrees
left_motor.run_target(500, 90)
right_motor.run_target(500, 90)

#ev3.speaker.beep()
