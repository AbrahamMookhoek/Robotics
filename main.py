#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog


start = [0.305, 1.219] # start location
goal = [3.658, 1.829]  # goal location

MAX_OBSTACLES = 25     # maximum number of obstacles 
num_obstacles = 13     # number of obstacles 
obstacle_indices = []
obstacle = [
[0.61, 2.743],[0.915, 2.743],[1.219, 2.743],[1.829, 1.219],
[1.829, 1.524],[ 1.829, 1.829], [1.829, 2.134],[2.743, 0.305],
[2.743, 0.61],[2.743, 0.915],[2.743, 2.743],[3.048, 2.743],
[3.353, 2.743],
[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],
[-1,-1],[-1,-1],[-1,-1]
]

class Node:
    def __init__(self, index, parent):
        self.index = index
        self.parent = parent

    def expand(self):
        childList = []

        if(self.index[0] == 0):
            childList.append(Node([1,self.index[1]], self))
        elif(self.index[0] == 15):
            childList.append(Node([14,self.index[1]], self))
        elif(self.index[0] in range(1,15)):
            childList.append(Node([self.index[0]+1,self.index[1]], self))
            childList.append(Node([self.index[0]-1,self.index[1]], self))
        
        if(self.index[1] == 0):
            childList.append(Node([self.index[0],1], self))
        elif(self.index[1] == 9):
            childList.append(Node([self.index[0],8], self))
        elif(self.index[1] in range(1,9)):
            childList.append(Node([self.index[0],self.index[1]+1], self))
            childList.append(Node([self.index[0],self.index[1]-1], self))
        
        return childList

def graph_search():
    fringe = [Node([round(start[0]/0.305),round(start[1]/0.305)], None)]
    closed = []

    while(len(fringe) > 0):
        currentNode = fringe.pop()
 
        if(currentNode.index not in obstacle_indices):
            if(currentNode.index not in closed):
                closed.append(currentNode.index)

                if(currentNode.index == [round(goal[0]/0.305),round(goal[1]/0.305)]):
                    return currentNode
                
                childList = currentNode.expand()
                for child in childList:
                    fringe.insert(0,child)
    
    return Node(start, None)
            

def main():
    # This program requires LEGO EV3 MicroPython v2.0 or higher.
    # Click "Open user guide" on the EV3 extension tab for more information.

    workspace = [[0]*16]*10

    for y in range(0,10):
        for x in range(0,16):
            for index in range(0,num_obstacles):
                if( (x == round((obstacle[index][0])/0.305)) and (y == round((obstacle[index][1])/0.305)) ):
                    obstacle_indices.append([round((obstacle[index][0])/0.305),round((obstacle[index][1])/0.305)])
                    obstacle_indices.append([round((obstacle[index][0])/0.305)-1,round((obstacle[index][1])/0.305)])
                    obstacle_indices.append([round((obstacle[index][0])/0.305),round((obstacle[index][1])/0.305)-1])
                    obstacle_indices.append([round((obstacle[index][0])/0.305)-1,round((obstacle[index][1])/0.305)-1])
                    workspace[round((obstacle[index][1])/0.305)][round((obstacle[index][0])/0.305)] = 1000
                    workspace[round((obstacle[index][1])/0.305)][round((obstacle[index][0])/0.305)-1] = 1000
                    workspace[round((obstacle[index][1])/0.305)-1][round((obstacle[index][0])/0.305)] = 1000
                    workspace[round((obstacle[index][1])/0.305)-1][round((obstacle[index][0])/0.305)-1] = 1000
                else:
                    workspace[y][x] = (abs(x - goal[0]) + abs(y - goal[1]))

    goalNode = graph_search()
    found_path = []

    while(goalNode.parent != None):
        found_path.insert(0,goalNode.index)
        goalNode = goalNode.parent
    found_path.insert(0,goalNode.index)

    for item in found_path:
        print(item)

    # Create your objects here.
    ev3 = EV3Brick()
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.D)
    gyro_sensor = GyroSensor(Port.S4)

    # Resetting the angle to 0 for the gyro sensor
    gyro_sensor.reset_angle(0)
    # Can check if the gyro_sensor.angle() and .speed() is zero before going forward which could be useful.

    # Turning left
    #while gyro_sensor.angle() != -50:
    #    print("Gyro Sensor Angle " + str(gyro_sensor.angle()))
    #    print("Right Motor Angle " + str(right_motor.angle()))
    #    right_motor.run(100)
    #right_motor.stop()

    # Turning right
    #while gyro_sensor.angle() != 50:
    #    print("Gyro Sensor Angle " + str(gyro_sensor.angle()))
    #    print("Left Motor Angle " + str(left_motor.angle()))
    #    left_motor.run(135)
    #left_motor.stop()

    #left_motor.run_angle(200, 260, wait=True)
    #right_motor.run_angle(200, -260, wait=True)

    # while True:
    #     while gyro_sensor.angle() > -2:
    #         print(gyro_sensor.angle())
    #         right_motor.run(350)
    #     right_motor.stop()


    #     while gyro_sensor.angle() < 2:
    #         print(gyro_sensor.angle())
    #         left_motor.run(180)
    #     left_motor.stop()


    print(gyro_sensor.angle())
    #right_motor.run_angle(1000, 360, wait=False)
    #left_motor.run_angle(500, 360, wait=True)

    # angle of 123 produces a 90 turn
    #robot.turn(123)

    # angle of 243 produces a 180 turn
    #robot.turn(243)

    # gyro_offset = 0
    # while True:
    #     gyro_minimum_rate, gyro_maximum_rate = 440, -440
    #     gyro_sum = 0
    #     for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
    #         gyro_sensor_value = gyro_sensor.speed()
    #         gyro_sum += gyro_sensor_value
    #         if gyro_sensor_value > gyro_maximum_rate:
    #             gyro_maximum_rate = gyro_sensor_value
    #         if gyro_sensor_value < gyro_minimum_rate:
    #             gyro_minimum_rate = gyro_sensor_value
    #         wait(5)
    #     if gyro_maximum_rate - gyro_minimum_rate < 2:
    #         break
    # gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT


    # Change the motor depending on the angle of the gyroscope to move in a straight line.
    # Create a loop that will loop until target distance is reached by the robot (Must determine if the sum of the motor distance is the distance traveled)
    # Each motor will need to rotate a determine amount then switch of and the other motor will rotate the same amount.
    # This will continue until the robot as traveled the desired distance.

    # MEASURING
    # speed() - Gets the speed of the motor
    # angle() - Gets the rotation angle of the motor
    # reset_angle(angle) - Sets the accumulated rotation angle of the motor to a desired value (deg)

    # STOPPING
    # stop() - Stops the motor and lets it spin freely
    # brake() - Passively brakes the motor
    # hold() - Stops the motor and actively holds it at its current angle

    # ACTION
    # run(speed) - Runs the motor at a constant speed (deg/s)
    # run_time(speed,time,then=Stop.HOLD, wait=True) - Runs the motor at a constant speed for a given amount of time
    # run_angle(speed,rotation_angle,then=Stop.HOLD, wait=True) - Runs the motor at a constant speed by a given angle
    # run_target(speed, target_angle, then=Stop.HOLD, wait=True) - Runs the motor at a constant speed towards a given target angle
    # The direction of rotation is automatically selected based on the target angle. It does matter if speed is positive or negative.
    # A redesign might be needed because of run_target.


main()