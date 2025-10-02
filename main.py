#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math # For pi and trigonometric functions

#Initialize the EV3 brick
ev3 = EV3Brick()
ev3.speaker.beep()

# Sets up motors and sensors
right_motor = Motor(Port.A)
left_motor = Motor(Port.B)
ts = TouchSensor(Port.S1)
uss = UltrasonicSensor(Port.S2)

# Robot's physical variables
wheel_diameter = 55.5 # mm
axle_track = 125 # mm

#Initialize the drivebase
db = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=125)

# State management variables
is_running = True
is_on = False

# Lawn variables
global_lawn_width = 1000 # mm
global_lawn_height = 1000 # mm

# Lawn mowing robot class that cuts grass in a column-by-column pattern, right to left
class LawnRobot:
    def __init__(self, drive_speed, turn_rate, robot_width):
        self.drive_speed = drive_speed # mm/s
        self.turn_rate = turn_rate # degrees/s
        self.width = robot_width # mm
        self.cols_mowed = 1 # (Renamed robotTurns variable) Keeps track of columns of grass mowed
        self.pos_x = 0 # Robot's x-coordinate
        self.pos_y = 0 # Robot's y-coordinate
        self.diff_y = 0 # Potential difference in actual distance driven and distance moved on the y-axis
        self.is_done = False # Bool to check if the program should stop

    def is_ts_pressed(self):
        if (ts.pressed()):
            self.is_done = True
        return self.is_done

    def check_column_status(self, lawn_height, lawn_width):
        # Checks if robot has mowed enough columns to cover the lawn width
        if (self.cols_mowed * self.width >= lawn_width):
            self.is_done = True
            finish_program()
            return
        
        # Otherwise turns and starts mowing next column
        self.begin_new_column()
  
        # Resets distance and relative y-position when reaching top or bottom point of lawn
        db.reset() 
        self.pos_y = 0
        self.diff_y = 0
    
    # Ensures robot turns the correct way based on amount of columns mowed and drives the length of its own width on the x-axis to begin mowing a new column
    def begin_new_column(self):
        if ((self.cols_mowed%2) == 0): # Clockwise 180-degree turn if number of columns mowed is even
            db.turn(90)
            db.straight(self.width)
            db.turn(90)
        else: # Counter-clockwise 180-degree turn if number of columns is odd
            db.turn(-90)
            db.straight(self.width)
            db.turn(-90)

        self.cols_mowed += 1 # Increments columns mowed

    # Calculates distance moved on the y-axis if the robot senses an obstacle and has to drive around based on a given length and angle
    def interpolate_distance_y(self, hypotenuse, angle):
        # Converts angle to radians
        angle_rad = math.radians(angle)
        # Calculates the adjacent side (y-component) using cosine
        adjacent = hypotenuse * math.cos(angle_rad)
        return adjacent

    # Drives around an obstacle if one is sensed by the ultrasound sensor by turning a given angle and driving a given distance around it
    def avoid_obstacle(self, length, angle):
        # Stores distance driven in current column before driving around obstacle so we can get the total distance moved on the y-axis rather than only measuring the distance driven by the robot and risk mowing too little of the column
        self.pos_y = db.distance()

        # Turns a positive angle for first and last turn sequence; otherwise negative angle 
        for i in range(0,4):
            if (self.is_ts_pressed()): 
                return
            match i:
                case 0: # First loop
                    db.turn(angle)
                    db.straight(length)
                case 3: # Last loop 
                    db.turn(angle)
                case _: 
                    db.turn(-angle)
                    db.straight(length)
                
        # Calculates difference in distance driven and distance moved on y-axis
        self.diff_y += self.interpolate_distance_y(angle, length)

    # Mows lawn 
    def mow_lawn(self, lawn_height, lawn_width):
        # Starts driving
        db.drive(self.drive_speed, self.turn_rate)

        if (self.is_ts_pressed()): # Stops program if touch sensor is pressed
            db.stop()
            return

        obstacle_met = False
        # Runs while an obstacle has been sensed but not gotten too close yet
        while (uss.distance() <= 370 and obstacle_met == False):
            db.drive(int(self.drive_speed/2), self.turn_rate) # Slows down in front of obstacle
            wait(2000) # Waits for 2 seconds in case obstacle is something that moves
            
            if (uss.distance() <= 250): # If obstacle is 25 cm or less away
                obstacle_met = True

        self.pos_y = db.distance() - self.diff_y # Sets the y-position equal to the distance driven on current column minus any difference in distance that may have occurred due to driving around obstacles 

        if (self.pos_y < lawn_height and obstacle_met):
            self.avoid_obstacle(length=250, angle=45)
            #self.dynamic_avoid_obstacle_TEST() 
        elif (self.pos_y >= lawn_height):
            self.check_column_status(lawn_height, lawn_width)

# Creates an instance of the lawn robot
robot = LawnRobot(drive_speed=110, turn_rate=0, robot_width=140)

def robot_loop():
    if (robot.is_done):
        finish_program()
        return

    # Mows lawn 
    robot.mow_lawn(lawn_height=global_lawn_height, lawn_width=global_lawn_width)

# Ends program
def finish_program():
    global is_running
    global is_on

    db.stop() # Extra stop to the drivebase just in case

    # Sets state management variables to false to ensure ending the program loop
    is_on = False
    is_running = False

    ev3.speaker.say("Exercise done")
    return

# Program loop
while (is_running):
    if ((ts.pressed()) and (is_on == False)):
        is_on = True
        ev3.speaker.say("Exercise 2")

    while (is_on):
        robot_loop()
