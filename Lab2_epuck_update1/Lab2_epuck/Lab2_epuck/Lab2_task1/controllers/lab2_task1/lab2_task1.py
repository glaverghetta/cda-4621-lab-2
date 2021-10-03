"""lab2_task1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# Converts input meters to inches.
def mToIn(m):
    return m * 39.3701
    
# Converts input radians to inches.
def inToRad(inch):
    return inch * 1.25

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

MIN_DIST_FROM_WALL = 2.5  # Minimum distance, in inches, that the robot should be from the side walls.
MAX_DIST_FROM_WALL = 5.5  # Maximum distance, in inches, that the robot should be from the side walls.
OPTIMAL_DIST_FROM_WALL = 3.624 # When the robot is this far from each wall, it will be directly in between them.
rt = OPTIMAL_DIST_FROM_WALL
kp = 2.5

INCHES_TO_TRAVEL = 40.0   # Total distance the robot must travel.
STOPPING_DISTANCE = 10.0  # Distance, in inches, from the wall at which the robot should halt.
TOTAL_TIME = 30.0         # Time, in seconds, given to complete the task.

inchesRemaining = INCHES_TO_TRAVEL               # inches left for the robot to travel
linearVelocity = inchesRemaining / TOTAL_TIME    # total forward velocity of the robot
timeRemaining = TOTAL_TIME                       # Time left for the robot to move 

#initialization of motors
lm = robot.getDevice('left wheel motor')
rm = robot.getDevice('right wheel motor')
lm.setPosition(float('inf'))
rm.setPosition(float('inf'))
lm.setVelocity(0)
rm.setVelocity(0)

#distance sensors 
fs = robot.getDevice('front_ds')
ls = robot.getDevice('left_ds')
rs = robot.getDevice('right_ds')
fs.enable(timestep)
ls.enable(timestep)
rs.enable(timestep)

outputFile = open("lab2task1.txt", "w")

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    #if mToIn(fs.getValue()) <= STOPPING_DISTANCE:
    if timeRemaining <= 0:
        lm.setVelocity(0)
        rm.setVelocity(0)
        break
        
    ytl = mToIn(ls.getValue()) # Left sensor's value
    ytr = mToIn(rs.getValue()) # Right sensor's value
    
    etl = rt - ytl   # Error values for left and right distances
    etr = rt - ytr
    
    utl = kp * etl   # Motor velocity control functions
    utr = kp * etr
    
    inchesRemaining = mToIn(fs.getValue()) - STOPPING_DISTANCE
    timeRemaining = TOTAL_TIME - robot.getTime()
    linearVelocity = inchesRemaining / timeRemaining
    
    
    if etl == 0:
        lm.setVelocity(inToRad(linearVelocity))
        rm.setVelocity(inToRad(linearVelocity))
        
    if abs(etl) > abs(etr):
        # Robot is closer to left wall.
        leftWheelVelocity = linearVelocity + utl
        rightWheelVelocity = (2 * linearVelocity) - leftWheelVelocity
        lm.setVelocity(inToRad(leftWheelVelocity))
        rm.setVelocity(inToRad(rightWheelVelocity))
    else:
        # Robot is closer to right wall.
        rightWheelVelocity = linearVelocity + utr
        leftWheelVelocity = (2 * linearVelocity) - rightWheelVelocity
        lm.setVelocity(inToRad(leftWheelVelocity))
        rm.setVelocity(inToRad(rightWheelVelocity))
        
    print(str(mToIn(fs.getValue())) + "\t" + str(mToIn(ls.getValue())) + "\t" + str(mToIn(rs.getValue())))
    outputFile.write(str(mToIn(fs.getValue())) + "\t" + str(mToIn(ls.getValue())) + "\t" + str(mToIn(rs.getValue())) + "\t" + str(robot.getTime()) + "\n")
    
outputFile.close()