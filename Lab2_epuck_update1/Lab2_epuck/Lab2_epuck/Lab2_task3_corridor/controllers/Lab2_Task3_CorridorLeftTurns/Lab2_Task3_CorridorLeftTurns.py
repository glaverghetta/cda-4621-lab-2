"""lab2_task3corridor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, DistanceSensor

# create the Robot instance.
robot = Robot()

# Converts input meters to inches.
def mToIn(m):
    return m * 39.3701
    
# Converts input radians to inches.
def inToRad(inch):
    return inch * 1.25
	
# Converts input radians to inches.
def radToIn(rad):
    return 0.8 * rad

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

MAX_ANGULAR_SPEED = 116.921     # Maximum number of degrees robot can turn in 1 sec.
MAX_MOTOR_SPEED_ANGULAR = 2.92  # Motor setting that results in the fastest possible in-place rotation.
DEGREES_PER_SECOND = 30         # For this lab, robot rotates 30 degrees per second.

turnSpeedPercent = DEGREES_PER_SECOND / MAX_ANGULAR_SPEED
timeAtTurnStart = 0    # Robot keeps track of the time when it started turning.

pathLength = 220			  # Length, in inches, of the entire path to be followed.
MINIMUM_LINEAR_SPEED = 3.5    # Minimum forward speed, in in/s, at which the robot should travel.
STOPPING_DISTANCE = 8.4       # Distance from a front wall, in inches, at which the robot should turn.
WALL_FOLLOWING_DISTANCE = 7
kp = 0.05

robotState = "wall_follow"    # Possible values: "wall_follow" or "turn_left" or "turn _around"

fs = robot.getDevice('front_ds')
ls = robot.getDevice('left_ds')
rs = robot.getDevice('right_ds')
fs.enable(timestep)
ls.enable(timestep)
rs.enable(timestep)

lp = robot.getDevice('left wheel sensor')
lp.enable(timestep)
rp = robot.getDevice('right wheel sensor')
rp.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# get handler to motors and set target position to infinity
lm = robot.getDevice('left wheel motor')
rm = robot.getDevice('right wheel motor')
lm.setPosition(float('inf'))
rm.setPosition(float('inf'))
lm.setVelocity(0)
rm.setVelocity(0)


while robot.step(timestep) != -1:
    distanceTravelled = (radToIn(lp.getValue()) + radToIn(rp.getValue())) / 2
    if distanceTravelled >= pathLength:
        lm.setVelocity(0)
        rm.setVelocity(0)
        break
	
    if robotState == "wall_follow":
        # If front sensor detects a wall in front, turn.
        if mToIn(fs.getValue()) <= STOPPING_DISTANCE:
            # Turn around if there's a wall too close on the left. Else turn left.
            if mToIn(ls.getValue()) <= STOPPING_DISTANCE:
                robotState = "turn_around"
                timeAtTurnStart = robot.getTime()
            else:
                robotState = "turn_left"
                timeAtTurnStart = robot.getTime()
        else:
            # Wall follow.
            ytl = mToIn(ls.getValue())
            ytr = mToIn(rs.getValue())
            
            etl = WALL_FOLLOWING_DISTANCE - ytl
            etr = WALL_FOLLOWING_DISTANCE - ytr
            
            utl = kp * etl
            utr = kp * etr
            
            if ytl < ytr:
                if etl == 0:
                    lm.setVelocity(inToRad(MINIMUM_LINEAR_SPEED))
                    rm.setVelocity(inToRad(MINIMUM_LINEAR_SPEED))
                elif etl < 0:
                    # Robot is too far from wall.
                    rightWheelVelocity = MINIMUM_LINEAR_SPEED + abs(utl)
                    leftWheelVelocity = (2 * MINIMUM_LINEAR_SPEED) - rightWheelVelocity
                    if rightWheelVelocity > 6.28:
                        rm.setVelocity(6.28)
                    else:
                        rm.setVelocity(rightWheelVelocity)
                    
                    if leftWheelVelocity > 6.28:
                        lm.setVelocity(6.28)
                    else:
                        lm.setVelocity(leftWheelVelocity)
                else:
                    # Robot is too close to wall.
                    leftWheelVelocity = MINIMUM_LINEAR_SPEED + abs(utl)
                    rightWheelVelocity = (2 * MINIMUM_LINEAR_SPEED) - leftWheelVelocity
                    if leftWheelVelocity > 6.28:
                        lm.setVelocity(6.28)
                    else:
                        lm.setVelocity(leftWheelVelocity)
                        
                    if rightWheelVelocity > 6.28:
                        rm.setVelocity(6.28)
                    else:
                        rm.setVelocity(rightWheelVelocity)
            else:
                if etr == 0:
                    lm.setVelocity(inToRad(MINIMUM_LINEAR_SPEED))
                    rm.setVelocity(inToRad(MINIMUM_LINEAR_SPEED))
                elif etr < 0:
                    # Robot is too far from wall.
                    leftWheelVelocity = MINIMUM_LINEAR_SPEED + abs(utr)
                    rightWheelVelocity = (2 * MINIMUM_LINEAR_SPEED) - leftWheelVelocity
                    if rightWheelVelocity > 6.28:
                        rm.setVelocity(6.28)
                    else:
                        rm.setVelocity(rightWheelVelocity)
                    
                    if leftWheelVelocity > 6.28:
                        lm.setVelocity(6.28)
                    else:
                        lm.setVelocity(leftWheelVelocity)
                else:
                    # Robot is too close to wall.
                    rightWheelVelocity = MINIMUM_LINEAR_SPEED + abs(utr)
                    leftWheelVelocity = (2 * MINIMUM_LINEAR_SPEED) - rightWheelVelocity
                    if leftWheelVelocity > 6.28:
                        lm.setVelocity(6.28)
                    else:
                        lm.setVelocity(leftWheelVelocity)
                        
                    if rightWheelVelocity > 6.28:
                        rm.setVelocity(6.28)
                    else:
                        rm.setVelocity(rightWheelVelocity)
            
    elif robotState == "turn_left":
        # Turn left until 3 secondsd have elapsed.
        if robot.getTime() - timeAtTurnStart >= 3:
            print("WALL FOLLOW")
            robotState = "wall_follow"
        else:
            lm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
            rm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
    else:
        # Turn around until 6 seconds have elapsed.
        if robot.getTime() - timeAtTurnStart >= 6:
            pathLength += 2 * distanceTravelled
            print("WALL_FOLLOW")
            robotState = "wall_follow"
        else:
            lm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
            rm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)

    print("ls: " + str(mToIn(ls.getValue())) + " rs: " + str(mToIn(rs.getValue())) + " ds: " + str(mToIn(fs.getValue())) + " distance travlled: " + str(distanceTravelled)) 
