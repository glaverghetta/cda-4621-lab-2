
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, DistanceSensor

# Converts input meters to inches.
def mToIn(m):
    return m * 39.3701
    
# Converts input radians to inches.
def inToRad(inch):
    return inch * 1.25
    
# Converts input radians to inches.
def radToIn(rad):
    return 0.8 * rad

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

MAX_ANGULAR_SPEED = 116.921     # Maximum number of degrees robot can turn in 1 sec.
MAX_MOTOR_SPEED_ANGULAR = 2.92  # Motor setting that results in the fastest possible in-place rotation.
DEGREES_PER_SECOND = 30         # For this lab, robot rotates 30 degrees per second.

turnSpeedPercent = DEGREES_PER_SECOND / MAX_ANGULAR_SPEED
timeAtTurnStart = 0    # Robot keeps track of the time when it started turning.
timeAtWaitStart = 0    # Time when the robot started waiting to turn when turning at the outside corner of an obstacle.

MINIMUM_LINEAR_SPEED = 3.5    # Minimum forward speed, in in/s, at which the robot should travel.
STOPPING_DISTANCE = 8.4       # Distance from a front wall, in inches, at which the robot should turn.
WALL_FOLLOW_DISTANCE = 4
kp = 1

robotState = "find_goal"    # "wall_follow" or "find_goal" or "move_to_goal" or "turn"
nextState = "move_to_goal"
turnType = "left"
#currentlyTurning = False
wallFound = False
canTurnAgain = True
turningCorner = False  # Set to true when following the outside corner of an obstacle.

fs = robot.getDevice('front_ds')
ls = robot.getDevice('left_ds')
rs = robot.getDevice('right_ds')
fs.enable(timestep)
ls.enable(timestep)
rs.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)
CAMERA_CENTER_X = 40

imu = robot.getDevice('inertial unit')
imu.enable(timestep)
imuCentered = False

# get handler to motors and set target position to infinity
lm = robot.getDevice('left wheel motor')
rm = robot.getDevice('right wheel motor')
lm.setPosition(float('inf'))
rm.setPosition(float('inf'))
lm.setVelocity(0)
rm.setVelocity(0)


while robot.step(timestep) != -1:
    print(robotState)
    # Rotate in place until goal is spotted
    if robotState == "find_goal":
        if camera.getRecognitionNumberOfObjects() > 0:
            goal = camera.getRecognitionObjects()[0]
            position = goal.get_position_on_image()[0]
            if position >= CAMERA_CENTER_X - 3 and position <= CAMERA_CENTER_X + 3:
                robotState = "move_to_goal"
                continue
        # Rotate in place
        lm.setVelocity(MAX_MOTOR_SPEED_ANGULAR / 3)
        rm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR / 3)

    # Move directly towards goal
    elif robotState == "move_to_goal":
        if mToIn(fs.getValue()) <= WALL_FOLLOW_DISTANCE * 1.5:
            robotState = "turn_after_motion_to_goal"
            continue
    
        goal = camera.getRecognitionObjects()[0]
        position = goal.get_position_on_image()[0]
        error = (CAMERA_CENTER_X - position) / (CAMERA_CENTER_X)
        size_0 = goal.get_size_on_image()[0]
        size_1 = goal.get_size_on_image()[1]
        size_sum = size_0 + size_1
        if size_sum >= 75 and position >= CAMERA_CENTER_X - 3 and position <= CAMERA_CENTER_X + 3:
            # Reached goal
            lm.setVelocity(0)
            rm.setVelocity(0)
            break
               
        if camera.getRecognitionNumberOfObjects() == 0:
            robotState = "find_goal"
            continue
            
        linearVelocity = 3.5
        if error == 0:
            lm.setVelocity(MINIMUM_LINEAR_SPEED)
            rm.setVelocity(MINIMUM_LINEAR_SPEED)
        elif error < 0 and mToIn(rs.getValue()) >= STOPPING_DISTANCE:
            rightWheelVelocity = (linearVelocity + error) * kp
            leftWheelVelocity = (2 * linearVelocity) - rightWheelVelocity
            lm.setVelocity(leftWheelVelocity)
            rm.setVelocity(rightWheelVelocity)
        elif error > 0 and mToIn(rs.getValue()) >= STOPPING_DISTANCE:
            leftWheelVelocity = (linearVelocity + error) * kp
            rightWheelVelocity = (2 * linearVelocity) - leftWheelVelocity
            lm.setVelocity(leftWheelVelocity)
            rm.setVelocity(rightWheelVelocity)
            
        
        
    elif robotState == "wall_follow":
        if mToIn(fs.getValue()) <= WALL_FOLLOW_DISTANCE * 2:
            timeAtTurnStart = robot.getTime()
            robotState = "turn_after_motion_to_goal"
            continue
            
        if mToIn(rs.getValue()) >= STOPPING_DISTANCE and mToIn(ls.getValue()) >= STOPPING_DISTANCE and canTurnAgain:
            timeAtWaitStart = robot.getTime()
            canTurnAgain = False
            robotState = "wait"
            nextState = "turn"
            turningCorner = True
            continue
        if turningCorner:
            if mToIn(rs.getValue()) <= WALL_FOLLOW_DISTANCE and mToIn(fs.getValue()) >= 2 * STOPPING_DISTANCE:
                print("done turning corner")
                turningCorner = False
                canTurnAgain = True
                if camera.getRecognitionNumberOfObjects() > 0 and mToIn(fs.getValue()) >= STOPPING_DISTANCE:
                    timeAtWaitStart = robot.getTime()
                    robotState = "wait"
                    nextState = "move_to_goal"
                           
        lm.setVelocity(MINIMUM_LINEAR_SPEED)
        rm.setVelocity(MINIMUM_LINEAR_SPEED)

    elif robotState == "turn":
        if robot.getTime() - timeAtTurnStart >= 3:
            robotState = "wall_follow"
        else:
            if turnType == "left":
                if turningCorner == False:
                    lm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                    rm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                else:
                    lm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                    rm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
            
                #if mToIn(rs.getValue()) >= STOPPING_DISTANCE * 2 and not currentlyTurning: 
                #    lm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                #    rm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                #else:
                #    currentlyTurning = True
                #    print("turning left " + str(robot.getTime()) + " " + robotState)
                #    lm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                #    rm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
            else:
                lm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                rm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                #if mToIn(ls.getValue()) >= STOPPING_DISTANCE * 2 and not currentlyTurning:
                #    lm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                #    rm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                #else:
                #    currentlyTurning = True
                #    lm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                #    rm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
    elif robotState == "wait":
        if robot.getTime() - timeAtWaitStart >= 1.75:
            if mToIn(rs.getValue()) >= WALL_FOLLOW_DISTANCE:
                robotState = nextState
                timeAtTurnStart = robot.getTime()
            elif camera.getRecognitionNumberOfObjects() > 0 and mToIn(rs.getValue()) <= STOPPING_DISTANCE:
                robotState = nextState
            else:
                timeAtWaitStart = robot.getTime() + 1
        else:
            lm.setVelocity(MINIMUM_LINEAR_SPEED)
            rm.setVelocity(MINIMUM_LINEAR_SPEED)
    
    else:
        rotation = imu.getRollPitchYaw()[2]
        # Determine if robot is moving mostly in a straight line
        #if imu <= -3.09 or imu <= 0.04 or (imu >= 1.52 and imu <= 1.62) or imu >= 3.09 or (imu <= -1.52 and imu >= -1.62):
        if abs(rotation) >= 3.09 or (abs(rotation) >= 1.52 and abs(rotation) <= 1.62) or abs(rotation) <= 0.05:
            imuCentered = True
        
        if mToIn(fs.getValue()) >= WALL_FOLLOW_DISTANCE and imuCentered == True:
            imuCentered = False
            robotState = "wall_follow"
        else:
            if turnType == "left":
                lm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
                rm.setVelocity(MAX_MOTOR_SPEED_ANGULAR * turnSpeedPercent)
