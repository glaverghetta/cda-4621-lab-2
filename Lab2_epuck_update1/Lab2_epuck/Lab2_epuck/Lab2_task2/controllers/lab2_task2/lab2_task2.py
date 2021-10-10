"""lab2_task2 controller."""

from controller import Robot
from controller import Camera
from controller import CameraRecognitionObject

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

MAX_MOTOR_SPEED_ANGULAR = 2.92 # Motor setting that results in the fastest possible in-place rotation
MAX_ANGULAR_SPEED = 116.92108 # maximum number of degrees the robot can turn in 1 sec.

STOPPING_DISTANCE = 10    # Distance, in inches, from the wall at which the robot should halt
TOTAL_TIME = 30    # Time, in seconds, given to complete the task
kp = 2.5
objectFound = False

CAMERA_CENTER_X = 40   # Camera is 80 x 80 pixels, so 40 is the center x value

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

# get handler to motors and set target position to infinity
lm = robot.getDevice('left wheel motor')
rm = robot.getDevice('right wheel motor')
lm.setPosition(float('inf'))
rm.setPosition(float('inf'))
lm.setVelocity(0)
rm.setVelocity(0)

outputFile = open("lab2task2.txt", "w")

while robot.step(timestep) != -1:
    
    timeRemaining = TOTAL_TIME - robot.getTime()
    if mToIn(fs.getValue()) <= STOPPING_DISTANCE:
        lm.setVelocity(0)
        rm.setVelocity(0)
        break
        
    # First step - rotate to face object
    if objectFound == False:
        if camera.getRecognitionNumberOfObjects() > 0:
            # Keep turning until object is close to center of the camera view
            object = camera.getRecognitionObjects()[0]
            position = object.get_position_on_image()[0]
            blobSize = object.get_size_on_image()[1]
            outputFile.write(str(mToIn(fs.getValue())) + "\t" + str(position) + "\t" + str(blobSize) + "\t" + str(robot.getTime()) + "\n")
            if position <= 45 and position >= 35:
                objectFound = True
                continue
        lm.setVelocity(MAX_MOTOR_SPEED_ANGULAR / 3)
        rm.setVelocity(-MAX_MOTOR_SPEED_ANGULAR / 3)
        
    # Second step - move to 10 inches from object
    else:
        inchesRemaining = mToIn(fs.getValue()) - STOPPING_DISTANCE
        linearVelocity = inchesRemaining / timeRemaining

        # Positive - Facing too far to the right. Negative - facing too far to the left.
        object = camera.getRecognitionObjects()[0]
        position = object.get_position_on_image()[0]
        error = (CAMERA_CENTER_X - position) / (CAMERA_CENTER_X / 8)
        blobSize = object.get_size_on_image()[1]

        if error == 0:
            lm.setVelocity(inToRad(linearVelocity))
            rm.setVelocity(inToRad(linearVelocity))
        elif error > 0:
            leftWheelVelocity = (linearVelocity + error) * kp
            rightWheelVelocity = (2 * linearVelocity) - leftWheelVelocity
            lm.setVelocity(inToRad(leftWheelVelocity))
            rm.setVelocity(inToRad(rightWheelVelocity))
        else:
            rightWheelVelocity = (linearVelocity + error) * kp
            leftWheelVelocity = (2 * linearVelocity) - rightWheelVelocity
            lm.setVelocity(inToRad(leftWheelVelocity))
            rm.setVelocity(inToRad(rightWheelVelocity))
            
        outputFile.write(str(mToIn(fs.getValue())) + "\t" + str(position) + "\t" + str(blobSize) + "\t" + str(robot.getTime()) + "\n")


outputFile.close()