"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

# create the Robot instance.
robot = Robot()
# Get the list of devices
devices = robot.getDeviceList()
# Print all device names
for device in devices:
    print(device)


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
MAX_SPEED = 6.28
r = 0.0201 # radius of wheels
d = 0.052  # distance between two wheels
delta_x = 0
delta_omegaz = 0
delta_time = timestep/1000  # loop duration in while in seconds
delta_distance = 0
xw = 0  # displacement along the X-axis of the world 
yw = 0.028  # displacement along the Y-axis of the world
omegaz = 1.57  # Rotations along the Z-axis

# Get wheel motors and init them
leftMotor = robot.getDevice("wheel_left_joint")
rightMotor = robot.getDevice("wheel_right_joint")
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


# add gps and compass
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)


# Lidar related
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Allow LIDAR to warm up
for _ in range(10):  # Wait for 10 simulation steps
    robot.step(timestep)

# Wait for the LIDAR to initialize
if lidar.getNumberOfPoints() == 0:
    print("LIDAR initialization failed: No points available.")
else:
    print("LIDAR initialized successfully.")

ranges = lidar.getRangeImage()
if ranges is None or len(ranges) == 0:
    print("Error: LIDAR range image is empty or not accessible.")
else:
    print(f"LIDAR range image lens: {len(ranges)}")
    print(f"LIDAR range image: {ranges}")

angles = np.linspace(3.1415, -3.1415, 360)



WP = [(0, 0.68), (0.44, 0.68), (0.65, 0.52), (0.32, 0.23), (0.63, 0), (0.63, -0.15), (0, -0.15),  (0, 0)]
index = 0

display = robot.getDevice('display')
display.setColor(0xFF0000)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # move_robot(2.0, 2.0)  # Move forward
    pass

# Enter here exit cleanup code.
