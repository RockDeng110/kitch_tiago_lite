from controller import Robot, Supervisor
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

# Create the robot instance
robot = Supervisor()

# Print all device names
for i in range(robot.getNumberOfDevices()):
    device = robot.getDeviceByIndex(i)
    print(device.getName())


# Get the time step of the current world
TIMESTEP = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28  # Maximum wheel speed
r = 0.075  # TIAGO LITE wheel radius (approximate)
d = 0.4  # Distance between wheels (approximate)

delta_x = 0
delta_omegaz = 0
delta_time = TIMESTEP / 1000  # Loop duration in seconds
delta_distance = 0
xw, yw = 0, 0.028  # Initial world position of the robot
omegaz = 1.57  # Initial orientation

# Initialize motors
leftMotor = robot.getDevice("wheel_left_joint")
rightMotor = robot.getDevice("wheel_right_joint")
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


# Sensors
lidar = robot.getDevice("Hokuyo URG-04LX-UG01")  # TIAGO LITE uses "front_laser"
lidar.enable(TIMESTEP)
lidar.enablePointCloud()
gps = robot.getDevice("gps")
gps.enable(TIMESTEP)
compass = robot.getDevice("compass")  # Use compass instead of IMU
compass.enable(TIMESTEP)

def wait_for_lidar():
    for _ in range(10):
        robot.step(TIMESTEP)
    if lidar.getNumberOfPoints() == 0:
        print("LIDAR initialization failed: No points available.")
    else:
        print("LIDAR initialized successfully.")

wait_for_lidar()

# WP = [(0, 0), (0.75, -0.6), (0.54, -2.75), (-0.62, -3.16), (-1.67, -2.74), (-1.8, 0), (-1, 0.14), 
#       (-1.8, 0), (-1.67, -2.74), (-0.62, -3.16), (0.54, -2.75), (0.75, -0.6),  (0, 0)]

WP = [(0, 0), (0.47, -0.256), (0.558, -1.19), (4.16, -2.58), (-0.08, -3.11), (-1.11, -3.13), (-1.67, -2.76), (-1.7, -2.33), (-1.7, -1.38), (-1.72, -0.574), (-1.28, 0.247), (-0.612, 0.309)]
index = 0  # max point number 15

# marker = robot.getFromDef("marker").getField("translation")

USE_GPS_AND_COMPASS = 1

# Get robot position and orientation
def calculate_robot_pose():
    global xw, yw, omegaz
    if USE_GPS_AND_COMPASS:
        xw, yw, _ = gps.getValues()
        omegaz = np.arctan2(compass.getValues()[0], compass.getValues()[1])
        # omegaz = np.arctan2(compass.getValues()[1], compass.getValues()[0])


# Convert world coordinates to map

def world2map(xw, yw, map_width=300, map_height=300, 
              world_min_x=-0.195, world_max_x=0.805, 
              world_min_y=-0.25, world_max_y=0.75):
    scale_x = map_width / (world_max_x - world_min_x)
    scale_y = map_height / (world_max_y - world_min_y)
    px = int((xw - world_min_x) * scale_x)
    py = int((yw - world_min_y) * scale_y)
    py = map_height - 1 - py  # Invert Y coordinate
    return max(0, min(map_width - 1, px)), max(0, min(map_height - 1, py))

# Initialize map
display = robot.getDevice('display')
map = np.zeros((300, 300))
kernel_size = 30
kernel = np.ones((kernel_size, kernel_size))

def shortest_turn_angle(target_angle, current_angle):
    """Returns the shortest angle difference between target and current."""
    diff = target_angle - current_angle
    return (diff + np.pi) % (2 * np.pi) - np.pi  # Ensures shortest turn

move_forward = 1
while robot.step(TIMESTEP) != -1:
    calculate_robot_pose()

    # Compute rho (distance to target)
    rho = np.sqrt((xw - WP[index][0])**2 + (yw - WP[index][1])**2)

    # Compute correct heading angle
    desired_angle = np.arctan2(WP[index][1] - yw, WP[index][0] - xw)

    # Ensure shortest turn
    alpha = shortest_turn_angle(desired_angle, omegaz)

    print(f"rho: {rho}, alpha: {alpha}, index: {index}")

    # Switch to next waypoint if close enough
    if rho < 0.1 and index < len(WP) - 1 and index >= 0:
        prev_index = index
        if move_forward == 1:
            index += 1
        else:
            index -= 1
        if index == len(WP) - 1:
            move_forward = 0
        print(f"Reached waypoint {prev_index}, moving to waypoint {index}")

    # Compute wheel velocities with improved parameters
    p1, p2 = 6.0, 3.0
    leftSpeed = -alpha * p1 + rho * p2
    rightSpeed = alpha * p1 + rho * p2
    leftMotor.setVelocity(max(min(leftSpeed, MAX_SPEED), -MAX_SPEED))
    rightMotor.setVelocity(max(min(rightSpeed, MAX_SPEED), -MAX_SPEED))

    # Display trajectory
    px, py = world2map(xw, yw)
    display.setColor(0xFF0000)
    display.drawPixel(px, py)

    # LIDAR processing
    ranges = np.array(lidar.getRangeImage())
    ranges = np.where(np.isinf(ranges), 100, ranges)  # Handle infinite values
    angles = np.linspace(3.1415, -3.1415, len(ranges))
    
    for i, angle in enumerate(angles):
        if not np.isfinite(ranges[i]) or abs(ranges[i]) > 1:
            continue
        x_i, y_i = ranges[i] * np.cos(angle), ranges[i] * np.sin(angle)
        x_w_s, y_w_s = x_i * np.cos(omegaz) - y_i * np.sin(omegaz) + xw, \
                       x_i * np.sin(omegaz) + y_i * np.cos(omegaz) + yw
        px, py = world2map(x_w_s, y_w_s)
        map[px, py] = min(1, map[px, py] + 0.01)
        v = int(map[px, py] * 255)
        display.setColor(v * 256**2 + v * 256 + v)
        display.drawPixel(px, py)
