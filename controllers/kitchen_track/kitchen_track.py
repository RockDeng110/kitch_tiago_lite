from controller import Robot, Supervisor
import numpy as np
import math
from matplotlib import pyplot as plt
from scipy import signal



# Constants
LIDAR_OFFSET_X = 0.202  # Adjust based on TIAGO Lite's LiDAR offset in meters
LIDAR_OFFSET_Y = 0.0  # Adjust based on actual placement
DISPLAY_WIDTH = 450
DISPLAY_HEIGHT = 567
LIDAR_MAX_RANGE = 5.0  # Example max LiDAR range
LIDAR_MIN_INDEX = 80  # First 80 readings should be ignored
LIDAR_MAX_INDEX = -80  # Last 80 readings should be ignored


def degrees_to_radians(degrees):
    return degrees * (math.pi / 180)








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
lidar = robot.getDevice("Hokuyo URG-04LX-UG01") 
lidar.enable(TIMESTEP)
lidar.enablePointCloud()
LIDAR_RESOLUTION = 0.36 # degree
LIDAR_FEILD_OF_VIEW = 240 # degree



gps = robot.getDevice("gps")
gps.enable(TIMESTEP)
compass = robot.getDevice("compass")  # Use compass instead of IMU
compass.enable(TIMESTEP)

# def wait_for_lidar():
#     for _ in range(10):
#         robot.step(TIMESTEP)
#     if lidar.getNumberOfPoints() == 0:
#         print("LIDAR initialization failed: No points available.")
#     else:
#         print("LIDAR initialized successfully.")

# wait_for_lidar()



# WP = [(0, 0), (0.75, -0.6), (0.54, -2.75), (-0.62, -3.16), (-1.67, -2.74), (-1.8, 0), (-1, 0.14), 
#       (-1.8, 0), (-1.67, -2.74), (-0.62, -3.16), (0.54, -2.75), (0.75, -0.6),  (0, 0)]

WP = [(0, 0), (0.47, -0.256), (0.62, -0.88), 
      (0.559, -2.43), (-0.08, -3.11), (-1.11, -3.13), 
      (-1.67, -2.76), (-1.7, -2.33), (-1.7, -1.38), 
      (-1.72, -0.574), (-1.28, 0.247), (-0.612, 0.309)]
index = 0  # max point number 15

# marker = robot.getFromDef("marker").getField("translation")



# Get robot position and orientation
def calculate_robot_pose():
    global xw, yw, omegaz
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]
    omegaz=np.arctan2(compass.getValues()[0],compass.getValues()[1])

# Convert world coordinates to map

def world2map(xw, yw, map_width=400, map_height=504, 
              world_min_x=-2.25, world_max_x=2.25, 
              world_min_y=-3.92, world_max_y=1.75):
    scale_x = map_width / (world_max_x - world_min_x)
    scale_y = map_height / (world_max_y - world_min_y)
    px = int((xw - world_min_x) * scale_x)
    py = int((yw - world_min_y) * scale_y)
    py = map_height - 1 - py  # Invert Y coordinate
    #print(f"Mapped X: {px}, Mapped Y: {py}")

    return max(0, min(map_width - 1, px)), max(0, min(map_height - 1, py))

# Initialize map
display = robot.getDevice('display')
display.setColor(0xFF0000)
print(f"Updated Display size: {display.getWidth()}x{display.getHeight()}")


# Display test
# for x in range(300):
#     for y in range(300):
#         display.setColor(0xFF0000)  # Red
#         display.drawPixel(x, y)  # Draw a red screen


map = np.zeros((450, 567))
kernel_size = 30
kernel = np.ones((kernel_size, kernel_size))





# Print in a structured way
def print_lidar_data(ranges, num_values=10):
    """
    Prints LiDAR range data in a structured and readable format.
    
    Args:
        ranges (list or np.array): Processed LiDAR range values.
        num_values (int): Number of values to print per row.
    """
    print("range size: {ranges.size}")
    formatted_ranges = [f"{r:.2f}" for r in ranges]  # Format numbers to 2 decimal places
    for i in range(0, len(formatted_ranges), num_values):
        print(" ".join(formatted_ranges[i:i+num_values]))


def update_probability_map(px, py, detected):
    """Update probability map with sensor data."""
    if detected:
        map[px, py] = min(1.0, map[px, py] + 0.05)  # Increase probability for obstacles
    else:
        map[px, py] = max(0.0, map[px, py] - 0.02)  # Decrease probability when space is cleared



def shortest_turn_angle(target_angle, current_angle):
    """Returns the shortest angle difference between target and current."""
    diff = target_angle - current_angle
    return (diff + np.pi) % (2 * np.pi) - np.pi  # Ensures shortest turn

move_forward = 1
while robot.step(TIMESTEP) != -1:
    calculate_robot_pose()
    # print(f"GPS position: xw={xw}, yw={yw}, theta={omegaz}")


    # Compute rho (distance to target)
    rho = np.sqrt((xw - WP[index][0])**2 + (yw - WP[index][1])**2)

    # Compute correct heading angle
    desired_angle = np.arctan2(WP[index][1] - yw, WP[index][0] - xw)

    # Ensure shortest turn
    alpha = shortest_turn_angle(desired_angle, omegaz)

    # print(f"rho: {rho}, alpha: {alpha}, index: {index}")

    # Switch to next waypoint if close enough
    if rho < 0.1 and index < len(WP) and index >= 0:
        prev_index = index
        if move_forward == 1:
            index += 1
        else:
            index -= 1
        if index >= len(WP) - 1:
            move_forward = 0
        print(f"Reached waypoint {prev_index}, moving to waypoint {index}")

    # Compute wheel velocities with improved parameters
    p1, p2 = 2.0, 3.0
    leftSpeed = -alpha * p1 + rho * p2
    rightSpeed = alpha * p1 + rho * p2
    leftMotor.setVelocity(max(min(leftSpeed, MAX_SPEED), -MAX_SPEED))
    rightMotor.setVelocity(max(min(rightSpeed, MAX_SPEED), -MAX_SPEED))

    # # Display trajectory
    # px, py = world2map(xw, yw)
    # # Increment probability (up to 1)
    # map[px, py] = min(1, map[px, py] + 0.01)
    # # Convert probability to grayscale (0-255)
    # v = int(map[px, py] * 255)
    # color = v * 256**2 + v * 256 + v  # Convert to 24-bit color
    # display.setColor(color)
    # display.drawPixel(px, py)


    # draw trajectory in display
    px, py = world2map(xw, yw)
    # print(f"px: {px} py: {py}")
    display.setColor(0xFF0000)
    display.drawPixel(px,py)

    # LIDAR processing
    ranges = lidar.getRangeImage()
    # Ignore blocked readings
    ranges = ranges[LIDAR_MIN_INDEX:LIDAR_MAX_INDEX]
    # Filter infinite values
    ranges = np.where(np.isinf(ranges), 100, ranges)  # Replace infinite values with 100
    # print(f"LIDAR data: {ranges}")  # Print first 10 values
    # print_lidar_data(ranges, num_values=30)

    angles = np.linspace(degrees_to_radians(120), degrees_to_radians(-120), 667)
    angles = angles[LIDAR_MIN_INDEX:LIDAR_MAX_INDEX]
   
    x_r, y_r = [], [] # coordinates in robot's system
    x_w, y_w = [], [] # coordinates in world's system
    for i, angle in enumerate(angles):
        try:
            # print(f"Angle {angle:.2f}, Distance {ranges[i]:.2f}")
            # Validate the range value
            # if not np.isfinite(ranges[i]) or abs(ranges[i]) > 1:
            #     #print(f"Skipping invalid range at index {i}: {ranges[i]}")
            #     continue
            
            # Robot's coordinate transformation
            x_i = ranges[i] * np.cos(angle)
            y_i = ranges[i] * np.sin(angle)

            # Apply LiDAR sensor offset
            x_i = x_i + LIDAR_OFFSET_X
            y_i = y_i + LIDAR_OFFSET_Y

            x_r.append(x_i)
            y_r.append(y_i)

            # World's coordinate transformation
            x_w_s = x_i * np.cos(omegaz) - y_i * np.sin(omegaz) + xw
            y_w_s = x_i * np.sin(omegaz) + y_i * np.cos(omegaz) + yw

            # Clamp display coordinates
            #x_w_s = max(0, min(300, x_w_s))
            #y_w_s = max(0, min(300, y_w_s))

            # Append valid coordinates
            x_w.append(x_w_s)
            y_w.append(y_w_s)

            # Debugging output
            #print(f"i: {i}, ranges[i]: {ranges[i]:.2f}, angle: {angle:.2f}, x_i: {x_i:.2f}, y_i: {y_i:.2f}, omegaz: {omegaz:.2f}")

            # Display pixel
            #print(f"Drawing pixel at ({x_w_s}, {y_w_s})")
            #display.setColor(0xFFFFFF)
            #px, py = world2map(x_w_s, y_w_s)
            #display.drawPixel(px,py)
            # print(f"obstacle in robot axes: X: {x_i}, Y: {y_i}")
            # print(f"obstacle in world axes: X: {x_w_s}, Y: {y_w_s}")
            
            # Convert world coordinates to map indices
            px, py = world2map(x_w_s, y_w_s)

            # Increment probability (up to 1)
            map[px, py] = min(1, map[px, py] + 0.01)
            # update_probability_map(px, py, detected=True)


            # # Convert probability to grayscale (0-255)
            v = int(map[px, py] * 255)
            color = v * 256**2 + v * 256 + v  # Convert to 24-bit color

            # Display the pixel
            display.setColor(color)
            display.drawPixel(px, py)
            # print(f"Drawing trajectory pixel at ({px}, {py}) with value {map[px, py]:.2f}")
        except Exception as e:
            print(f"Error at index {i}: {e}")
            continue


    # Plot the scan for debug
    # plt.clf()
    # # plt.scatter(x_r, y_r, s=2, label="Robot Frame", color="blue")  
    # plt.scatter(x_w, y_w, s=2, label="World Frame", color="orange")  
    # plt.legend()
    # plt.pause(0.01)

    # Perform 2D convolution to compute the configuration space every 100 timesteps
    if robot.step(TIMESTEP) % 100 == 0:
        cmap = signal.convolve2d(map, kernel, mode='same')
        cspace = cmap > 0.9  # Threshold to mark obstacles
        
        # Visualize the configuration space
        plt.clf()
        plt.imshow(cspace, cmap='gray')
        plt.title("Configuration Space")
        plt.pause(0.001)
