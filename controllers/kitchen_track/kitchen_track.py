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

# WP = [(0, 0), (0.47, -0.256), (0.62, -0.88), 
#       (0.559, -2.43), (-0.08, -3.11), (-1.11, -3.13), 
#       (-1.67, -2.76), (-1.7, -2.33), (-1.7, -1.38), 
#       (-1.72, -0.574), (-1.28, 0.247), (-0.612, 0.309)]

# WP = [(0, 0), (0.47, -0.256),  
#       (0.559, -2.43), (-0.08, -3.11), (-1.11, -3.13), 
#       (-1.67, -2.76), (-1.7, -1.38), 
#       (-1.72, -0.574), (-1.28, 0.247)]

# WP = [(0, 0), (0.47, -0.256),  
#       (0.559, -3.11), (-1.11, -3.13), 
#       (-1.67, -2.76), (-1.7, -1.38), 
#       (-1.72, -0.574), (-1.28, 0.247)]

# WP = [(0, 0), (0.47, -0.256),  
#       (0.53, -2.8), (-1.11, -3.13), 
#       (-1.67, -2.76), (-1.7, -1.38), 
#       (-1.72, -0.574), (-1.28, 0.247)]

WP = [(0, 0), (0.47, -0.256),  
      (0.53, -3),
      (-1.67, -3), (-1.7, -1.38), 
      (-1.72, -0.574), (-1.28, 0.247)]

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

def world2mapv(xw, yw, map_width=400, map_height=504, 
              world_min_x=-2.25, world_max_x=2.25, 
              world_min_y=-3.92, world_max_y=1.75):
    scale_x = map_width / (world_max_x - world_min_x)
    scale_y = map_height / (world_max_y - world_min_y)
    
    # Apply scaling
    px = ((xw - world_min_x) * scale_x).astype(int)
    py = ((yw - world_min_y) * scale_y).astype(int)
    
    # Invert Y coordinate
    py = map_height - 1 - py

    # Clamp values to be within map bounds
    px = np.clip(px, 0, map_width - 1)
    py = np.clip(py, 0, map_height - 1)

    return px, py

def fft_convolve2d(image, kernel):
    # Pad the image and kernel to the same size
    pad_x = image.shape[0] + kernel.shape[0] - 1
    pad_y = image.shape[1] + kernel.shape[1] - 1
    
    padded_image = np.pad(image, ((0, pad_x - image.shape[0]), (0, pad_y - image.shape[1])), mode='constant')
    padded_kernel = np.pad(kernel, ((0, pad_x - kernel.shape[0]), (0, pad_y - kernel.shape[1])), mode='constant')

    # Perform the FFT on both image and kernel
    fft_image = np.fft.fft2(padded_image)
    fft_kernel = np.fft.fft2(padded_kernel)

    # Multiply in the frequency domain (element-wise)
    result_fft = fft_image * fft_kernel

    # Perform the inverse FFT to get the convolved image
    result = np.fft.ifft2(result_fft)
    
    # Return the real part of the convolution result
    return np.real(result)

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
kernel_size = 55
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
            if index == 0:
                break
        if index >= len(WP) - 1:
            move_forward = 0
        print(f"Reached waypoint {prev_index}, moving to waypoint {index}")

    # Compute wheel velocities with improved parameters
    # p1, p2 = 6.0, 3.0
    p1, p2 = 8.0, 3.0
    leftSpeed = -alpha * p1 + rho * p2
    rightSpeed = alpha * p1 + rho * p2
    leftMotor.setVelocity(max(min(leftSpeed, MAX_SPEED), -MAX_SPEED))
    rightMotor.setVelocity(max(min(rightSpeed, MAX_SPEED), -MAX_SPEED))


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
   
    
    # Step 1: Convert polar to Cartesian in robot's coordinate frame
    x_r = ranges * np.cos(angles)
    y_r = ranges * np.sin(angles)

    # Step 2: Apply LiDAR sensor offset
    x_r += LIDAR_OFFSET_X
    y_r += LIDAR_OFFSET_Y

    # Step 3: Transform to world coordinates
    cos_omega = np.cos(omegaz)
    sin_omega = np.sin(omegaz)

    x_w = x_r * cos_omega - y_r * sin_omega + xw
    y_w = x_r * sin_omega + y_r * cos_omega + yw

    # Step 4: Convert world coordinates to pixel indices
    px, py = world2mapv(x_w, y_w)

    # Step 5: Increment probability values (clamped to max 1)
    map[px, py] = np.minimum(1, map[px, py] + 0.01)

    # Step 6: Convert probability values to grayscale (0-255)
    v = (map[px, py] * 255).astype(int)
    color = (v * 256**2 + v * 256 + v).astype(int)  # Ensure NumPy int32 -> Python int

    # Update display pixels (vectorized loop)
    for i in range(len(px)):
        display.setColor(int(color[i]))  # Convert NumPy int to Python int
        display.drawPixel(int(px[i]), int(py[i]))  # Ensure indices are Python int


    # Plot the scan for debug
    # plt.clf()
    # # plt.scatter(x_r, y_r, s=2, label="Robot Frame", color="blue")  
    # plt.scatter(x_w, y_w, s=2, label="World Frame", color="orange")  
    # plt.legend()
    # plt.pause(0.01)

# Perform 2D convolution to compute the configuration space every 100 timesteps
# cmap = signal.convolve2d(map, kernel, mode='same')
cmap = fft_convolve2d(map, kernel)  # Use FFT-based convolution
cspace = cmap > 0.9  # Threshold to mark obstacles

# Visualize the configuration space
plt.clf()
plt.imshow(cspace, cmap='gray')
plt.title("Configuration Space")
plt.pause(1000)
