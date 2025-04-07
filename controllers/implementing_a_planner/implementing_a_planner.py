"""implementing_a_planner controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)



def getNeighbors(position, grid, threshold=0.3):
    i, j = position
    rows, cols = len(grid), len(grid[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right
    neighbors = []

    for di, dj in directions:
        ni, nj = i + di, j + dj
        # Check boundaries
        if 0 <= ni < rows and 0 <= nj < cols:
            if grid[ni][nj] < threshold:
                neighbors.append((ni, nj))
    
    return neighbors

map_data = [
    [0, 0, 0],
    [0.9, 0.8, 0],
    [0, 0, 0]
]

print(getNeighbors((0, 0), map_data))  # Output: [(0, 1)]
print(getNeighbors((0, 1), map_data))  # Output: [(0, 0), (0, 2)]
print(getNeighbors((0, 2), map_data))  # Output: [(0, 1), (1, 2)]
print(getNeighbors((1, 2), map_data))  # Output: [(0, 2), (2, 2)]
print(getNeighbors((2, 0), map_data))  # Output: [(2, 1)]
print(getNeighbors((2, 1), map_data))  # Output: [(2, 0), (2, 2)]
print(getNeighbors((2, 2), map_data))  # Output: [(2, 1), (1, 2)]






# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
