# Add the controller Webots Python library path
import sys
webots_path = 'C:\Program Files\Webots\lib\controller\python'
sys.path.append(webots_path)

# Add other libraries
import math
from controller import Robot


# Get robot's heading in degree based on compass values
def get_robot_heading(compass_value):
    rad = math.atan2(compass_value[0], compass_value[1])
    bearing = (rad - 1.5708) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360.0
    
    heading = 360 - bearing
    if heading > 360.0:
        heading -= 360.0
    return heading


# Create an instance of robot
robot = Robot()


# Load Devices such as sensors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Set the motors to rotate for ever
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
# But with no velocity :)
left_motor.setVelocity(0)
right_motor.setVelocity(0)

sampling_period = 1 # in ms
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
# Enables the devices
gps.enable(sampling_period)
compass.enable(sampling_period)

robot.step(1000) # take some dummy steps in environment for safe initialization of the initial heading and position
initial_gps_value = gps.getValues()
initial_compass_value = compass.getValues()

# General parameters
max_speed = 6.  # Angular speed in rad/s
destination_coordinate = [-0.209, -0.249] # Target position 
distance_threshold = 0.008  # in meters
angle_threshold = 4 # angle degree threshold to check if heading is in range

# degree to target using direction vector
direction_vector = [destination_coordinate[0] - initial_gps_value[0], 
                    destination_coordinate[1] - initial_gps_value[1]]
degree_to_target = math.atan2(direction_vector[0], direction_vector[1]) * 180 / math.pi
degree_to_target = round(degree_to_target) % 360

# initial degree of robot
initial_degree = round(get_robot_heading(initial_compass_value))

# how much initial degree differs from target angle
degree_diff = abs(initial_degree - degree_to_target)
print(f'Target Angle: {degree_to_target}, Robot Heading: {initial_degree}')



rotate_right = True if degree_to_target <= 180 else False # for taking the shorter rotation
# Rotate the robot to face the target angle
while True:
    current_degree = get_robot_heading(compass.getValues()) # current robot angle
    diff_current_vs_initial_degree = abs(current_degree - degree_to_target) # difference of angle bettwen intial degree vs current degree of robot heading
    if rotate_right:
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(-max_speed)
    else:
        left_motor.setVelocity(-max_speed)
        right_motor.setVelocity(max_speed)
    robot.step()
    
    # If condition is met break the loop
    if diff_current_vs_initial_degree < angle_threshold:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        robot.step(1000)  
        print('Correct Heading.')
        break
    

# Move the robot to the destination
print('Moving to the target location...')
while True:
    current_coordinate = gps.getValues() # current robot position via gps sensor
    distance_to_target_x = abs(current_coordinate[0] - destination_coordinate[0]) # difference in x of current vs destination position
    distance_to_target_y = abs(current_coordinate[1] - destination_coordinate[1]) # difference in y of current vs destination position
    reach_condition = distance_to_target_x and distance_to_target_y < distance_threshold # did robot reach the target location?
    
    # Check if the robot is already at the destination
    if reach_condition:
        print('Destination Reached.')
    else:
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)
    robot.step()
    
    # if reached break the loop
    if reach_condition:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        robot.step(1000)  
        print('Finished.')
        break