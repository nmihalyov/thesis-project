import sim
import sys
import time
import numpy as np
import math

PI = math.pi
MINIMUM_DISTANCE = 0.3
VELOCITY = 1
GAIN = 0.5

COORDINATES = [-1.2516592741012573, -2.3094260692596436, 0.13866472244262695]

def toFixed(f: float, n=0):
    a, b = str(f).split('.')
    return float('{}.{}{}'.format(a, b[:n], '0'*(n-len(b))))

# Function to get robot to spicified coordinates
def getToSpecificCoordinate(coords, left_motor_handle, right_motor_handle):
  errorCode, robot_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_visible', sim.simx_opmode_oneshot_wait)
  errorCode, current_coords = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_oneshot_wait)
  destination = [toFixed(coords[0], 2), toFixed(coords[1], 2), toFixed(coords[2], 2)]

  # print(orient)
  rotate = True
  while rotate:
    angle = toFixed(math.atan2(destination[1] - toFixed(current_coords[0], 2), destination[0] - toFixed(current_coords[0], 2)), 1)
    errorCode, orient = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_oneshot_wait)
    print('Angle:', angle)
    print('Orient:', toFixed(orient[2], 1))
    if angle != toFixed(orient[2], 1):
      velocity_left = 1
      velocity_right = -1
    else:
      velocity_left = 0
      velocity_right = 0
      rotate = False
    errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, velocity_left, sim.simx_opmode_streaming)
    errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, velocity_right, sim.simx_opmode_streaming)
    time.sleep(0.05)
  print(current_coords)
  print('Robot X:', toFixed(current_coords[0], 2))
  print('Robot Y:', toFixed(current_coords[1], 2))
  print('')
  print('Destintion X:', destination[0])
  print('Destintion Y:', destination[1])
  print('')
  # print('Angle: ', angle)

  arrived = False
  while not arrived:
    errorCode, current_coords = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
    print('Current coordinates:', toFixed(current_coords[0], 2), toFixed(current_coords[1], 2), toFixed(current_coords[2], 2))
    print('Destination:', destination[0], destination[1], toFixed(coords[2], 2))
    print('')

    # velocity_left = VELOCITY + GAIN * steer
    # velocity_right = VELOCITY - GAIN * steer
    if destination[0] != toFixed(current_coords[0], 2):
      velocity_left = 1
      velocity_right = 1
    else:
      velocity_left = 0
      velocity_right = 0
      arrived = True
      print('Robot has arrived destination')

    errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, velocity_left, sim.simx_opmode_streaming)
    errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, velocity_right, sim.simx_opmode_streaming)
    time.sleep(0.05)

print('Program started')

# Close all opened connections
sim.simxFinish(-1)

# Connect to CoppeliaSim
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
  print('Connection successful')
else:
  sys.exit('Failed connecting to remote API server')

# Retrieve left & right motor handles
errorCode, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

# Empty list for handles
sensor_handles = []

# Empty array for sensor measurements
sensor_val = np.array([]) 

# Orientation of all the sensors
sensor_loc = np.array([-PI/2, -50/180.0*PI, -30/180.0*PI, -10/180.0*PI, 10/180.0*PI, 30/180.0*PI, 50/180.0*PI, PI/2, PI/2, 130/180.0*PI, 150/180.0*PI, 170/180.0*PI, -170/180.0*PI, -150/180.0*PI, -130/180.0*PI, -PI/2])

# A loop to retrieve sensor arrays and initiate sensors
for x in range(0, 16):
  errorCode, sensor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x + 1), sim.simx_opmode_oneshot_wait)
  sensor_handles.append(sensor_handle)
  errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_streaming)                
  # Get list of value
  sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))

getToSpecificCoordinate(COORDINATES, left_motor_handle, right_motor_handle)

# while True:
#   sensor_val = np.array([])

#   for x in range(0, 16):
#     errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensor_handles[x], sim.simx_opmode_buffer)
#     # Get list of values
#     sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))

#   # Square the values of front-facing sensors 1-8
#   sensor_sq = sensor_val[0:8] * sensor_val[0:8]
      
#   min_ind = np.where(sensor_sq == np.min(sensor_sq))[0][0]

#   if sensor_sq[min_ind] < MINIMUM_DISTANCE:
#     steer = -1/sensor_loc[min_ind]
#   else:
#     steer = 0

#   velocity_left = VELOCITY + GAIN * steer
#   velocity_right = VELOCITY - GAIN * steer
#   # print('Left velocity: ', velocity_left)
#   # print('Right velocity: ', velocity_right)
#   # print('')

#   # Send target velocity to the robot
  # errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, velocity_left, sim.simx_opmode_streaming)
  # errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, velocity_right, sim.simx_opmode_streaming)

#   # Execute loop once every 0.1 seconds (= 10 Hz)
#   time.sleep(0.1)