import sim
import sys
import time
import numpy as np
import math

PI = math.pi
MINIMUM_DISTANCE = 0.3
VELOCITY = 1
GAIN = 0.5

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


while True:
  sensor_val = np.array([])

  for x in range(0, 16):
    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensor_handles[x], sim.simx_opmode_buffer)
    # Get list of values
    sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))

  # Square the values of front-facing sensors 1-8
  sensor_sq = sensor_val[0:8] * sensor_val[0:8]
      
  min_ind = np.where(sensor_sq == np.min(sensor_sq))[0][0]

  if sensor_sq[min_ind] < MINIMUM_DISTANCE:
    steer = -1/sensor_loc[min_ind]
  else:
    steer = 0

  velocity_left = VELOCITY + GAIN * steer
  velocity_right = VELOCITY - GAIN * steer
  print('Left velocity: ', velocity_left)
  print('Right velocity: ', velocity_right)
  print('')

  # Send target velocity to the robot
  errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, velocity_left, sim.simx_opmode_streaming)
  errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, velocity_right, sim.simx_opmode_streaming)

  # Execute loop once every 0.2 seconds (= 5 Hz)
  time.sleep(0.2)