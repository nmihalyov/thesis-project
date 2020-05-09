import sim
import sys
import time
import numpy as np
import math

PI = math.pi
MINIMUM_DISTANCE = 0.3
VELOCITY = 1
GAIN = 0.5

ROTATION_VELOCITY = 0.2

EULER_ANGLES_RANGE = [-3.14, 3.14]

COORDINATES = [1.7516592741012573, -5.5094260692596436, 0.13866472244262695]

# Функция вращения вокруг оси до направления к координате
def rotate(destination, current_coords, robot_handle):
  errorCode, orient = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_oneshot_wait)
  rotate_direction = min(EULER_ANGLES_RANGE, key=lambda rotate_direction:abs(rotate_direction-orient[2]))

  rotate = True
  while rotate:
    angle = round(math.atan2(destination[1] - current_coords[1], destination[0] - current_coords[0]), 2)
    errorCode, orient = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)

    if angle != round(orient[2], 2):
      if rotate_direction > 0:
        velocity_left = -ROTATION_VELOCITY
        velocity_right = ROTATION_VELOCITY
      else:
        velocity_left = ROTATION_VELOCITY
        velocity_right = -ROTATION_VELOCITY
    else:
      velocity_left = 0
      velocity_right = 0
      rotate = False
    errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, velocity_left, sim.simx_opmode_streaming)
    errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, velocity_right, sim.simx_opmode_streaming)
    time.sleep(0.0001)

# Функция объезда препятствий
def avoidObstacle():
  sensor_val = np.array([])

  for x in range(0, 16):
    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensor_handles[x], sim.simx_opmode_buffer)
    # Список значений с сенсоров
    sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))

  # Квадрат значений
  sensor_sq = sensor_val[0:8] * sensor_val[0:8]
      
  min_ind = np.where(sensor_sq == np.min(sensor_sq))[0][0]

  if sensor_sq[min_ind] < MINIMUM_DISTANCE:
    steer = -1/sensor_loc[min_ind]
  else:
    steer = 0

  velocity_left = VELOCITY + GAIN * steer
  velocity_right = VELOCITY - GAIN * steer

  return velocity_left, velocity_right

# Функция движения к заданой координате
def getToCoordinate(coords):
  errorCode, robot_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_visible', sim.simx_opmode_oneshot_wait)
  errorCode, current_coords = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_oneshot_wait)
  destination = [round(coords[0], 2), round(coords[1], 2), round(coords[2], 2)]

  rotate(destination, current_coords, robot_handle)

  arrived = False
  while not arrived:
    errorCode, current_coords = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)

    if destination[0] != round(current_coords[0], 2):
      velocity_left, velocity_right = avoidObstacle()
    else:
      velocity_left = 0
      velocity_right = 0
      arrived = True

    errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, velocity_left, sim.simx_opmode_streaming)
    errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, velocity_right, sim.simx_opmode_streaming)
    time.sleep(0.05)


print('Program started')

# Закрытие всех подключений
sim.simxFinish(-1)

# Подключение к CoppeliaSim
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
  print('Connection successful')
else:
  sys.exit('Failed connecting to remote API server')

# Получение левого и правого моторов
errorCode, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)

# Пустой массив сенсоров
sensor_handles = []

# Пустой массив значений сенсоров
sensor_val = np.array([]) 

# Ориентации сенсоров
sensor_loc = np.array([-PI/2, -50/180.0*PI, -30/180.0*PI, -10/180.0*PI, 10/180.0*PI, 30/180.0*PI, 50/180.0*PI, PI/2, PI/2, 130/180.0*PI, 150/180.0*PI, 170/180.0*PI, -170/180.0*PI, -150/180.0*PI, -130/180.0*PI, -PI/2])

# Цикл получения массивов сенсоров и их инициализации
for x in range(0, 16):
  errorCode, sensor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(x + 1), sim.simx_opmode_oneshot_wait)
  sensor_handles.append(sensor_handle)
  errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensor_handle, sim.simx_opmode_streaming)                
  # Получение списка значений
  sensor_val = np.append(sensor_val, np.linalg.norm(detectedPoint))

getToCoordinate(COORDINATES)