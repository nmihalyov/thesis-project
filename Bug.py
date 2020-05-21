# -*- coding: utf-8 -*-
import time
import sys
import numpy as np

import vrep
from assets import *


class Bug:
  def __init__(self, TARGET_NAME, ROBOT_NAME, ROBOT_SPEED):
    # Устанавливаем состояние движения
    self.state = States.MOVING

    # Минимальное и максимальное расстояние до препятствия
    self.MIN_DETECTION_DIST = 0
    self.MAX_DETECTION_DIST = 1

    # Название цели на сцене
    self.TARGET_NAME = TARGET_NAME
    # Название робота на сцене
    self.ROBOT_NAME = ROBOT_NAME
    # Скорость робота
    self.ROBOT_SPEED = ROBOT_SPEED
    # Отступ вокруг робота
    self.INDENT_DIST = 0.5

    # Частота обновления получаемой информации (5 Гц)
    self.SLEEP_TIME = 0.2
    # Число "пи"
    self.PI = math.pi

    # Инициализация направления, позиции и значений углов Эйлера для работа
    self.bot_dir = None
    self.bot_pos = None
    self.bot_euler_angles = None

    # Инициализация позиции цели и направления до нее
    self.target_pos = None
    self.target_dir = np.zeros(3)

    # Инициализация массива значений сенсора
    self.detect = np.zeros(16)

    # Инициализация подключения к серверу V-REP
    self._init_client_id()
    #  Инициализация хендла робота
    self._init_robot_handle()
    #  Инициализация хендлов сенсора робота
    self._init_sensor_handles()
    #  Инициализация хендлов колес робота
    self._init_wheels_handles()
    #  Инициализация хендла цели
    self._init_target_handle()

    # Инициализация частот и коэффициентов ПИД-регулятора объезда препятствия
    self.obstacle_dist_stab_PID = PIDController(50)
    self.obstacle_follower_PID = PIDController(50)
    self.obstacle_dist_stab_PID.set_coefficients(2, 0, 0.5)
    self.obstacle_follower_PID.set_coefficients(2, 0, 0)


  def _init_client_id(self):
    vrep.simxFinish(-1)

    self.client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if self.client_id != -1:
      print('Подключено к удаленному серверу API')
    else:
      sys.exit('Не удалось подключиться к серверу')


  def _init_robot_handle(self):
    error_code, self.bot_handle = vrep.simxGetObjectHandle(self.client_id, self.ROBOT_NAME, vrep.simx_opmode_oneshot_wait)


  def _init_sensor_handles(self):
    self.sensor_handles = []

    for x in range(0, 16):
      error_code, sensor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor' + str(x + 1), vrep.simx_opmode_oneshot_wait)
      self.sensor_handles.append(sensor_handle)
      vrep.simxReadProximitySensor(self.client_id, sensor_handle, vrep.simx_opmode_streaming)


  def _init_wheels_handles(self):
    error_code, self.left_motor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    error_code, self.right_motor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)


  def _init_target_handle(self):
    error_code, self.target_handle = vrep.simxGetObjectHandle(self.client_id, self.TARGET_NAME, vrep.simx_opmode_oneshot_wait)


  def _init_values(self):
    # Инициализация значений цели и робота
    error_code, _ = vrep.simxGetObjectPosition(self.client_id, self.target_handle, -1, vrep.simx_opmode_oneshot)
    error_code, _ = vrep.simxGetObjectPosition(self.client_id, self.bot_handle, -1, vrep.simx_opmode_oneshot)
    error_code, _ = vrep.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming)


  def read_values(self):
    # Чтение значений позиции цели
    error_code, target_pos = vrep.simxGetObjectPosition(self.client_id, self.target_handle, -1, vrep.simx_opmode_streaming)
    self.target_pos = Vector3(x=target_pos[0], y=target_pos[1], z=target_pos[2])

    # Чтение значений позиции робота
    error_code, bot_pos = vrep.simxGetObjectPosition(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming)
    self.bot_pos = Vector3(x=bot_pos[0], y=bot_pos[1], z=bot_pos[2])

    # Чтение значений углов Эйлера робота
    error_code, bot_euler_angles = vrep.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming)
    self.bot_euler_angles = Vector3(x=bot_euler_angles[0], y=bot_euler_angles[1], z=bot_euler_angles[2])

  def stop_move(self):
    # Остановка движения робота
    error_code = vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  0, vrep.simx_opmode_streaming)
    error_code = vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, 0, vrep.simx_opmode_streaming)

  def read_from_sensors(self):
    # Чтение значений с сенсоров
    for i in range(0, 16):
      error_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(self.client_id, self.sensor_handles[i], vrep.simx_opmode_streaming)

      # Значение дистанции до препятствия
      dist = math.sqrt(detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)

      # Присвоение значений "веса" датчиков: чем больше значение, тем ближе к конкретному датчику препятствие
      if dist < self.MIN_DETECTION_DIST:
        self.detect[i] = self.MIN_DETECTION_DIST
      elif dist > self.MAX_DETECTION_DIST or detection_state == False:
        self.detect[i] = self.MAX_DETECTION_DIST
      else:
        self.detect[i] = self.MAX_DETECTION_DIST - ((dist - self.MAX_DETECTION_DIST) / (self.MIN_DETECTION_DIST - self.MAX_DETECTION_DIST))


  def print_info(self):
    print('Цель: ' + self.TARGET_NAME)
    print('Робот: ' + self.ROBOT_NAME)
    print('Скорость: ' + str(self.ROBOT_SPEED))

  def tick(self):
    time.sleep(self.SLEEP_TIME)

  def loop(self):
    pass

  def action_moving(self):
    pass

  def action_rotating(self):
    pass

  def action_rounding(self):
    pass