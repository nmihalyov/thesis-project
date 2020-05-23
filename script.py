# -*- coding: utf-8 -*-
import time
import sys
import numpy as np

import vrep
from assets import *

class Algorithm:
  def __init__(self, ROBOT, TARGET):
    # Устанавливаем состояние движения
    self.state = States.MOVING

    # Минимальное и максимальное расстояние до препятствия
    self.MIN_DETECTION_DIST = 0
    self.MAX_DETECTION_DIST = 1

    # Название робота на сцене
    self.ROBOT = ROBOT
    # Название цели на сцене
    self.TARGET = TARGET
    # Скорость робота
    self.SPEED = 1.5
    # Отступ вокруг робота
    self.INDENT_DIST = 0.5

    # Задержка между выполнениями итераций цикла
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

    # Инициализация значения ммнимальной дистанции до цели
    self.min_dist_to_target = None

    # Вывод информации в терминал
    self.print_info()


  def _init_client_id(self):
    vrep.simxFinish(-1)

    self.client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if self.client_id != -1:
      print('Подключено к удаленному серверу API')
    else:
      sys.exit('Не удалось подключиться к серверу')


  def _init_robot_handle(self):
    error_code, self.bot_handle = vrep.simxGetObjectHandle(self.client_id, self.ROBOT, vrep.simx_opmode_oneshot_wait)


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
    error_code, self.target_handle = vrep.simxGetObjectHandle(self.client_id, self.TARGET, vrep.simx_opmode_oneshot_wait)


  def _init_values(self):
    # Инициализация значений цели и робота
    error_code, _ = vrep.simxGetObjectPosition(self.client_id, self.target_handle, -1, vrep.simx_opmode_oneshot)
    error_code, _ = vrep.simxGetObjectPosition(self.client_id, self.bot_handle, -1, vrep.simx_opmode_oneshot)
    error_code, _ = vrep.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming)


  def read_values(self):
    # Чтение значений позиции цели
    error_code, target_pos = vrep.simxGetObjectPosition(self.client_id, self.target_handle, -1, vrep.simx_opmode_streaming)
    self.target_pos = Vector3(x=target_pos[0], y=target_pos[1], z=0)

    # Чтение значений позиции робота
    error_code, bot_pos = vrep.simxGetObjectPosition(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming)
    self.bot_pos = Vector3(x=bot_pos[0], y=bot_pos[1], z=0)

    # Чтение значений углов Эйлера робота
    error_code, bot_euler_angles = vrep.simxGetObjectOrientation(self.client_id, self.bot_handle, -1, vrep.simx_opmode_streaming)
    self.bot_euler_angles = Vector3(x=0, y=0, z=bot_euler_angles[2])


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

      # Присвоение значений датчиков: чем больше значение, тем ближе к конкретному датчику препятствие
      if dist < self.MIN_DETECTION_DIST:
        self.detect[i] = self.MIN_DETECTION_DIST
      elif dist > self.MAX_DETECTION_DIST or detection_state == False:
        self.detect[i] = self.MAX_DETECTION_DIST
      else:
        self.detect[i] = self.MAX_DETECTION_DIST - ((dist - self.MAX_DETECTION_DIST) / (self.MIN_DETECTION_DIST - self.MAX_DETECTION_DIST))


  def print_info(self):
    print('Цель: ' + self.TARGET)
    print('Робот: ' + self.ROBOT)
    print('Скорость: ' + str(self.SPEED))


  def tick(self):
    # Задержка программы
    time.sleep(self.SLEEP_TIME)


  def loop(self):
    self._init_values()
    self.read_values()

    # Основной цикл программы
    while Utils.distance_between_points(self.bot_pos, self.target_pos) > 0.4 or Utils.distance_between_points(self.bot_pos, self.target_pos) == 0:
      self.tick()
      # print(Utils.distance_between_points(self.bot_pos, self.target_pos))

      # Останавливаем движение и читаем данные робота и цели
      self.stop_move()
      self.read_values()

      # Получаем данные с сенсоров
      self.read_from_sensors()

      # Инициализируем новый кватернион
      q_rot = Quaternion()
      # Получаем кватернион из угла разворота вокруг вектора и самого вектора соответственно
      q_rot.set_from_vector(self.bot_euler_angles.z, Vector3(0, 0, 1))
      # Присваиваем вектору робота значения поворота вектора кватернионом
      self.bot_dir = q_rot.rotate(Vector3(1, 0, 0))

      # Проверка текущего состояния и выполнение соответствующей функции
      if self.state == States.MOVING:
        self.action_moving()
      elif self.state == States.ROTATING:
        self.action_rotating()
      elif self.state == States.ROUNDING:
        self.action_rounding()

    print('Цель достигнута')
    self.stop_move()


  def action_moving(self):
    # Если робот оказалася близко к препятствию, объезжаем это препятствие, иначе продолжаем движение до цели
    if (self.detect[4] + self.detect[5])/2 < self.INDENT_DIST:
      # Инициализируем новый кватернион
      q = Quaternion()
      # Получаем кватернион из угла разворота вокруг вектора и самого вектора соответственно
      q.set_from_vector(self.PI/2, Vector3(0, 0, 1))
      # Присваиваем вектору цели значения поворота вектора направления робота кватернионом
      self.target_dir = q.rotate(self.bot_dir)
      # Переключаем состояние
      self.state = States.ROTATING
    else:
      # Значение угла в радианах между векторами направления движения робота и между роботом и целью
      angle = Utils.angle_between_vectors(self.bot_dir, self.target_pos.minus(self.bot_pos))

      # Корректируем движение робота до прямолинейного до цели, если необходимо
      if math.fabs(angle) > self.PI/180:
        vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.SPEED + angle, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.SPEED - angle, vrep.simx_opmode_streaming)
      else:
        vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle,  self.SPEED, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.SPEED, vrep.simx_opmode_streaming)


  def action_rotating(self):
    # Значение угла в радианах между векторами направления движения робота и цели
    angle = Utils.angle_between_vectors(self.bot_dir, self.target_dir)

    # Поворачиваем робота вокруг своей оси, если необходимо, иначе переключаем состояние
    if math.fabs(angle) > 5 * self.PI/180:
      vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle, angle, vrep.simx_opmode_streaming)
      vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, -angle, vrep.simx_opmode_streaming)
    else:
      self.state = States.ROUNDING


  def action_rounding(self):
    # Инициализация нового кватерниона
    q = Quaternion()
    # Получаем кватернион из угла разворота вокруг вектора и самого вектора соответственно
    q.set_from_vector(self.PI/2, Vector3(0, 0, 1))
    # Присваиваем перпендикуляру вектора робота значения поворота вектора направления робота кватернионом
    perp_bot_dir = q.rotate(self.bot_dir)

    # Значение угла в радианах между векторами перпендикуляра к роботу и между роботом и целью
    angle = Utils.angle_between_vectors(perp_bot_dir, self.target_pos.minus(self.bot_pos))

    # Текущая дистанция до цели
    current_dist_to_target = Utils.distance_between_points(self.bot_pos, self.target_pos)

    # Если минимальное значение дистанции до цели меньше, чем текущее, то присваиваем минимальному значению текущее
    if self.min_dist_to_target == None or self.min_dist_to_target <= current_dist_to_target:
      self.min_dist_to_target = current_dist_to_target
    # Прекращаем объезд препятствия
    elif math.fabs(angle) < 5.0 / 180 * self.PI:
      self.state = States.MOVING
      return

    # Разница между значениями двух крайних справа датчиков
    delta = self.detect[7] - self.detect[8]

    # Устанавливаем расстояние до препятствия
    obstacle_dist = min([self.detect[7], self.detect[8]]) - self.INDENT_DIST

    obstacle_dist_stab = self.obstacle_dist_stab_PID.output(obstacle_dist)
    obstacle_follower = self.obstacle_follower_PID.output(delta)
  
    vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle, self.SPEED + obstacle_follower + obstacle_dist_stab - (1 - (self.detect[4] + self.detect[5])/2), vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, self.SPEED - obstacle_follower - obstacle_dist_stab + (1 - (self.detect[4] + self.detect[5])/2), vrep.simx_opmode_streaming)


bug = Algorithm(ROBOT='P3DX', TARGET='Destination')

bug.loop()