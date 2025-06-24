import math
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor

TIRE_DIAMETER: float = 100.0
WHEEL_TREAD: float = 120.0

class Plotter(object):
    def __init__(self) -> None:
        self.running = False
        self.distance = 0.0
        self.azimuth = 0.0
        self.loc_x = 0.0
        self.loc_y = 0.0

    def plot(
        self,
        hub: Hub,
        arm_motor: Motor,
        right_motor: Motor,
        left_motor: Motor,
        touch_sensor: TouchSensor,
        color_sensor: ColorSensor,
        sonar_sensor: SonarSensor,
    ) -> None:
        if not self.running:
            self.running = True
            right_motor.reset_count()
            left_motor.reset_count()
            self.prev_ang_r = right_motor.get_count()
            self.prev_ang_l = left_motor.get_count()
            return

        # accumulate distance
        cur_ang_r = right_motor.get_count()
        cur_ang_l = left_motor.get_count()
        delta_dist_r = math.pi * TIRE_DIAMETER * (cur_ang_r - self.prev_ang_r) / 360.0
        delta_dist_l = math.pi * TIRE_DIAMETER * (cur_ang_l - self.prev_ang_l) / 360.0
        delta_dist = (delta_dist_r + delta_dist_l / 2.0)
        if (delta_dist >= 0.0):
            self.distance += delta_dist
        else:
            self.distance -= delta_dist
        self.prev_ang_r = cur_ang_r
        self.prev_ang_l = cur_ang_l
        # calculate azimuth
        delta_azi =  (delta_dist_l - delta_dist_r) / WHEEL_TREAD
        self.azimuth += delta_azi
        if (self.azimuth > 2*math.pi):
            self.azimuth -= 2*math.pi
        elif (self.azimuth < 0.0):
            self.azimuth += 2*math.pi
        # estimate location
        self.loc_x += delta_dist * math.sin(self.azimuth)
        self.loc_y += delta_dist * math.cos(self.azimuth)
        return

    def get_distance(self) -> int:
        return int(self.distance)

    def get_azimuth(self) -> int:
        return int(self.azimuth)

    def get_degree(self) -> int:
        degree = int(360.0 * self.azimuth / 2.0 / math.pi)
        if (degree == 360):
            degree = 0
        return degree

    def get_loc_x(self) -> int:
        return int(self.loc_x)

    def get_loc_y(self) -> int:
        return int(self.loc_y)
    
