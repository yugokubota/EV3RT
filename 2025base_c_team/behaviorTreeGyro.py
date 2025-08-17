import argparse
from enum import IntEnum, Enum
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor
from simple_pid import PID
from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.composites import Parallel
from py_trees.common import ParallelPolicy
from py_trees import (
    display as display_tree,
    logging as log_tree
)
from py_etrobo_util import TraceSide, Plotter, SymmetricClamper

EXEC_INTERVAL: float = 0.04
ARM_SHIFT_PWM = 30
MAX_POWER = 100
MIN_POWER = 60

class ArmDirection(IntEnum):
    UP = -1
    DOWN = 1

class HeadingType(Enum):
    ABSOLUTE = "absolute"
    RELATIVE = "relative"

g_plotter: Plotter = None
g_hub: Hub = None
g_arm_motor: Motor = None
g_right_motor: Motor = None
g_left_motor: Motor = None
g_touch_sensor: TouchSensor = None
g_color_sensor: ColorSensor = None
g_sonar_sensor: SonarSensor = None
g_gyro_sensor: GyroSensor = None
g_course: int = 0


class TheEnd(Behaviour):
    def __init__(self, name: str):
        super(TheEnd, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.behavior tree exhausted. ctrl+C shall terminate the program" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.RUNNING


class ResetDevice(Behaviour):
    def __init__(self, name: str):
        super(ResetDevice, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.count = 0

    def update(self) -> Status:
        if self.count == 0:
            g_arm_motor.reset_count()
            g_right_motor.reset_count()
            g_left_motor.reset_count()
            g_gyro_sensor.reset()
            self.logger.info("%+06d %s.resetting..." % (g_plotter.get_distance(), self.__class__.__name__))
        elif self.count > 3:
            self.logger.info("%+06d %s.complete" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        self.count += 1
        return Status.RUNNING


class ArmUpDownFull(Behaviour):
    def __init__(self, name: str, direction: ArmDirection):
        super(ArmUpDownFull, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.direction = direction
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.prev_degree = g_arm_motor.get_count()
            self.count = 0
        else:
            cur_degree = g_arm_motor.get_count()
            if abs(cur_degree - self.prev_degree) < 5:
                if self.count > 10:
                    g_arm_motor.set_power(0)
                    g_arm_motor.set_brake(True)
                    self.logger.info("%+06d %s.position set to %d" % (g_plotter.get_distance(), self.__class__.__name__, cur_degree))
                    return Status.SUCCESS
                else:
                    self.count += 1
            self.prev_degree = cur_degree
        g_arm_motor.set_power(ARM_SHIFT_PWM * self.direction)
        return Status.RUNNING


class IsDistanceEarned(Behaviour):
    def __init__(self, name: str, delta_dist: int):
        super(IsDistanceEarned, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.delta_dist = delta_dist
        self.running = False
        self.earned = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.orig_dist = g_plotter.get_distance()
            self.logger.info("%+06d %s.accumulation started for delta=%d" % (self.orig_dist, self.__class__.__name__, self.delta_dist))
        cur_dist = g_plotter.get_distance()
        earned_dist = cur_dist - self.orig_dist
        if (earned_dist >= self.delta_dist or -earned_dist <= -self.delta_dist):
            if not self.earned:
                self.earned = True
                self.logger.info("%+06d %s.delta distance earned" % (cur_dist, self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class IsSonarOn(Behaviour):
    def __init__(self, name: str, alert_dist: int):
        super(IsSonarOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.alert_dist = alert_dist
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.detection started for dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, self.alert_dist))
        
        dist = g_sonar_sensor.get_distance()
        if (dist <= self.alert_dist and dist > 0):
            self.logger.info("%+06d %s.alerted at dist=%d" % (g_plotter.get_distance(), self.__class__.__name__, dist))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class IsTouchOn(Behaviour):
    def __init__(self, name: str):
        super(IsTouchOn, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        if g_touch_sensor.is_pressed():
            self.logger.info("%+06d %s.pressed" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class StopNow(Behaviour):
    def __init__(self, name: str):
        super(StopNow, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        g_right_motor.set_power(0)
        g_right_motor.set_brake(True)
        g_left_motor.set_power(0)
        g_left_motor.set_brake(True)
        self.logger.info("%+06d %s.motors stopped" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.SUCCESS


class TraceLine(Behaviour):
    def __init__(self, name: str, target: int, power: int, pid_p: float, pid_i: float, pid_d: float,
                 trace_side: TraceSide) -> None:
        super(TraceLine, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        self.trace_side = trace_side
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))
        if self.trace_side == TraceSide.NORMAL:
            turn = (-1) * g_course * int(self.pid(g_color_sensor.get_brightness()))
        else: # TraceSide.OPPOSITE
            turn = g_course * int(self.pid(g_color_sensor.get_brightness()))
        g_right_motor.set_power(self.power - turn)
        g_left_motor.set_power(self.power + turn)
        return Status.RUNNING


class SpinAround(Behaviour):
    def __init__(self, name: str, target: int, max_power: int, min_power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(SpinAround, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.clamper = SymmetricClamper(min_power, max_power)
        self.running = False

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            self.running = True
            self.logger.info("%+06d %s.spin started at heading=%d for %d" % (g_plotter.get_distance(),
                                                                             self.__class__.__name__, current_heading, self.target_heading))
        error = float(self.target_heading) - current_heading
        # normalize error to [-180, 180]
        if error > 180.0:
            error -= 360.0
        if error < -180.0:
            error += 360.0
        if abs(error) < 2.0:
            self.logger.info("%+06d %s.spin ended at heading=%d" % (g_plotter.get_distance(),
                                                                    self.__class__.__name__, current_heading))
            return Status.SUCCESS
        power = int(self.clamper.clamp(self.pid(current_heading)))
        g_right_motor.set_power(g_course * power)
        g_left_motor.set_power((-1) * g_course * power)
        return Status.RUNNING    


class RunByGyro(Behaviour):
    def __init__(self, name: str, target: int, power: int,
                 pid_p: float, pid_i: float, pid_d: float, target_type: HeadingType) -> None:
        super(RunByGyro, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.power = power
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.running = False

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL, output_limits=(-self.power, self.power))
            self.running = True
            self.logger.info("%+06d %s.gyro run started toward heading=%d" % (g_plotter.get_distance(),
                                                                              self.__class__.__name__, self.target_heading))
        turn = int(self.pid(current_heading))
        g_right_motor.set_power(self.power - turn)
        g_left_motor.set_power(self.power + turn)
        return Status.RUNNING


class TraverseBehaviourTree(object):
    def __init__(self, tree: BehaviourTree) -> None:
        self.tree = tree
        self.running = False
    def __call__(
        self,
        **kwargs,
    ) -> None:
        global g_plotter
        if not self.running:
            if g_hub is None:
                print(" -- TraverseBehaviorTree waiting for ETrobo devices to be exposed...")
            else:
                self.running = True
                g_plotter = Plotter()
                print(" -- TraverseBehaviorTree initialization complete")
        else:
            self.tree.tick_once()
            g_plotter.plot(**kwargs)


class ExposeDevices(object):
    def __call__(
        self,
        hub: Hub,
        arm_motor: Motor,
        right_motor: Motor,
        left_motor: Motor,
        touch_sensor: TouchSensor,
        color_sensor: ColorSensor,
        sonar_sensor: SonarSensor,
        gyro_sensor: GyroSensor,
    ) -> None:
        global g_hub, g_arm_motor, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor, g_sonar_sensor, g_gyro_sensor
        g_hub = hub
        g_arm_motor = arm_motor
        g_right_motor = right_motor
        g_left_motor = left_motor
        g_touch_sensor = touch_sensor
        g_color_sensor = color_sensor
        g_sonar_sensor = sonar_sensor
        g_gyro_sensor = gyro_sensor


def build_behaviour_tree() -> BehaviourTree:
    root = Sequence(name="loop by color sensor", memory=True)
    calibration = Sequence(name="calibration", memory=True)
    start = Parallel(name="start", policy=ParallelPolicy.SuccessOnOne())
    edge_01 = Parallel(name="run by gyro and distance", policy=ParallelPolicy.SuccessOnOne())
    edge_02 = Parallel(name="run by gyro and distance", policy=ParallelPolicy.SuccessOnOne())
    edge_03 = Parallel(name="run by gyro and distance", policy=ParallelPolicy.SuccessOnOne())
    edge_04 = Parallel(name="run by gyro and distance", policy=ParallelPolicy.SuccessOnOne())
    square = Sequence(name="square", memory=True)
    calibration.add_children(
        [
            ArmUpDownFull(name="arm up", direction=ArmDirection.UP),
            ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
            ResetDevice(name="device reset"),
        ]
    )
    start.add_children(
        [
            IsSonarOn(name="soner start", alert_dist=90),
            IsTouchOn(name="touch start"),
        ]
    )
    edge_01.add_children(
        [
            RunByGyro(name="run straight", target=90, power=70,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 1500),
        ]
    )
    edge_02.add_children(
        [
            RunByGyro(name="run straight", target=180, power=70,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 1500),
        ]
    )
    edge_03.add_children(
        [
            RunByGyro(name="run straight", target=270, power=70,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 1500),
        ]
    )
    edge_04.add_children(
        [
            RunByGyro(name="run straight", target=360, power=70,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            IsDistanceEarned(name="check distance", delta_dist = 1500),
        ]
    )
    square.add_children(
        [
            SpinAround(name="spin by 90 degrees", target=90, max_power=55, min_power=MIN_POWER,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            edge_01,
            SpinAround(name="spin by 90 degrees", target=180, max_power=55, min_power=MIN_POWER,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            edge_02,
            SpinAround(name="spin by 90 degrees", target=270, max_power=55, min_power=MIN_POWER,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            edge_03,
            SpinAround(name="spin by 90 degrees", target=360, max_power=55, min_power=MIN_POWER,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
            edge_04,
            SpinAround(name="spin by 90 degrees", target=450, max_power=55, min_power=MIN_POWER,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        ]
    )
    root.add_children(
        [
            calibration,
            start,
            square,
            StopNow(name="stop"),
            TheEnd(name="end"),
        ]
    )
    return root

def initialize_etrobo(backend: str) -> ETRobo:
    return (ETRobo(backend=backend)
            .add_hub('hub')
            .add_device('arm_motor', device_type=Motor, port='C')
            .add_device('right_motor', device_type=Motor, port='A')
            .add_device('left_motor', device_type=Motor, port='B')
            .add_device('touch_sensor', device_type=TouchSensor, port='D')
            .add_device('color_sensor', device_type=ColorSensor, port='E')
            .add_device('sonar_sensor', device_type=SonarSensor, port='F')
            .add_device('gyro_sensor', device_type=GyroSensor, port='',
                        config=[2.0, 2500.0,
                        [-0.239569, -2.50881, 0.6617843], [361.9036, 355.9302, 361.8885],
                        [10089.76, -9720.13, 9931.442, -9704.719, 9522.367, -10210.74]])
            )

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('course', choices=['right', 'left'], help='Course to run')
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args()

    if args.course == 'right':
        g_course = -1
    else:
        g_course = 1

    #py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = build_behaviour_tree()
    display_tree.render_dot_tree(tree)
    
    etrobo = initialize_etrobo(backend='raspike_art')
    etrobo.add_handler(ExposeDevices())
    etrobo.add_handler(TraverseBehaviourTree(tree))
    etrobo.dispatch(interval=EXEC_INTERVAL, logfile=args.logfile)