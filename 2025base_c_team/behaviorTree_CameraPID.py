import argparse
import time
import math
import threading
import signal
from enum import Enum, IntEnum, auto
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor
from simple_pid import PID
import py_trees.common
from py_trees.trees import BehaviourTree
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Parallel, Selector
from py_trees.common import ParallelPolicy
from py_trees import (
    display as display_tree,
    logging as log_tree
)
from py_etrobo_util import Video, TraceSide, Plotter
from py_etrobo_util.plotter import TIRE_DIAMETER

EXEC_INTERVAL: float = 0.02
VIDEO_INTERVAL: float = 0.02
ARM_SHIFT_PWM = 30
JUNCT_UPPER_THRESH = 50
JUNCT_LOWER_THRESH = 30

class ArmDirection(IntEnum):
    UP = -1
    DOWN = 1

class JState(Enum):
    INITIAL = auto()
    JOINING = auto()
    JOINED = auto()
    FORKING = auto()
    FORKED = auto()

class Color(Enum):
    JETBLACK = auto()
    BLACK = auto()
    BLUE = auto()
    RED = auto()
    YELLOW = auto()
    GREEN = auto()
    WHITE = auto()

g_plotter: Plotter = None
g_hub: Hub = None
g_arm_motor: Motor = None
g_right_motor: Motor = None
g_left_motor: Motor = None
g_touch_sensor: TouchSensor = None
g_color_sensor: ColorSensor = None
g_sonar_sensor: SonarSensor = None
g_video: Video = None
g_video_thread: threading.Thread = None
g_course: int = 0


class TheEnd(Behaviour):
# ctl+cで処理を終了させるようにしている
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
# ロボットのモーターの回転数をリセットする専用の「初期化ビヘイビア」
    def __init__(self, name: str):
        super(ResetDevice, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.count = 0

    def update(self) -> Status:
        if self.count == 0:
            g_arm_motor.reset_count()
            g_right_motor.reset_count()
            g_left_motor.reset_count()
            self.logger.info("%+06d %s.resetting..." % (g_plotter.get_distance(), self.__class__.__name__))
        elif self.count > 3:
            self.logger.info("%+06d %s.complete" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        self.count += 1
        return Status.RUNNING


class ArmUpDownFull(Behaviour):
# アームを上げ下げして初期化するビヘイビア
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
            if cur_degree == self.prev_degree:
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
# ロボットがある距離だけ進んだかどうかをチェックするビヘイビア
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
            return Status.FAILURE


class IsSonarOn(Behaviour):
# 障害物が近くにある場合に次の行動を制御できる
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


class IsJunction(Behaviour):
# 分岐チェックを知らせるだけのクラス
    def __init__(self, name: str, target_state: JState) -> None:
        super(IsJunction, self).__init__(name)
        self.target_state = target_state
        self.reached = False
        self.prev_roe = 0
        self.state:JState = JState.INITIAL
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.scan started" % (g_plotter.get_distance(), self.__class__.__name__))
        roe = g_video.get_range_of_edges()
        if roe != 0:
            if self.state == JState.INITIAL:
                if (self.target_state == JState.JOINING or self.target_state == JState.JOINED) and roe >= JUNCT_UPPER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.lines are joining" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.JOINING
                elif (self.target_state == JState.FORKING or self.target_state == JState.FORKED) and roe >= JUNCT_LOWER_THRESH and self.prev_roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.lines are forking" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.FORKING
            elif self.state == JState.JOINING:
                if roe <= JUNCT_LOWER_THRESH:
                    self.logger.info("%+06d %s.the join completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.JOINED
                    
            elif self.state == JState.FORKING:
                if roe <= JUNCT_LOWER_THRESH and self.prev_roe >= JUNCT_UPPER_THRESH:
                    self.logger.info("%+06d %s.the fork completed" % (g_plotter.get_distance(), self.__class__.__name__))
                    self.state = JState.FORKED
            else:
                pass
        self.prev_roe = roe

        if not self.reached and self.state == self.target_state:
            self.reached = True
            self.logger.info("%+06d %s.target state reached" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        else:
            return Status.RUNNING


class RunAsInstructed(Behaviour):
# ロボットの左右のモーターに固定のPWM（出力）を与えて動かす「行動ノード」
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:
        super(RunAsInstructed, self).__init__(name)
        self.pwm_l = g_course * pwm_l
        self.pwm_r = g_course * pwm_r
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.started with pwm=(%s, %s)" % (g_plotter.get_distance(), self.__class__.__name__, self.pwm_l, self.pwm_r))
        g_right_motor.set_power(self.pwm_r)
        g_left_motor.set_power(self.pwm_l)
        return Status.RUNNING


class TraceLine_sensor(Behaviour):
    def __init__(self, name: str, target: int, power: int, pid_p: float, pid_i: float, pid_d: float,
                 trace_side: TraceSide) -> None:
        super(TraceLine_sensor, self).__init__(name)
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
        print(f"brt={g_color_sensor.get_brightness():.1f} target={self.pid.setpoint} turn={turn}")
        right_power = self.power - turn
        left_power = self.power + turn
        g_right_motor.set_power(right_power)
        g_left_motor.set_power(left_power)
        print(f"right_motor power: {right_power}, left_motor power: {left_power}")
        return Status.RUNNING


class TraceLineCam(Behaviour):
    def __init__(self, name: str, power: int, pid_p: float, pid_i: float, pid_d: float,
                 gs_min: int, gs_max: int, trace_side: TraceSide) -> None:
        super(TraceLineCam, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        self.gs_min = gs_min
        self.gs_max = gs_max
        self.trace_side = trace_side
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            g_video.set_thresholds(self.gs_min, self.gs_max)
            if self.trace_side == TraceSide.NORMAL:
                if g_course == -1: # right course
                    g_video.set_trace_side(TraceSide.RIGHT)
                else:
                    g_video.set_trace_side(TraceSide.LEFT)
            elif self.trace_side == TraceSide.OPPOSITE: 
                if g_course == -1: # right course
                    g_video.set_trace_side(TraceSide.LEFT)
                else:
                    g_video.set_trace_side(TraceSide.RIGHT)
            else: # TraceSide.CENTER
                g_video.set_trace_side(TraceSide.CENTER)
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))
        turn = (-1) * int(self.pid(g_video.get_theta()))
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
    ) -> None:
        global g_hub, g_arm_motor, g_right_motor, g_left_motor, g_touch_sensor, g_color_sensor, g_sonar_sensor
        g_hub = hub
        g_arm_motor = arm_motor
        g_right_motor = right_motor
        g_left_motor = left_motor
        g_touch_sensor = touch_sensor
        g_color_sensor = color_sensor
        g_sonar_sensor = sonar_sensor


class VideoThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            g_video.process(g_plotter, g_hub, g_arm_motor, g_right_motor, g_left_motor, g_color_sensor, g_sonar_sensor)
            time.sleep(VIDEO_INTERVAL)

class IsObstacleNear(Behaviour):# 20250625_add_kubota_ソナーで障害物検知するクラス追加
    def __init__(self, name: str, threshold: int = 250):
        super().__init__(name)
        self.threshold = threshold

    def update(self) -> Status:
        dist = g_sonar_sensor.get_distance()
        print("dist = ",dist)
        if 0 < dist < self.threshold:
            return Status.SUCCESS
            print("IsObstacleNear_start")
        return Status.FAILURE

class AvoidObstacleArcFull(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.done = False
        self.dist = None

    def update(self) -> Status:
        print("AvoidObstacleArcFull_start")
        if self.done:
            return Status.SUCCESS

        # 初回のみ距離取得
        # if self.dist is None:
        #     dist = g_sonar_sensor.get_distance()
        #     print("dist = ",dist)
        #     if dist <= 0:
        #         print("fallback")
        #         dist = 100
        #     self.dist = dist
        # else:
        #     dist = self.dist

        # --- 以下、単純な回避動作 ---
        # 右カーブ
        g_left_motor.set_power(52)
        g_right_motor.set_power(10)
        time.sleep(2)  # 必要に応じて調整
        # 止める
        g_left_motor.set_power(0)
        g_right_motor.set_power(0)

        # 左に戻す
        g_left_motor.set_power(20)
        g_right_motor.set_power(60)
        time.sleep(2.5)  # 必要に応じて調整
        g_left_motor.set_power(0)
        g_right_motor.set_power(0)

        # ライン復帰
        g_left_motor.set_power(30)
        g_right_motor.set_power(30)
        time.sleep(0.4)
        g_left_motor.set_power(0)
        g_right_motor.set_power(0)

        # ライン復帰
        g_left_motor.set_power(50)
        g_right_motor.set_power(10)
        time.sleep(1.25)
        g_left_motor.set_power(0)
        g_right_motor.set_power(0)

        # フラグを立てて終了
        self.done = True
        print("AvoidObstacleArcFull complete!")
        return Status.SUCCESS

class ArcTurn(Behaviour):#20250627_add_kubota_ダブルループ用カーブクラスの追加
    def __init__(self, name, direction, degree=45, power=30, radius=200):
        super().__init__(name)
        self.direction = direction  # "left" or "right"
        self.degree = degree
        self.power = power
        self.radius = radius
        self.running = False

    def update(self) -> Status:
        print("Arcturn_start")
        if not self.running:
            self.running = True
            # degree→タイヤ回転数変換は省略例
            base_angle = self.degree
            if self.direction == "right":
                left_curve_power = self.power
                right_curve_power = int(self.power * 0.5)
                g_left_motor.set_power(left_curve_power)
                g_right_motor.set_power(right_curve_power)
                print(f"right_motor power: {right_curve_power}, left_motor power: {left_curve_power}")
            else:
                left_curve_power = int(self.power * 0.5)
                right_curve_power = self.power
                g_left_motor.set_power(left_curve_power)
                g_right_motor.set_power(right_curve_power)
                print(f"right_motor power: {right_curve_power}, left_motor power: {left_curve_power}")
            # time.sleepで簡易的にカーブの長さを調整する例
            time.sleep(base_angle / 90 * 0.7)  # 調整要
            g_left_motor.set_power(0)
            g_right_motor.set_power(0)
            return Status.SUCCESS
        return Status.RUNNING

class IsDistancePassed(Behaviour):
    def __init__(self, name: str, target_distance: int):
        super().__init__(name)
        self.target_distance = target_distance
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.start_distance = g_plotter.get_distance()
            print(f"[IsDistancePassed] Start: {self.start_distance}, Target: {self.target_distance}")
        now_distance = g_plotter.get_distance()
        if now_distance - self.start_distance >= self.target_distance:
            print(f"[IsDistancePassed] Passed: {now_distance - self.start_distance}")
            return Status.SUCCESS
        return Status.FAILURE

def build_behaviour_tree() -> BehaviourTree:
    # 各ノードを定義
    # オブジェクトを回避するためのノード
    avoid_seq = Sequence(name="avoid_seq", memory=True)
    avoid_seq.add_children([
        IsDistancePassed(name="distance_passed", target_distance=2700),
        AvoidObstacleArcFull(name="arc_avoid")
    ])
    # オブジェクト回避のライントレース
    traceline_cam_for_obstacle = TraceLineCam(
        name="camera_trace_for_obstacle",
        power=55, pid_p=1.0, pid_i=0, pid_d=0.1,
        gs_min=0, gs_max=40,
        trace_side=TraceSide.NORMAL
    )
    # ダブルループのライントレース
    traceline_cam_for_loop = TraceLineCam(
        name="camera_trace_for_loop",
        power=50, pid_p=2.0, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,
        trace_side=TraceSide.NORMAL
    )
    # オブジェクト回避とライントレース
    obstacle_selector = Selector(name="obstacle_or_trace", memory=False)
    obstacle_selector.add_children([
        avoid_seq, 
        traceline_cam_for_obstacle
    ])
    # ダブルループ侵入
    enter_circle = Sequence(name="enter_circle", memory=True)
    enter_circle.add_children([
        IsJunction(name="enter_junction", target_state=JState.FORKING),
        ArcTurn(name="arc_into_circle", direction="right", degree=90, power=30, radius=200)
    ])
    # ダブルループ処理
    double_loop = Sequence(name="eight_loop", memory=True)
    double_loop.add_children([
        TraceLineCam(name="trace_outer",power=60, pid_p=2.0, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.NORMAL),
        IsJunction(name="cross_junction1", target_state=JState.JOINING),
        ArcTurn(name="arc_to_small", direction="left", degree=45, power=30, radius=80),
        TraceLineCam(name="trace_outer",power=60, pid_p=2.0, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.NORMAL),
        IsJunction(name="cross_junction2", target_state=JState.FORKING),
        ArcTurn(name="arc_to_big", direction="right", degree=45, power=30, radius=200)
    ])
    # ダブルループ侵入とダブルループ処理とライントレース
    mid_selector = Selector(name="mid_selector", memory=False)
    mid_selector.add_children([
        Sequence(name="loop_seq", memory=True, children=[
        enter_circle,
        double_loop,
        ]),
        traceline_cam_for_loop
    ])
    loop_01 = Sequence(name="loop_01_with_obstacle", memory=True)
    loop_01.add_children([
        obstacle_selector,
        # mid_selector,
        TraceLineCam(name="trace_clear_obstacle",power=40, pid_p=2.4, pid_i=0.002, pid_d=0.25,
        gs_min=0, gs_max=40,trace_side=TraceSide.NORMAL),
        # IsDistanceEarned(name="check distance", delta_dist=40000)
    ])
    calibration = Sequence(name="calibration", memory=True)
    calibration.add_children([
        ArmUpDownFull(name="arm up", direction=ArmDirection.UP),
        ArmUpDownFull(name="arm down", direction=ArmDirection.DOWN),
        ResetDevice(name="device reset")
    ])
    start = Sequence(name="start", memory=True)
    start.add_children([
        IsTouchOn(name="touch start"),
    ])
    # 各ノードの定義終了
    root = Sequence(name="loop by cam", memory=True)
    root.add_children([
        calibration,
        start,
        loop_01,
        StopNow(name="stop"),
        TheEnd(name="end"),
    ])
    return root

def initialize_etrobo(backend: str) -> ETRobo:
    return (ETRobo(backend=backend)
            .add_hub('hub')
            .add_device('arm_motor', device_type=Motor, port='C')
            .add_device('right_motor', device_type=Motor, port='A')
            .add_device('left_motor', device_type=Motor, port='B')
            .add_device('touch_sensor', device_type=TouchSensor, port='D')
            .add_device('color_sensor', device_type=ColorSensor, port='E')
            .add_device('sonar_sensor', device_type=SonarSensor, port='F'))

def setup_thread():
    global g_video, g_video_thread
    g_video = Video()

    print(" -- starting VideoThread...")
    g_video_thread = VideoThread()
    g_video_thread.start()

def cleanup_thread():
    global g_video, g_video_thread
    print(" -- stopping VideoThread...")
    g_video_thread.stop()
    g_video_thread.join()

    del g_video

def sig_handler(signum, frame) -> None:
    sys.exit(1)
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('course', choices=['right', 'left'], help='Course to run')
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args()

    if args.course == 'right':
        g_course = -1
    else:
        g_course = 1

    setup_thread()

    #py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = build_behaviour_tree()
    display_tree.render_dot_tree(tree)

    signal.signal(signal.SIGTERM, sig_handler)

    try:
        etrobo = initialize_etrobo(backend='raspike_art')
        etrobo.add_handler(ExposeDevices())
        etrobo.add_handler(TraverseBehaviourTree(tree))
        etrobo.dispatch(interval=EXEC_INTERVAL, logfile=args.logfile)
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        cleanup_thread()
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        print(" -- exiting...")
