import argparse
import time
import math
import threading
import signal
from enum import Enum, IntEnum, auto
from etrobo_python import ETRobo, Hub, Motor, TouchSensor, ColorSensor, SonarSensor, GyroSensor
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
from py_etrobo_util import Video, TraceSide, Plotter, SymmetricClamper
from py_etrobo_util.plotter import TIRE_DIAMETER
import colorsys#GRBをHSVに変える標準ライブラリ

EXEC_INTERVAL: float = 0.02
VIDEO_INTERVAL: float = 0.02
ARM_SHIFT_PWM = 30
JUNCT_UPPER_THRESH = 50
JUNCT_LOWER_THRESH = 30
MAX_POWER = 100
MIN_POWER = 60

class ArmDirection(IntEnum):
    UP = -1
    DOWN = 1

class HeadingType(Enum):
    ABSOLUTE = "absolute"
    RELATIVE = "relative"

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
g_gyro_sensor: GyroSensor = None
g_video: Video = None
g_video_thread: threading.Thread = None
g_course: int = 0
g_gate: int = 0
g_is_distance_to_gate = None


class TheEnd(Behaviour):# ctl+cで処理を終了させるようにしている
    def __init__(self, name: str):
        super(TheEnd, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.behavior tree exhausted. ctrl+C shall terminate the program" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.RUNNING


class ResetDevice(Behaviour):# ロボットのモーターの回転数をリセットする専用の「初期化ビヘイビア」
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


class ArmUpDownFull(Behaviour):# アームを上げ下げして初期化するビヘイビア
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


class IsDistanceEarned(Behaviour):# ロボットがある距離だけ進んだかどうかをチェックするビヘイビア
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


class IsSonarOn(Behaviour):# 障害物が近くにある場合に次の行動を制御できる
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


class IsJunction(Behaviour):# 分岐チェックを知らせるだけのクラス
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


class RunAsInstructed(Behaviour):# ロボットの左右のモーターに固定のPWM（出力）を与えて動かす「行動ノード」
    def __init__(self, name: str, pwm_l: int, pwm_r: int) -> None:
        super(RunAsInstructed, self).__init__(name)
        self.pwm_l = pwm_l
        self.pwm_r = pwm_r
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.started with pwm=(%s, %s)" % (g_plotter.get_distance(), self.__class__.__name__, self.pwm_l, self.pwm_r))
        g_right_motor.set_power(g_course * self.pwm_r)
        g_left_motor.set_power(g_course * self.pwm_l)
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
                pid_p: float,
                pid_i: float,
                pid_d: float,
                target_type: HeadingType) -> None:
        super(RunByGyro, self).__init__(name)
        self.target = target
        self.target_type = target_type
        self.power = power
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.running = False
        self.target_heading = 0.0

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                desired_heading = current_heading + self.target
            else:
                desired_heading = self.target
            k = round((current_heading - desired_heading) / 360.0)
            self.target_heading = desired_heading + 360.0 * k    
            self.pid = PID( self.pid_p, 
                            self.pid_i, 
                            self.pid_d, 
                            setpoint=self.target_heading,
                            sample_time=EXEC_INTERVAL, 
                            output_limits=(-self.power, self.power))
            self.running = True
            self.logger.info("%+06d %s.gyro run started toward heading=%.1f" % (g_plotter.get_distance(),self.__class__.__name__, self.target_heading))
        steer = round(self.pid(current_heading))
        right = max(-100, min(100, self.power - steer))
        left  = max(-100, min(100, self.power + steer))
        g_right_motor.set_power(right)
        g_left_motor.set_power(left)
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
                 gs_min: int, gs_max: int, trace_side: TraceSide,
                 dynamic_pid_by_distance: list = None # ← 本橋追加
                 ) -> None: 
        super(TraceLineCam, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=0, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        self.gs_min = gs_min
        self.gs_max = gs_max
        self.trace_side = trace_side
        self.dynamic_pid_by_distance = dynamic_pid_by_distance if dynamic_pid_by_distance else [] # ← 本橋追加
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
            elif self.trace_side == TraceSide.RIGHT: 
                g_video.set_trace_side(TraceSide.RIGHT)
            elif self.trace_side == TraceSide.LEFT: 
                g_video.set_trace_side(TraceSide.LEFT)
            else: # TraceSide.CENTER
                g_video.set_trace_side(TraceSide.CENTER)
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))
        
        #距離に応じたPIDの動的切り替え （本橋追記）
        if self.dynamic_pid_by_distance:
            current_distance = g_plotter.get_distance() - 2500
            for entry in self.dynamic_pid_by_distance:
                if entry["start"] <= current_distance < entry["end"]:
                    self.power = entry["power"]
                    self.pid.p = entry["p"]
                    self.pid.i = entry["i"]
                    self.pid.d = entry["d"]
                    # print(f"[TraceLineCam] Distance={current_distance}, Power={self.power}, PID={self.pid.p}, {self.pid.i}, {self.pid.d}")
                    break
        
        turn = (-1) * int(self.pid(g_video.get_theta()))
        g_right_motor.set_power(self.power - turn - 1)
        g_left_motor.set_power(self.power + turn)
        return Status.RUNNING

class TraceLineSensor(Behaviour):# カラーセンサー用クラス
    def __init__(self, name: str, target: int, power: int, pid_p: float, pid_i: float, pid_d: float,
                 trace_side: TraceSide) -> None:
        super(TraceLineSensor, self).__init__(name)
        self.power = power
        self.pid = PID(pid_p, pid_i, pid_d, setpoint=target, sample_time=EXEC_INTERVAL, output_limits=(-power, power))
        self.trace_side = trace_side
        self.running = False

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.trace started with TS=%s" % (g_plotter.get_distance(), self.__class__.__name__, self.trace_side.name))

        brightness = g_color_sensor.get_brightness()
        if self.trace_side == TraceSide.NORMAL:
            turn = (-1) * g_course * int(self.pid(brightness))
        else:  # TraceSide.OPPOSITE
            turn = g_course * int(self.pid(brightness))

        g_right_motor.set_power(self.power - turn)
        g_left_motor.set_power(self.power + turn)
        return Status.RUNNING

class DetectRed(Behaviour):# 赤色検知用クラス
    def __init__(self, name: str):
        super().__init__(name)
        self.count = 0
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        r, g, b = g_color_sensor.get_raw_color()
        # 正規化：最大値で割る（例：センサの上限値が1023なら/1023.0、255なら/255.0）
        max_rgb = max(r, g, b, 1)  # 1で割りゼロ防止
        r_norm = r / max_rgb
        g_norm = g / max_rgb
        b_norm = b / max_rgb
        # colorsysで変換（返り値: h,s,vは0.0〜1.0）
        h, s, v = colorsys.rgb_to_hsv(r_norm, g_norm, b_norm)
        # 色相Hだけ0〜360度に直す
        h_deg = int(h * 360)
        s_per = int(s * 100)
        v_per = int(v * 100)
        # print(f"RGB: {r}, {g}, {b} → HSV: {h_deg}°, {s_per}%, {v_per}%")
        # 青色のHSV範囲例 (h: 200〜260くらい、s: 高め、v: 中～高)
        if ((0 <= h_deg <= 20) or (340 <= h_deg <= 360)) and s_per > 50 and v_per > 30:
            self.logger.info("%+06d %s.DetectRed!" % (g_plotter.get_distance(), self.__class__.__name__))
            print(f"DetectRed: RED! h={h_deg} s={s_per} v={v_per}")
            return Status.SUCCESS
        else:
            # print(f"DetectRed: Not RED h={h_deg} s={s_per} v={v_per}")
            return Status.RUNNING

class DetectBlue(Behaviour):# 青色検知用クラス
    def __init__(self, name: str):
        super().__init__(name)
        self.count = 0
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False
        self._did_reset = False

    def update(self) -> Status:
        r, g, b = g_color_sensor.get_raw_color()
        # 正規化：最大値で割る（例：センサの上限値が1023なら/1023.0、255なら/255.0）
        max_rgb = max(r, g, b, 1)  # 1で割りゼロ防止
        r_norm = r / max_rgb
        g_norm = g / max_rgb
        b_norm = b / max_rgb
        # colorsysで変換（返り値: h,s,vは0.0〜1.0）
        h, s, v = colorsys.rgb_to_hsv(r_norm, g_norm, b_norm)
        # 色相Hだけ0〜360度に直す
        h_deg = int(h * 360)
        s_per = int(s * 100)
        v_per = int(v * 100)
        # print(f"RGB: {r}, {g}, {b} → HSV: {h_deg}°, {s_per}%, {v_per}%")
        # 青色のHSV範囲例 (h: 200〜260くらい、s: 高め、v: 中～高)
        if 200 <= h_deg <= 260 and s_per > 40 and v_per > 30:
            self.logger.info("%+06d %s.DetectBlue Once!" % (g_plotter.get_distance(), self.__class__.__name__))
            print(f"DetectBlue: BLUE! h={h_deg} s={s_per} v={v_per}")
            return Status.SUCCESS
        else:
            # print(f"DetectBlue: Not Blue h={h_deg} s={s_per} v={v_per}")
            return Status.RUNNING

class DetectBlue_failure(Behaviour):# 青色検知用クラス
    def __init__(self, name: str):
        super().__init__(name)
        self.count = 0
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.running = False

    def update(self) -> Status:
        r, g, b = g_color_sensor.get_raw_color()
        # 正規化：最大値で割る（例：センサの上限値が1023なら/1023.0、255なら/255.0）
        max_rgb = max(r, g, b, 1)  # 1で割りゼロ防止
        r_norm = r / max_rgb
        g_norm = g / max_rgb
        b_norm = b / max_rgb
        # colorsysで変換（返り値: h,s,vは0.0〜1.0）
        h, s, v = colorsys.rgb_to_hsv(r_norm, g_norm, b_norm)
        # 色相Hだけ0〜360度に直す
        h_deg = int(h * 360)
        s_per = int(s * 100)
        v_per = int(v * 100)
        # print(f"RGB: {r}, {g}, {b} → HSV: {h_deg}°, {s_per}%, {v_per}%")
        # 青色のHSV範囲例 (h: 200〜260くらい、s: 高め、v: 中～高)
        if 200 <= h_deg <= 260 and s_per > 40 and v_per > 30:
            self.logger.info("%+06d %s.DetectBlue Once!" % (g_plotter.get_distance(), self.__class__.__name__))
            print(f"DetectBlue: BLUE! h={h_deg} s={s_per} v={v_per}")
            return Status.SUCCESS
        else:
            # print(f"DetectBlue: Not Blue h={h_deg} s={s_per} v={v_per}")
            return Status.FAILURE

class Detectcolor(Behaviour):# 色や明るさを取得する
    def __init__(self, name: str):
        super().__init__(name)

    def update(self) -> Status:
        r, g, b = g_color_sensor.get_raw_color()
        # 正規化：最大値で割る（例：センサの上限値が1023なら/1023.0、255なら/255.0）
        max_rgb = max(r, g, b, 1)  # 1で割りゼロ防止
        r_norm = r / max_rgb
        g_norm = g / max_rgb
        b_norm = b / max_rgb
        # colorsysで変換（返り値: h,s,vは0.0〜1.0）
        h, s, v = colorsys.rgb_to_hsv(r_norm, g_norm, b_norm)
        # 色相Hだけ0〜360度に直す
        h_deg = int(h * 360)
        s_per = int(s * 100)
        v_per = int(v * 100)
        print(f"RGB: {r}, {g}, {b} → HSV: {h_deg}°, {s_per}%, {v_per}%")
        brightness = g_color_sensor.get_brightness()
        print(f"brightness={brightness}")
        # 青色のHSV範囲例 (h: 200〜260くらい、s: 高め、v: 中～高)
        if 200 <= h_deg <= 260 and s_per > 40 and v_per > 30:
            # print(f"DetectBlue: BLUE! h={h_deg} s={s_per} v={v_per}")
            return Status.RUNNING
        # print(f"DetectBlue: Not Blue h={h_deg} s={s_per} v={v_per}")
        return Status.RUNNING

class IsOnBlackLine(Behaviour):#黒色を明るさで検知
    def __init__(self, name: str, threshold: int = 40):
        super().__init__(name)
        self.threshold = threshold
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        brightness = g_color_sensor.get_brightness()
        if brightness < self.threshold:  # 明るさがthreshold未満=黒い
            self.logger.info("%+06d %s.DetectBlack!" % (g_plotter.get_distance(), self.__class__.__name__))
            print(f"[IsOnBlackLine] Detected! brightness={brightness}")
            return Status.SUCCESS
        else:
            # self.logger.info("%+06d %s.NotDetected..." % (g_plotter.get_distance(), self.__class__.__name__))
            # print(f"[IsOnBlackLine] NotDetected... brightness={brightness}")
            return Status.FAILURE

class IsOnBlackLine_running(Behaviour):#黒色を明るさで検知
    def __init__(self, name: str, threshold: int = 40):
        super().__init__(name)
        self.threshold = threshold
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        brightness = g_color_sensor.get_brightness()
        if brightness < self.threshold:  # 明るさがthreshold未満=黒い
            self.logger.info("%+06d %s.DetectBlack!" % (g_plotter.get_distance(), self.__class__.__name__))
            print(f"[IsOnBlackLine] Detected! brightness={brightness}")
            return Status.SUCCESS
        else:
            # self.logger.info("%+06d %s.NotDetected..." % (g_plotter.get_distance(), self.__class__.__name__))
            # print(f"[IsOnBlackLine] NotDetected... brightness={brightness}")
            return Status.RUNNING

class DetectBlackCount(Behaviour):
    def __init__(self, name: str, black_thresh: int = 5, gray_thresh: int = 30, target_count: int = 3):
        super().__init__(name)
        self.black_thresh = black_thresh
        self.gray_thresh = gray_thresh
        self.target_count = target_count
        self.count = 0

    def update(self) -> Status:
        brightness = g_color_sensor.get_brightness()
        if brightness < self.black_thresh:
            self.count += 1
            print(f"黒検知回数: {self.count}")
            if self.count >= self.target_count:
                return Status.SUCCESS
            else:
                return Status.RUNNING
        if brightness < self.gray_thresh:
            self.count += 1
            print(f"黒/グレー検知回数: {self.count} (brightness={brightness})")
            if self.count >= self.target_count:
                return Status.SUCCESS
            else:
                return Status.RUNNING
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
            # voltage = g_hub.get_battery_voltage()
            # current = g_hub.get_battery_current()
            # print(f"[Battery] Voltage={voltage} mV, Current={current} mA")

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

class TurnToObject(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.done = False
        self.dist = None
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        if self.done:
            return Status.SUCCESS
        self.logger.info("%+06d %s.AvoidObstacleArcFull_start!" % (g_plotter.get_distance(), self.__class__.__name__))
        if g_course == 1: #LEFTコースの場合
            # バック
            g_left_motor.set_power(-50)
            g_right_motor.set_power(-50)
            time.sleep(0.6)  # 必要に応じて調整
            g_left_motor.set_power(0)
            g_right_motor.set_power(0)

            # 右周りに180°回転
            g_left_motor.set_power(60)
            g_right_motor.set_power(100)
            time.sleep(0.85)  # 必要に応じて調整 
            g_left_motor.set_power(0)
            g_right_motor.set_power(0)
        else:           #RIGHTコースの場合
            # バック
            g_left_motor.set_power(-50)
            g_right_motor.set_power(-50)
            time.sleep(0.6)  # 必要に応じて調整
            g_left_motor.set_power(0)
            g_right_motor.set_power(0)

            # 左周りに180°回転
            g_left_motor.set_power(100)
            g_right_motor.set_power(60)
            time.sleep(0.85)  # 必要に応じて調整 
            g_left_motor.set_power(0)
            g_right_motor.set_power(0)

        # フラグを立てて終了
        self.done = True
        self.logger.info("%+06d %s.TurnToObject_complete!" % (g_plotter.get_distance(), self.__class__.__name__))
        return Status.SUCCESS

class AvoidObstacleArcFull(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.done = False
        self.dist = None
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        if self.done:
            return Status.SUCCESS
        self.logger.info("%+06d %s.AvoidObstacleArcFull_start!" % (g_plotter.get_distance(), self.__class__.__name__))
        # --- 以下、単純な回避動作 ---
        # 止める
        g_left_motor.set_power(0)
        g_right_motor.set_power(0)
        # 右カーブ
        g_left_motor.set_power(100)
        g_right_motor.set_power(60)
        time.sleep(0.6)  # 必要に応じて調整
        # 止める
        g_left_motor.set_power(0)
        g_right_motor.set_power(0)

        # 左に戻す
        g_left_motor.set_power(60)
        g_right_motor.set_power(100)
        time.sleep(1.1)  # 必要に応じて調整 
        # g_left_motor.set_power(0)
        # g_right_motor.set_power(0)

        # ライン復帰
        g_left_motor.set_power(100)
        g_right_motor.set_power(100)
        time.sleep(0.5)
        # 止める
        g_left_motor.set_power(0)
        g_right_motor.set_power(0)

        # ライン復帰
        #g_left_motor.set_power(50)
        #g_right_motor.set_power(10)
        #time.sleep(1.17)
        #g_left_motor.set_power(0)
        #g_right_motor.set_power(0)

        # フラグを立てて終了
        self.done = True
        self.logger.info("%+06d %s.AvoidObstacleArcFull_complete!" % (g_plotter.get_distance(), self.__class__.__name__))
        
        return Status.SUCCESS

class ArcTurn(Behaviour):#20250627_add_kubota_ダブルループ用カーブクラスの追加
    def __init__(self, name, direction, degree=45, power=30, radius=200):
        super().__init__(name)
        self.direction = direction  # "left" or "right"
        self.degree = degree
        self.power = power
        self.radius = radius
        self.running = False
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        if not self.running:
            self.logger.info("%+06d %s.Arcturn_start!" % (g_plotter.get_distance(), self.__class__.__name__))
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
            self.logger.info("%+06d %s.Arcturn_complete!" % (g_plotter.get_distance(), self.__class__.__name__))
            return Status.SUCCESS
        return Status.SUCCESS

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
            print("======================== end ========================")
            return Status.SUCCESS
        return Status.RUNNING

def gate_value(front_val: int, back_val: int) -> int:
    """
    --gate の指定に応じて値を切り替えるユーティリティ。
    front/back 以外は来ない想定だが、万が一に備えて front をデフォルト。
    """
    try:
        return front_val if g_gate == 'front' else back_val
    except NameError:
        # 念のため g_gate 未設定でも落ちないように
        return front_val

def build_behaviour_tree() -> BehaviourTree:
    # 各ノードを定義

    # ============= オブジェクト回避 =============

    # オブジェクトを回避するためのノード
    avoid_seq = Sequence(name="avoid_seq", memory=True)
    avoid_seq.add_children([
        IsDistancePassed(name="distance_passed", target_distance=2400),
        SpinAround(name="spin by 90 degrees_After_puton_back_first", target=0, max_power=60, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        AvoidObstacleArcFull(name="arc_avoid")
    ])
    # ============= ライントレース =============

    # オブジェクト回避前のライントレース
    traceline_cam_for_obstacle = TraceLineCam(
        name="camera_trace_for_obstacle",
        power=60, pid_p=1.8, pid_i=0.0015, pid_d=0.23,
        #power=60, pid_p=1.4, pid_i=0.0015, pid_d=0.3,
        gs_min=0, gs_max=40,
        trace_side=TraceSide.RIGHT
    )
    # オブジェクト回避とライントレース
    obstacle_Parallel = Parallel(name="obstacle_or_trace", policy=ParallelPolicy.SuccessOnOne())
    obstacle_Parallel.add_children([
        avoid_seq, 
        # traceline_cam_for_obstacle
        RunByGyro(name="run_back_GoBlackLine", target=0, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    # オブジェクトを無視してジャイロで真っ直ぐ
    gyro_obstacle_ignore_Parallel = Parallel(name="gyro_obstacle_ignore", policy=ParallelPolicy.SuccessOnOne())
    gyro_obstacle_ignore_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=4675),#カーブまで
        RunByGyro(name="run_back_GoBlackLine", target=0, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    gyro_obstacle_avoid_Parallel = Parallel(name="gyro_obstacle_avoid", policy=ParallelPolicy.SuccessOnOne())
    gyro_obstacle_avoid_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=850),#カーブまで
        RunByGyro(name="run_back_GoBlackLine", target=0, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    #向正面をジャイロで真っ直ぐ
    gyro_mukoujoumen_Parallel = Parallel(name="gyro_mukoujoumen", policy=ParallelPolicy.SuccessOnOne())
    gyro_mukoujoumen_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=3100),#カーブまで
        RunByGyro(name="run_back_GoBlackLine", target=-90, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    gyro_gotolap_Parallel = Parallel(name="gyro_gotolap", policy=ParallelPolicy.SuccessOnOne())
    gyro_gotolap_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=500),#カーブまで
        RunByGyro(name="run_back_GoBlackLine", target=-180, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    # # オブジェクト回避後からLAP完了まで（LAP完了は青色検知）
    # traceline_cam_lapfinish_Parallel = Parallel(name="detectblue_or_trace", policy=ParallelPolicy.SuccessOnOne())
    # traceline_cam_lapfinish_Parallel.add_children([
    #     DetectBlue(name="detect_blue"),
    #     TraceLineCam(name="traceline_cam_lapfinish",power=48, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
    #     gs_min=0, gs_max=80,trace_side=TraceSide.NORMAL,
    #     # 距離ごとのPOWERとPID設定（本橋修正）
    #     dynamic_pid_by_distance=[
    #         {"start": 0, "end": 2650, "power": 45, "p": 2.2, "i": 0.0012, "d": 0.18},
    #         # {"start": 2550, "end": 4800, "power": 80, "p": 0.4,  "i": 0.0035,  "d": 0.3},
    #         {"start": 2650, "end": 4700, "power": 70, "p": 1.2, "i": 0.0015, "d": 0.25},
    #         {"start": 4700, "end": 5700, "power": 45, "p": 2.2, "i": 0.0012, "d": 0.18},
    #         {"start": 5700, "end": 9999, "power": 40, "p": 1.75, "i": 0.0012, "d": 0.18}
    #     ]
    #     ),
    # ])
    # オブジェクト回避後からLAP完了まで（LAP完了は青色検知）※ジャイロ用
    traceline_cam_start_doubleloop_Parallel = Parallel(name="detectblue_or_trace", policy=ParallelPolicy.SuccessOnOne())
    traceline_cam_start_doubleloop_Parallel.add_children([
        DetectBlue(name="detect_blue"),
        IsDistancePassed(name="distance_passed", target_distance=500),      #青検知しなかったとき用
        TraceLineCam(name="traceline_cam_lapfinish",power=40, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.CENTER)
    ])

    # ================ ダブルループ処理 ================

    # ================ 黒線検知でライントレース ================
    #※RunAsInstructedの曲がり具合は要調整

    # 一定距離右周りに弧を描くように走る
    distance_loop_Parallel = Parallel(name="distance_loop_Parallel", policy=ParallelPolicy.SuccessOnOne())
    distance_loop_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=750),
        #RunAsInstructed(name="go_straight", pwm_l=58, pwm_r=50),      #LEFT用
        RunAsInstructed(name="go_straight", pwm_l=-50, pwm_r=-58),  #RIGHT用
    ])
    # part1_黒線を検知した場合ライントレース
    double_loop_black_selector_1 = Selector(name="double_loop_black_selector1",memory=False)
    double_loop_black_selector_1.add_children([
        IsOnBlackLine(name="detect_blackline_1", threshold=5),
        #RunAsInstructed(name="go_straight_1", pwm_l=58, pwm_r=50),      #LEFT用
        RunAsInstructed(name="go_straight_1", pwm_l=-40, pwm_r=-48),  #RIGHT用
    ])
    # part2_黒線を検知した場合ライントレース
    double_loop_black_selector_2 = Selector(name="double_loop_black_selector2",memory=False)
    double_loop_black_selector_2.add_children([
        IsOnBlackLine(name="detect_blackline_2", threshold=5),
        #RunAsInstructed(name="go_straight_2", pwm_l=45, pwm_r=48),      #LEFT用
        RunAsInstructed(name="go_straight_2", pwm_l=-42, pwm_r=-45),  #RIGHT用
    ])
    # part3_黒線を検知した場合ライントレース
    double_loop_black_selector_3 = Selector(name="double_loop_black_selector3",memory=False)
    double_loop_black_selector_3.add_children([
        IsOnBlackLine(name="detect_blackline_3", threshold=5),
        #RunAsInstructed(name="go_straight_3", pwm_l=40, pwm_r=47),      #LEFT用
        RunAsInstructed(name="go_straight_3", pwm_l=-50, pwm_r=-60),  #RIGHT用
    ])
    # part4_黒線を検知した場合ライントレース
    double_loop_black_selector_4 = Selector(name="double_loop_black_selector4",memory=False)
    double_loop_black_selector_4.add_children([
        IsOnBlackLine(name="detect_blackline_4", threshold=5),
        #RunAsInstructed(name="go_straight_4", pwm_l=50, pwm_r=50),      #LEFT用
        RunAsInstructed(name="go_straight_4", pwm_l=-50, pwm_r=-50),  #RIGHT用
    ])
    # 小円に入るときの調整
    SmallCircleEntryTuning_selector = Parallel(name="SmallCircleEntryTuning_selector", policy=ParallelPolicy.SuccessOnOne())
    SmallCircleEntryTuning_selector.add_children([
        IsDistancePassed(name="distance_passed", target_distance=500),  #200は適当なので要調整
        #RunAsInstructed(name="SmallCircle_Entry", pwm_l=60, pwm_r=50),      #LEFT用
        RunAsInstructed(name="SmallCircle_Entry", pwm_l=-50, pwm_r=-50),  #RIGHT用
    ])
    # 大円に入るときの調整
    BigCircleEntryTuning_selector = Parallel(name="BigCircleEntryTuning_selector", policy=ParallelPolicy.SuccessOnOne())
    BigCircleEntryTuning_selector.add_children([
        IsDistancePassed(name="distance_passed", target_distance=550),  #200は適当なので要調整
        #RunAsInstructed(name="BigCircle_Entry", pwm_l=60, pwm_r=50),      #LEFT用
        RunAsInstructed(name="BigCircle_Entry", pwm_l=-50, pwm_r=-35),  #RIGHT用
    ])

    # ================ 青色検知するまでライントレース ================
    
    # part1_青色検知するまでライントレース
    double_loop_blue_parallel_1 = Parallel(name="double_loop_blue_parallel_1",policy=ParallelPolicy.SuccessOnOne())
    double_loop_blue_parallel_1.add_children([
        DetectBlue(name="detect_blue"),
        IsDistancePassed(name="distance_passed", target_distance=2025),      #青検知しなかったとき用
        TraceLineCam(name="traceline_cam_lapfinish",power=40, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.OPPOSITE),
    ])
    # part2_青色検知するまでライントレース
    double_loop_blue_parallel_2 = Parallel(name="double_loop_blue_parallel_2",policy=ParallelPolicy.SuccessOnOne())
    double_loop_blue_parallel_2.add_children([
        DetectBlue(name="detect_blue"),
        IsDistancePassed(name="distance_passed", target_distance=2200),      #青検知しなかったとき用
        TraceLineCam(name="Tracelinecam_DetectBlue_2",power=40, pid_p=2.0, pid_i=0.0012, pid_d=0.1,
        gs_min=0, gs_max=50,trace_side=TraceSide.NORMAL),#小円は右のエッジをトレースしたいから"OPPOSITE"
    ])
    # part3_青色検知するまでライントレース
    double_loop_blue_selector_3 = Selector(name="double_loop_blue_selector_3",memory=False)
    double_loop_blue_selector_3.add_children([
        DetectBlue_failure(name="detect_blue"),
        TraceLineCam(name="Tracelinecam_DetectBlue_3",power=40, pid_p=2.0, pid_i=0.0012, pid_d=0.1,
        gs_min=0, gs_max=40,trace_side=TraceSide.CENTER),
    ])

    escape_double_loop_Parallel = Parallel(name="detectblue_or_trace", policy=ParallelPolicy.SuccessOnOne())
    escape_double_loop_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=800),
        TraceLineCam(name="detectblue_or_trace",power=48, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.NORMAL),
    ])

    # ================ スマートキャリー用のノード ================
    # --------ダブルループ抜けてからオブジェクト下の青検知まで
    traceline_cam_smacary_Parallel = Parallel(name="detectblue_or_trace", policy=ParallelPolicy.SuccessOnOne())
    traceline_cam_smacary_Parallel.add_children([
        DetectBlue(name="detect_blue"),
        TraceLineCam(name="detectblue_or_trace",power=40, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.CENTER),
    ])
    # --------ゲートの位置までまっすぐ進む（ゲートの位置で進む距離が変わる）
    BringObject_to_Gate_Parallel = Parallel(name="BringObject_to_Gate", policy=ParallelPolicy.SuccessOnOne())
    BringObject_to_Gate_Parallel.add_children([
        # -----ゲートの位置で距離が変わるようになっている⇒gate_value(300=front, 500=back)
        IsDistancePassed(name="distance_passed", target_distance=gate_value(500, 850)),
        RunAsInstructed(name="go_gate", pwm_l=-50, pwm_r=-65),
    ])
    # --------90度回転して、ジャイロでまっすぐ進む（距離で制御。）
    SpinAndRun_Parallel = Parallel(name="SpinAndRun", policy=ParallelPolicy.SuccessOnOne())
    SpinAndRun_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=2400),
        RunByGyro(name="run straight_SpinAndRun", target=93, power=80,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    # --------青色検知からゲート通過までをノード化したもの
    SpinAndRun_Sequence = Sequence(name="SpinAndRun_Sequence", memory=True)
    SpinAndRun_Sequence.add_children([
        BringObject_to_Gate_Parallel,#-----ゲート位置までオブジェクトを運ぶ（ゲート位置によって距離制御あり）
        # SpinAround(name="spin by 90 degrees_SpinAndRun_1", target=90, max_power=70, min_power=MIN_POWER,
        #             pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        SpinAndRun_Parallel,#--------------ゲートを通過する
        SpinAround(name="spin by 45or90 degrees_SpinAndRun_2", target=-360, max_power=60, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    # --------ジャイロでターゲットまでまっすぐ進む（距離制御でオブジェクトを置く）
    smart_carry_puton_first_Parallel = Parallel(name="smart_carry_puton", policy=ParallelPolicy.SuccessOnOne())
    smart_carry_puton_first_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=gate_value(330, 600)),
        RunByGyro(name="run straight_smart_carry_puton", target=0, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    # --------ジャイロでバック（距離で制御）
    After_puton_back_first_Parallel = Parallel(name="After_puton_back", policy=ParallelPolicy.SuccessOnOne())
    After_puton_back_first_Parallel.add_children([
        IsDistancePassed(name="distance_passed_back", target_distance=250),
        RunAsInstructed(name="go_straight_3", pwm_l=60, pwm_r=60),
    ])
    # --------次のオブジェクトの黒線に赤検知するまでライントレース
    detect_red_Parallel = Parallel(name="detect_red_Parallel", policy=ParallelPolicy.SuccessOnOne())
    detect_red_Parallel.add_children([
        DetectRed(name="detect_red"),
        TraceLineCam(name="traceline_to_object",power=48, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.NORMAL),
    ])
    # --------ターゲットまでまっすぐ進む_puton後（距離で制御）
    After_puton_back_second_Parallel = Parallel(name="After_puton_back", policy=ParallelPolicy.SuccessOnOne())
    After_puton_back_second_Parallel.add_children([
        IsDistancePassed(name="distance_passed_back", target_distance=1050),
        RunByGyro(name="run straight_smart_carry_puton", target=-180, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --------ジャイロでターゲットまでまっすぐ進む（距離で制御）
    smart_carry_puton_second_Parallel = Parallel(name="smart_carry_puton", policy=ParallelPolicy.SuccessOnOne())
    smart_carry_puton_second_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=1680),
        RunByGyro(name="run straight_smart_carry_puton_second", target=-90, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    # --------ジャイロでバック（距離で制御）
    After_puton_back_third_Parallel = Parallel(name="After_puton_back", policy=ParallelPolicy.SuccessOnOne())
    After_puton_back_third_Parallel.add_children([
        IsDistancePassed(name="distance_passed_back", target_distance=200),
        RunAsInstructed(name="go_straight_3", pwm_l=60, pwm_r=60),
    ])
    # --------ジャイロで黒線に向かう処理
    GoBlackLine_Parallel = Parallel(name="GoBlackLine", policy=ParallelPolicy.SuccessOnOne())
    GoBlackLine_Parallel.add_children([
        IsDistancePassed(name="distance_passed_GoBlackLine", target_distance=800),
        RunByGyro(name="run_back_GoBlackLine", target=-45, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    # GoBlackLine2_Parallel = Parallel(name="GoBlackLine_2", policy=ParallelPolicy.SuccessOnOne())
    # GoBlackLine2_Parallel.add_children([
    #     IsDistancePassed(name="distance_passed_GoBlackLine_2", target_distance=200),
    #     RunByGyro(name="run_back_GoBlackLine_2", target=-90, power=60,
    #             pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    # ])
    DetectBlackLine_Parallel = Parallel(name="DetectBlackLine", policy=ParallelPolicy.SuccessOnOne())
    DetectBlackLine_Parallel.add_children([
        IsOnBlackLine_running(name="detect_blackline", threshold=5),
        IsDistancePassed(name="distance_passed_GoBlackLine", target_distance=600),
        RunByGyro(name="run_back_DetectBlackLine", target=-90, power=40,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])
    # --------ジャイロで90°回転して青ライン検知するまでライントレース
    traceline_cam_DetectBlue_GOAL_Parallel = Parallel(name="traceline_cam_DetectBlue_GOAL", policy=ParallelPolicy.SuccessOnOne())
    traceline_cam_DetectBlue_GOAL_Parallel.add_children([
        DetectBlue(name="detect_blue"),
        TraceLineCam(name="traceline_cam_DetectBlue_GOAL",power=48, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.NORMAL),
    ])

    loop_01 = Sequence(name="loop_01_with_obstacle_and_doubleloop", memory=True)
    loop_01.add_children([
        # Detectcolor(name="detectcolor"),#       色や明るさを検知できる（ずっとRUNNINGで無限ループ）※次の処理にはいかない仕様
        # --------直線とオブジェクト回避--------
        obstacle_Parallel,#                     直線のライントレースをする。一定距離走ったらオブジェクト回避して抜ける。
        # traceline_cam_lapfinish_Parallel,#      オブジェクト回避後からLAP通過までのライントレース（青いライン検知で抜ける）
        # gyro_obstacle_ignore_Parallel,#           最初の直線（オブジェクト無視）
        SpinAround(name="spin by 90 degrees_After_puton_back_first", target=0, max_power=60, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        gyro_obstacle_avoid_Parallel,
        SpinAround(name="spin by 90 degrees_After_puton_back_first", target=-87, max_power=60, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        gyro_mukoujoumen_Parallel,#               向正面の直線
        SpinAround(name="spin by 90 degrees_After_puton_back_first", target=-177, max_power=60, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        gyro_gotolap_Parallel,#                   LAPまで進む
        traceline_cam_start_doubleloop_Parallel,# LAPからダブルループまでのライントレース（青いライン検知で抜ける）
        # --------ここからダブルループ--------
        # SpinAround(name="spin by 90 degrees_After_puton_back_first", target=-183, max_power=60, min_power=MIN_POWER,
        #             pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        distance_loop_Parallel,#                  ①弧のラインに向かってトレースをするように調整する処理（トレースはしてない）
        # double_loop_black_selector_1,#            ②調整した後、黒いライン検知する処理（いらないかも）
        double_loop_blue_parallel_1,#             ③ライントレースしながら青いラインを探す処理
        # --------ここから下は上手くいかないかも---------
        # --------小円に移るときの処理--------
        SmallCircleEntryTuning_selector,#         ④青いラインを発見後に小円に入るときに左周りの弧を描き、黒線を迎えに行く
        # double_loop_black_selector_2,#            ④黒い線を探しながら弧を描く処理（重なってる黒いラインを無視する処理が必要かも）
        # ArcTurn(name="arc_move1", direction="right", degree=45, power=45, radius=80),
        double_loop_blue_parallel_2,#             ⑤ライントレースしながら青いラインを探す処理
        # --------小円から大円に移るときの処理--------
        # SpinAround(name="spin by 90 degrees_After_puton_back_first", target=300, max_power=60, min_power=MIN_POWER,
        #             pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        BigCircleEntryTuning_selector,#           ⑥青いラインを発見後に大円に入るときに右周りの弧を描き、黒線を迎えに行く
        # double_loop_black_selector_3,#            ⑥黒い線を探しながら弧を描く処理（重なってる黒いラインを無視する処理が必要かも）
        double_loop_blue_selector_3,#             ⑦ライントレースしながら青いラインを探す処理
        escape_double_loop_Parallel,#             ⑧ダブルループを抜ける処理
    ])

    loop_02 = Sequence(name="loop_02_with_smart_carry_twin", memory=True)
    loop_02.add_children([
        # --------オブジェクト下の青検知～ターゲットサークルの黒検知まで
        traceline_cam_smacary_Parallel,
        SpinAndRun_Sequence,
        # --------ターゲットにオブジェクトを置く
        smart_carry_puton_first_Parallel,
        # --------バックして黒線に向かって回転
        After_puton_back_first_Parallel,
        SpinAround(name="spin by 90 degrees_After_puton_back_first", target=-180, max_power=60, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        # --------ライントレースしながらオブジェクト下の赤検知～ターゲットサークルの黒検知まで
        #detect_red_Parallel,#---------------------赤検知
        After_puton_back_second_Parallel,
        SpinAround(name="spin by 90 degrees_detect_red", target=-90, max_power=60, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        # --------ターゲットにオブジェクトを置く
        smart_carry_puton_second_Parallel,#-------オブジェクトを置く
        # --------バックして黒線へ
        After_puton_back_third_Parallel,#--------バック
        SpinAround(name="spin by 90 degrees_After_puton_back_second", target=-45, max_power=80, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        GoBlackLine_Parallel,#--------------------45度回転して一定距離ジャイロで進む
        SpinAround(name="spin by 90 degrees_GoBlackLine_1", target=-90, max_power=80, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        # GoBlackLine2_Parallel,#-------------------45度回転して一定距離ジャイロで進む
        # SpinAround(name="spin by 90 degrees_GoBlackLine_2", target=-45, max_power=60, min_power=MIN_POWER,
        #             pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        # --------黒線検知で90度回転する
        DetectBlackLine_Parallel,#----------------黒線を見つけるまでジャイロで進む
        SpinAround(name="spin by 90 degrees_DetectBlackLine", target=-180, max_power=80, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
        # --------青線検知するまでライントレース
        traceline_cam_DetectBlue_GOAL_Parallel,
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

    root = Sequence(name="loop_by_camera", memory=True)
    root.add_children([
        calibration,
        start,
        loop_01,
        loop_02,
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
            .add_device('sonar_sensor', device_type=SonarSensor, port='F')
            .add_device('gyro_sensor', device_type=GyroSensor, port='',
                        config=[2.0, 2500.0,
                        [-0.239569, -2.50881, 0.6617843], [361.9036, 355.9302, 361.8885],
                        [10089.76, -9720.13, 9931.442, -9704.719, 9522.367, -10210.74]])
            )

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
    parser.add_argument('--gate', choices=['front', 'back'], default='front', help='Gate position to use')
    parser.add_argument('--logfile', type=str, default=None, help='Path to log file')
    args = parser.parse_args()

    if args.course == 'right':
        g_course = -1
    else:
        g_course = 1
    
    g_gate = args.gate

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
