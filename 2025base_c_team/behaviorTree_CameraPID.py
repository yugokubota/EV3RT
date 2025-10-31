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
from py_etrobo_util.video import FRAME_WIDTH, FRAME_HEIGHT

EXEC_INTERVAL: float = 0.02
VIDEO_INTERVAL: float = 0.02
ARM_SHIFT_PWM = 30
JUNCT_UPPER_THRESH = 50
JUNCT_LOWER_THRESH = 30
MAX_POWER = 100
MIN_POWER = 50

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
        self.debug_count = 0

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.logger.info("%+06d %s.started with pwm=(%s, %s)" % (g_plotter.get_distance(), self.__class__.__name__, self.pwm_l, self.pwm_r))
        right_power = g_course * self.pwm_r
        left_power  = g_course * self.pwm_l
        g_right_motor.set_power(right_power)
        g_left_motor.set_power(left_power)
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
        self.max_power = max_power
        self.clamper = SymmetricClamper(min_power, max_power)
        self.running = False

    def update(self) -> Status:
        current_heading = (-1) * g_course * g_gyro_sensor.get_angle()
        if not self.running:
            if self.target_type == HeadingType.RELATIVE:
                self.target_heading = current_heading + self.target
                desired_heading = current_heading + self.target
            else:
                self.target_heading = self.target
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d, setpoint=self.target_heading, sample_time=EXEC_INTERVAL)
            desired_heading = self.target
            # RunByGyro と同じ：「現在角に最も近い等価目標角」へ折り返し
            k = round((current_heading - desired_heading) / 360.0)
            self.target_heading = desired_heading + 360.0 * k
            self.pid = PID(self.pid_p, self.pid_i, self.pid_d,
                        setpoint=self.target_heading,
                        sample_time=EXEC_INTERVAL,
                        output_limits=(-self.max_power, self.max_power))
            self.pid.reset()
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
        # --- 追加: 初期スパイク抑制 ---
        self._just_started = False
        self._turn_cap = 0             # 現在のturn上限
        self._turn_cap_init = 10       # 初期上限（お好みで 5〜15）
        self._turn_cap_step = 10       # 1tickごとに増やす量
        self._deadband_deg = 1.5       # 微小誤差は無視（1.5〜3.0推奨）
        self._min_turn = 4             # ← 追加: 最小舵（3〜5推奨）
        self._trim_r = 0               # ← 追加: 右モータ微トリム（必要時のみ 2〜4 など）
        # ← デバッグ用カウンタ追加
        self.debug_count = 0

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
            # 初期化時にPID内部状態を完全リセット
            self.pid.reset()
            self.running = True
            self._just_started = True
            self._turn_cap = self._turn_cap_init
            self.logger.info("%+06d %s.gyro run started toward heading=%.1f" % (g_plotter.get_distance(),self.__class__.__name__, self.target_heading))
        # 誤差（[-180,180]へ正規化してからデッドバンド適用）
        err = float(self.target_heading) - current_heading
        if err > 180.0:  err -= 360.0
        if err < -180.0: err += 360.0
        if abs(err) < self._deadband_deg:
            steer = 0
        else:
            # PIDはfloatで受けて最小舵を保証
            steer_f = float(self.pid(current_heading))
            if abs(steer_f) < self._min_turn:
                steer = self._min_turn if steer_f >= 0.0 else -self._min_turn
            else:
                steer = int(steer_f)

        # ソフトスタート：最初の数tickは turn を段階解放
        if self._just_started:
            if steer > 0:
                steer = min(steer, self._turn_cap)
            else:
                steer = max(steer, -self._turn_cap)
            # 上限を拡大していき、十分大きくなったら解除
            self._turn_cap = min(self.power, self._turn_cap + self._turn_cap_step)
            if self._turn_cap >= self.power:
                self._just_started = False
        right = max(-100, min(100, self.power - steer))
        left  = max(-100, min(100, self.power + steer))
        # （必要なら右モータに微トリムを掛けて直進癖を補正）
        right = max(-100, min(100, self.power - steer - self._trim_r))
        left  = max(-100, min(100, self.power + steer))
        # --- クリッピングしない操舵（スケーリング）---
        left_cmd  = self.power + steer
        right_cmd = self.power - steer - self._trim_r   # 右微トリム（必要時のみ作用）
        maxmag = max(100.0, abs(left_cmd), abs(right_cmd))
        scale = 100.0 / maxmag           # maxmag<=100ならscale=1.0
        left  = int(left_cmd  * scale)
        right = int(right_cmd * scale)
        g_right_motor.set_power(right)
        g_left_motor.set_power(left)
        # ---- デバッグ出力を10回だけ ----
        if self.debug_count < 20:
            print(f"[RunByGyro] hdg={current_heading:.1f} tgt={self.target_heading:.1f} "
            f"err={err:.1f} steer={steer} L={left} R={right}")
            self.debug_count += 1
        # --------------------------------
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

# ============================================= 検証中 ビヘイビア =============================================
# 機能ごとに分割
# おそらく、640ピクセル × 480ピクセルの画像で、(0,0)が左上、(639,479)が右下
# カメラの新常識: 上側:0.0～下側:FRAME_HEIGHT
# cy: 小さいほど上、大きいほど下
# cx: 小さいほど左、大きいほど右
# dx: cx - FRAME_WIDTH/2 (負: 左、正: 右)
# dy: cy - FRAME_HEIGHT/2 (負: 上、正: 下)
# area: 点の大きさ（面積）
# 青点検出ビヘイビア
class DetectBlueDot(Behaviour):
    def __init__(self, name: str, max_cy_ratio: float = 0.66):
        super().__init__(name)
        self.max_cy = int(FRAME_HEIGHT * max_cy_ratio)

    def update(self) -> Status:
        found, cx, cy, area = g_video.get_blue_info()
        if not found:
            print("[DetectBlueDot] Blue dot not found.")
            return Status.RUNNING
        
        if cy > self.max_cy:
            print(f"[DetectBlueDot] Blue dot too low in image (cy={cy}). Ignored.")
            return Status.RUNNING

        # データを共有変数に保存（外部で読み取れるように）
        g_shared["blue_detected"] = {
            "found": True,
            "cx": cx,
            "cy": cy,
            "area": area
        }
        print(f"[DetectBlueDot] ★★★ Blue dot detected ★★★ at ({cx}, {cy}), area={area}")
        return Status.SUCCESS

# 青点に向けた旋回ビヘイビア
class TurnToBlueDot(Behaviour):
    def __init__(self, name: str,
                 gyro_p: float = 1.1, gyro_i: float = 0.001, gyro_d: float = 0.03,
                 angle_margin: float = 2.0,
                 camera_fov_deg: float = 60.0,
                 min_pwm: int = 10):
        super().__init__(name)
        self.gyro_p = gyro_p
        self.gyro_i = gyro_i
        self.gyro_d = gyro_d
        self.angle_margin = angle_margin
        self.camera_fov_deg = camera_fov_deg
        self.min_pwm = min_pwm
        self.pid = None
        self.target_angle = None
    
    def update(self) -> Status:
        blue = g_shared.get("blue_detected")
        # 青色データがない場合failure
        if not blue or not blue.get("found"):
            print("[TurnToBlueDot] No blue dot data. Aborting.")
            return Status.FAILURE

        cx = blue["cx"]
        dx = cx - (FRAME_WIDTH // 2)
        px_per_deg = FRAME_WIDTH / self.camera_fov_deg
        rel_angle = dx / px_per_deg

        current_angle = g_gyro_sensor.get_angle()

        if self.pid is None:
            self.target_angle = current_angle + rel_angle
            self.pid = PID(self.gyro_p, self.gyro_i, self.gyro_d,
                           setpoint=self.target_angle,
                           sample_time=EXEC_INTERVAL,
                           output_limits=(-40, 40))
            self.pid.reset()
            print(f"[TurnToBlueDot] Turning to target angle: {self.target_angle:.2f}")
            return Status.RUNNING

        error = self.target_angle - current_angle
        error = (error + 180) % 360 - 180  # wrap error to [-180, 180]

        if abs(error) < self.angle_margin:
            g_left_motor.set_power(0)
            g_right_motor.set_power(0)
            print("[TurnToBlueDot] Angle aligned.")
            return Status.SUCCESS

        turn_pwm = int(self.pid(current_angle))
        if 0 < abs(turn_pwm) < self.min_pwm:
            turn_pwm = self.min_pwm if turn_pwm > 0 else -self.min_pwm

        g_left_motor.set_power(-turn_pwm)
        g_right_motor.set_power(turn_pwm)
        print(f"[TurnToBlueDot] Turning... error={error:.2f}, pwm={turn_pwm}")
        return Status.RUNNING

    def terminate(self, new_status):
        g_left_motor.set_power(0)
        g_right_motor.set_power(0)
        self.pid = None
        self.target_angle = None

# 大円のグレーまで走行し、検出直後停止するビヘイビア
class ForwardUntilGray(Behaviour):
    def __init__(self, name: str, threshold: int = 85, power: int = 40):
        super().__init__(name)
        self.threshold = threshold
        self.power = power

    def update(self) -> Status:
        brightness = g_color_sensor.get_brightness()
        print(f"[ForwardUntilGray] Brightness: {brightness}")

        if brightness < self.threshold:
            g_left_motor.set_power(0)
            g_right_motor.set_power(0)
            print("[ForwardUntilGray] Gray detected. Stopping.")
            return Status.SUCCESS

        g_left_motor.set_power(self.power)
        g_right_motor.set_power(self.power)
        return Status.RUNNING

    def terminate(self, new_status):
        g_left_motor.set_power(0)
        g_right_motor.set_power(0)

# 現在のヨー角（角度）を取得して表示するビヘイビア
class GetCurrentYaw(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def update(self) -> Status:
        yaw = g_gyro_sensor.get_angle()  # ジャイロセンサーから角度を取得
        print(f"[GetCurrentYaw] Current yaw angle: {yaw:.2f}°")
        self.logger.info("%+06d %s.yaw=%.2f°" % (g_plotter.get_distance(), self.__class__.__name__, yaw))
        return Status.SUCCESS


# 上記に分割済みのため、使用不可============================================================
# カメラで青を中央に合わせる→近づいたらジャイロ固定で一定距離前進して置く（バック禁止）
class AimBlueThenGo(Behaviour):
    def __init__(self, name: str,
                 align_px: int = 3,
                 gyro_p: float = 1.1, gyro_i: float = 0.001, gyro_d: float = 0.03,
                 min_cy_ratio: float = 0.60,
                 angle_margin: float = 2.0):
        super().__init__(name)
        self.align_px = align_px
        self.gyro_p = gyro_p
        self.gyro_i = gyro_i
        self.gyro_d = gyro_d
        self.min_cy_ratio = min_cy_ratio
        self.angle_margin = angle_margin
        self.phase = "aim"
        self.running = False
        self.target_angle = None
        self.pid = None

    def update(self) -> Status:
        if not self.running:
            self.running = True
            self.phase = "aim"
            print("[AimBlueThenGo] start aiming...")

        found, cx, cy, area = g_video.get_blue_info()
        if not found:
            # 青が見えない場合は停止
            g_left_motor.set_power(0)
            g_right_motor.set_power(0)
            print("[AimBlueThenGo] Blue not found, stop.")
            return Status.RUNNING

        # 1. 青点の中心から目標角度を算出
        dx = cx - (FRAME_WIDTH // 2)
        # カメラ画角からピクセル→角度変換（例: 320px=60度なら1px=0.1875度）
        # 実際のカメラ画角に合わせて調整してください
        CAMERA_FOV_DEG = 60.0
        px_per_deg = FRAME_WIDTH / CAMERA_FOV_DEG
        rel_angle = dx / px_per_deg  # 右が正、左が負

        # 2. 目標角度を現在角度に加算（相対角度で旋回）
        current_angle = g_gyro_sensor.get_angle()
        if self.phase == "aim":
            self.target_angle = current_angle + rel_angle
            # PID初期化
            self.pid = PID(self.gyro_p, self.gyro_i, self.gyro_d,
                           setpoint=self.target_angle,
                           sample_time=EXEC_INTERVAL,
                           output_limits=(-40, 40))
            self.pid.reset()
            print(f"[AimBlueThenGo] Blue center dx={dx}, rel_angle={rel_angle:.2f}, target={self.target_angle:.2f}")
            self.phase = "turn"
            return Status.RUNNING

        # 3. ジャイロで目標角度まで旋回
        if self.phase == "turn":
            current_angle = g_gyro_sensor.get_angle()
            error = self.target_angle - current_angle
            if error > 180: error -= 360
            if error < -180: error += 360
            print(f"[DEBUG] current={current_angle:.2f}, target={self.target_angle:.2f}, error={error:.2f}")
            if abs(error) < self.angle_margin:
                g_left_motor.set_power(0)
                g_right_motor.set_power(0)
                print(f"[AimBlueThenGo] Finished turning. error={error:.2f}")
                self.phase = "done"
                return Status.SUCCESS
            turn_pwm = int(self.pid(current_angle))
            print(f"[DEBUG] pid_output={turn_pwm}")
            # 最小PWM保証
            min_turn_pwm = 10
            if 0 < abs(turn_pwm) < min_turn_pwm:
                turn_pwm = min_turn_pwm if turn_pwm > 0 else -min_turn_pwm
            g_left_motor.set_power(-turn_pwm)
            g_right_motor.set_power(turn_pwm)
            print(f"[AimBlueThenGo] Turning... error={error:.2f}, pwm={turn_pwm}")
            return Status.RUNNING

        if self.phase == "done":
            g_left_motor.set_power(0)
            g_right_motor.set_power(0)
            return Status.SUCCESS

        g_left_motor.set_power(0)
        g_right_motor.set_power(0)
        return Status.SUCCESS

# カメラで青色を探して見つけたら停止するクラス
# search_range: (x, y, w, h)で指定した範囲の平均色を取得して青色を検知
# 640ピクセル × 480ピクセルの画像で、(0,0)が左上、(639,479)が右下
# カメラ全体を範囲にするなら (0, 0, 640, 480)らしい
# search_step: ヨー角を動かすステップ（度数）
# max_angle: 探索の最大角度（-max_angleから+max_angleまで動かす）
# class SearchBlueAndStop(Behaviour):
#     def __init__(self, name: str, search_range: tuple[int, int, int, int], search_step: int, max_angle: int):
#         super().__init__(name)
#         self.search_range = search_range  # (x, y, w, h)
#         self.search_step = search_step
#         self.max_angle = max_angle
#         self.current_angle = -max_angle
#         self.found_angle = None
#         self.state = "search"

#     def update(self) -> Status:
#         if self.state == "search":
#             # 指定範囲の平均色取得
#             r, g, b = g_video.get_area_average_color(*self.search_range)
#             max_rgb = max(r, g, b, 1)
#             r_norm, g_norm, b_norm = r / max_rgb, g / max_rgb, b / max_rgb
#             h, s, v = colorsys.rgb_to_hsv(r_norm, g_norm, b_norm)
#             h_deg = int(h * 360)
#             s_per = int(s * 100)
#             v_per = int(v * 100)
#             print(f"[SearchBlue] angle={self.current_angle} h={h_deg} s={s_per} v={v_per}")
#             if 200 <= h_deg <= 260 and s_per > 40 and v_per > 30:
#                 self.found_angle = self.current_angle
#                 self.state = "stop"
#                 return Status.RUNNING
#             # ヨー角を動かす
#             g_right_motor.set_power(30)
#             g_left_motor.set_power(-30)
#             time.sleep(0.05)
#             self.current_angle += self.search_step
#             if self.current_angle > self.max_angle:
#                 g_right_motor.set_power(0)
#                 g_left_motor.set_power(0)
#                 return Status.FAILURE
#             return Status.RUNNING
#         elif self.state == "stop":
#             # 検知した角度で停止
#             g_right_motor.set_power(0)
#             g_left_motor.set_power(0)
#             print(f"[SearchBlue] Stop at angle={self.found_angle}")
#             return Status.SUCCESS
#         return Status.RUNNING

# class DetectBlueInCenterArea(Behaviour):
#     def __init__(self, name: str, radius: int = 20):
#         super().__init__(name)
#         self.radius = radius  # 中心円の半径（ピクセル単位など）
#         self.logger.debug("%s.__init__()" % (self.__class__.__name__))

#     def update(self) -> Status:
#         # カメラ画像の中心円エリアのRGB値を取得（仮のAPI例）
#         r, g, b = g_video.get_center_area_color(radius=self.radius)
#         max_rgb = max(r, g, b, 1)
#         r_norm = r / max_rgb
#         g_norm = g / max_rgb
#         b_norm = b / max_rgb
#         h, s, v = colorsys.rgb_to_hsv(r_norm, g_norm, b_norm)
#         h_deg = int(h * 360)
#         s_per = int(s * 100)
#         v_per = int(v * 100)
#         # 青色の判定（例: h=200〜260, s/vは適宜調整）
#         if 200 <= h_deg <= 260 and s_per > 40 and v_per > 30:
#             self.logger.info("%s: Blue detected in center area! h=%d s=%d v=%d" % (self.__class__.__name__, h_deg, s_per, v_per))
#             print(f"[DetectBlueInCenterArea] BLUE! h={h_deg} s={s_per} v={v_per}")
#             return Status.SUCCESS
#         return Status.RUNNING
# ============================================= 検証中 ビヘイビア =============================================

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
    def __init__(self, name: str, black_thresh: int = 5, gray_brightness: int = 85, gray_saturation: int = 40, target_count: int = 3):
        super().__init__(name)
        self.black_thresh = black_thresh
        self.gray_brightness = gray_brightness
        self.gray_saturation = gray_saturation
        self.target_count = target_count
        self.count = 0

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
        brightness = g_color_sensor.get_brightness()
        if brightness < self.black_thresh:
            self.count += 1
            print(f"黒検知回数: {self.count}")
            if self.count >= self.target_count:
                return Status.SUCCESS
            else:
                return Status.RUNNING
        if brightness < self.gray_brightness and s_per > self.gray_saturation:
            self.count += 1
            print(f"グレー検知回数: {self.count} (brightness={brightness}(saturation={s_per}))")
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
        g_right_motor.set_power(70)
        time.sleep(0.6)  # 必要に応じて調整
        g_left_motor.set_power(80)
        g_right_motor.set_power(100)
        time.sleep(0.6)
        # # 止める
        # g_left_motor.set_power(0)
        # g_right_motor.set_power(0)

        # 左に戻す
        g_left_motor.set_power(60)
        g_right_motor.set_power(100)
        time.sleep(0.6)  # 必要に応じて調整 
        # g_left_motor.set_power(0)
        # g_right_motor.set_power(0)

        # ライン復帰
        g_left_motor.set_power(100)
        g_right_motor.set_power(100)
        time.sleep(0.55)
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

class ResetGyroPID(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.done = False

    def update(self) -> Status:
        if not self.done:
            g_gyro_sensor.reset()
            print("[ResetGyroPID] gyro reset done")
            self.done = True
            return Status.SUCCESS
        return Status.SUCCESS

    # --- 灰色検知までジャイロで直進 ---
def make_forward_until_gray_by_gyro():
    node = Parallel(name="ForwardUntilGraybyGyro", policy=ParallelPolicy.SuccessOnOne())
    node.add_children([
        DetectBlackCount(
            name="detect_black_count_smartcarry_start",
            black_thresh=5,
            gray_brightness=85,
            gray_saturation=15,
            target_count=1
        ),
        RunByGyro(
            name="run_straight_until_gray_smartcarry_start",
            target=0,          # ← すべて target=0 で固定
            power=40,
            pid_p=1.1,
            pid_i=0.001,
            pid_d=0.03,
            target_type=HeadingType.RELATIVE
        ),
    ])
    return node


    # --- ほんの少しジャイロで直進 ---
def make_gostraightbygyro_short():
    node = Parallel(name="GostraightbyGyro_short", policy=ParallelPolicy.SuccessOnOne())
    node.add_children([
        IsDistancePassed(name="distance_passed_short", target_distance=50),
        RunByGyro(
            name="run_straight_short_smartcarry_start",
            target=0,          # ← すべて target=0
            power=40,
            pid_p=1.1,
            pid_i=0.001,
            pid_d=0.03,
            target_type=HeadingType.RELATIVE
        ),
    ])
    return node

def build_behaviour_tree() -> BehaviourTree:
    # 各ノードを定義

# =========================================================== LAP走行 ===========================================================
    # コース前半：直線⇒オブジェクト回避⇒最初のカーブまで走行⇒向正面走行⇒次のカーブで曲がって、LAPまで走行

    # ---オブジェクト回避シーケンス---
    # [Purpose] オブジェクトを回避する
    # [Exit]    距離2430⇒AvoidObstacleArcFullのSuccesse
    # [Control] Sequence(memory=True) : 子を順番に実行（成功で次へ）
    avoid_seq = Sequence(name="avoid_seq", memory=True)
    avoid_seq.add_children([
        IsDistancePassed(name="distance_passed", target_distance=2430),
        AvoidObstacleArcFull(name="arc_avoid")
    ])

    # --- 直進(ジャイロ) or 回避(距離到達) の競合 ---
    # [Purpose] 回避条件が整えば回避に切替、それまではジャイロ直進で前進
    # [Exit]    avoid_seqのSuccesse
    # [Control] Parallel(SuccessOnOne)
    obstacle_Parallel = Parallel(name="obstacle_or_gyro", policy=ParallelPolicy.SuccessOnOne())
    obstacle_Parallel.add_children([
        # avoid_seq, # 回避条件（距離到達→回避実行）
        IsDistancePassed(name="distance_passed", target_distance=2430),
        RunByGyro(name="object_avoid_gyro", target=0, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    obstacle_avoid_start_Parallel = Parallel(name="obstacle_avoid_start", policy=ParallelPolicy.SuccessOnOne())
    obstacle_avoid_start_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=400),
        RunByGyro(name="object_avoid_gyro", target=25, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    obstacle_avoid_middle_Parallel = Parallel(name="obstacle_avoid_middle", policy=ParallelPolicy.SuccessOnOne())
    obstacle_avoid_middle_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=850),
        RunByGyro(name="object_avoid_gyro", target=0, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    obstacle_avoid_end_Parallel = Parallel(name="obstacle_avoid_end", policy=ParallelPolicy.SuccessOnOne())
    obstacle_avoid_end_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=600),
        RunByGyro(name="object_avoid_gyro", target=-45, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # # オブジェクトを無視してジャイロで真っ直ぐパターン
    # gyro_obstacle_ignore_Parallel = Parallel(name="gyro_obstacle_ignore", policy=ParallelPolicy.SuccessOnOne())
    # gyro_obstacle_ignore_Parallel.add_children([
    #     IsDistancePassed(name="distance_passed", target_distance=4675),
    #     RunByGyro(name="run_back_GoBlackLine", target=0, power=100,
    #             pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    # ])

    # --- 回避後：カーブ手前までジャイロ直進 ---
    # [Purpose] 回避直後の姿勢を維持しつつ所定距離だけ前進
    # [Exit]    距離950
    # [Control] Parallel(SuccessOnOne)
    gyro_obstacle_end_to_first_curve_Parallel = Parallel(name="gyro_obstacle_end_to_first_curve", policy=ParallelPolicy.SuccessOnOne())
    gyro_obstacle_end_to_first_curve_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=700),
        RunByGyro(name="gyro_obstacle_end_to_first_curve", target=0, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 回避後：カーブ手前までジャイロ直進 ---
    # [Purpose] 回避直後の姿勢を維持しつつ所定距離だけ前進
    # [Exit]    距離950
    # [Control] Parallel(SuccessOnOne)
    gyro_first_curve_45degree_Parallel = Parallel(name="gyro_curve_45degree", policy=ParallelPolicy.SuccessOnOne())
    gyro_first_curve_45degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=50),
        RunByGyro(name="gyro_first_curve_45degree", target=45, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 回避後：カーブ手前までジャイロ直進 ---
    # [Purpose] 回避直後の姿勢を維持しつつ所定距離だけ前進
    # [Exit]    距離950
    # [Control] Parallel(SuccessOnOne)
    gyro_first_curve_90degree_Parallel = Parallel(name="gyro_curve_90degree", policy=ParallelPolicy.SuccessOnOne())
    gyro_first_curve_90degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=50),
        RunByGyro(name="run_back_GoBlackLine", target=90, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 向正面へ（LAP手前のカーブまで） ---
    # [Purpose] 所定の角度(-90)を維持して長めの直線を前進
    # [Exit]    距離3000
    # [Control] Parallel(SuccessOnOne)
    gyro_mukoujoumen_Parallel = Parallel(name="gyro_mukoujoumen", policy=ParallelPolicy.SuccessOnOne())
    gyro_mukoujoumen_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=2700),
        RunByGyro(name="gyro_mukoujoumen", target=90, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 回避後：カーブ手前までジャイロ直進 ---
    # [Purpose] 回避直後の姿勢を維持しつつ所定距離だけ前進
    # [Exit]    距離950
    # [Control] Parallel(SuccessOnOne)
    gyro_second_curve_135degree_Parallel = Parallel(name="gyro_curve_135degree", policy=ParallelPolicy.SuccessOnOne())
    gyro_second_curve_135degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=50),
        RunByGyro(name="gyro_second_curve_135degree", target=135, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 回避後：カーブ手前までジャイロ直進 ---
    # [Purpose] 回避直後の姿勢を維持しつつ所定距離だけ前進
    # [Exit]    距離950
    # [Control] Parallel(SuccessOnOne)
    gyro_second_curve_180degree_Parallel = Parallel(name="gyro_curve_180degree", policy=ParallelPolicy.SuccessOnOne())
    gyro_second_curve_180degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=50),
        RunByGyro(name="gyro_second_curve_180degree", target=180, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- LAP手前のカーブからLAPまで ---
    # [Purpose] 所定の角度(-180)を維持してLAP地点まで前進
    # [Exit]    距離500
    # [Control] Parallel(SuccessOnOne)
    gyro_gotolap_Parallel = Parallel(name="gyro_gotolap", policy=ParallelPolicy.SuccessOnOne())
    gyro_gotolap_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=500),
        RunByGyro(name="gyro_gotolap", target=180, power=100,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- LAP完了後：センタートレース開始、青検知 or 距離でSuccess ---
    # [Purpose] ライントレースしつつ、ダブルループへ入るトリガーを検出（青/距離）
    # [Exit]    青検知 or 距離500
    # [Control] Parallel(SuccessOnOne)
    traceline_cam_start_doubleloop_Parallel = Parallel(name="detectblue_or_distance_or_trace", policy=ParallelPolicy.SuccessOnOne())
    traceline_cam_start_doubleloop_Parallel.add_children([
        DetectBlue(name="detect_blue"),
        IsDistancePassed(name="distance_passed", target_distance=500),
        TraceLineCam(name="traceline_cam_lapfinish",power=40, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.CENTER)
    ])

# =========================================================== ダブルループ処理 ===========================================================
    # コース中盤：大円(内エッジ切替/青検知 or 距離) → 小円(内エッジ切替/青検知 or 距離) → 大円(センターへ切替/青検知 or 距離) → 脱出の流れ

    # --- ダブルループ進入調整（エッジ切替のみ） ---
    # [Purpose] 青検知をしないように距離制御のみでライントレースし、大円に入る（エッジは内エッジに切替）
    # [Exit]    距離750
    # [Control] Parallel(SuccessOnOne)
    Doubleloop_start_Parallel = Parallel(name="Doubleloop_start", policy=ParallelPolicy.SuccessOnOne())
    Doubleloop_start_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=950),
        TraceLineCam(name="traceline_start_doubleloop",power=48, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=30,trace_side=TraceSide.NORMAL),
    ])

    # --- 大円：内エッジでライントレース（距離フェイルセーフ付き） ---
    # [Purpose] 内エッジで安定トレースしつつ青検知をして次のエッジ切替の処理へ
    # [Exit]    青検知 or 距離1850
    # [Control] Parallel(SuccessOnOne)
    Bigcircle_Linetrace_InnerEdge_parallel = Parallel(name="Bigcircle_Linetrace_InnerEdge",policy=ParallelPolicy.SuccessOnOne())
    Bigcircle_Linetrace_InnerEdge_parallel.add_children([
        DetectBlue(name="detect_blue"),
        IsDistancePassed(name="distance_passed", target_distance=1200),      #青検知しなかったとき用の距離制御
        TraceLineCam(name="traceline_cam_inner_egde",power=48, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.NORMAL),
    ])

    # --- 小円進入チューニング（エッジ切替） ---
    # [Purpose] 小円のエッジに合わせる（エッジを切り替えて小円の内エッジへ）
    # [Exit]    距離800
    # [Control] Parallel(SuccessOnOne)
    SmallCircleEntryTuning_Parallel = Parallel(name="SmallCircleEntryTuning", policy=ParallelPolicy.SuccessOnOne())
    SmallCircleEntryTuning_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=800),
        TraceLineCam(name="traceline_entry_smallcircle",power=48, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.OPPOSITE),
    ])

    # --- 小円: 内エッジでライントレース（距離フェイルセーフ付き） ---
    # [Purpose] 内エッジで安定トレースしつつ青検知をして次のエッジ切替の処理へ
    # [Exit]    青検知 or 距離2200
    # [Control] Parallel(SuccessOnOne)
    SmallCircle_Linetrace_InnerEdge_parallel = Parallel(name="Smallcircle_Linetrace_InnerEdge",policy=ParallelPolicy.SuccessOnOne())
    SmallCircle_Linetrace_InnerEdge_parallel.add_children([
        DetectBlue(name="detect_blue"),
        IsDistancePassed(name="distance_passed", target_distance=2200),      #青検知しなかったとき用の距離制御
        TraceLineCam(name="traceline_cam_inner_egde",power=43, pid_p=2.0, pid_i=0.0012, pid_d=0.1,
        gs_min=0, gs_max=50,trace_side=TraceSide.OPPOSITE),
    ])

    # --- 大円への戻りチューニング（エッジ切替） ---
    # [Purpose] 大円復帰に向けてエッジ切り替え（エッジを切り替えて大円のセンターへ）
    # [Exit]    距離400
    # [Control] Parallel(SuccessOnOne)
    BigCircleEntryTuning_Parallel = Parallel(name="BigCircleEntryTuning", policy=ParallelPolicy.SuccessOnOne())
    BigCircleEntryTuning_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=600),
        TraceLineCam(name="traceline_entry_bigcircle",power=42, pid_p=2.0, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.NORMAL),
    ])
    
    # --- 大円： センターでライントレース ---
    # [Purpose] 青検知するまでセンターでライントレース（青検知するようにセンターにしている）
    # [Exit]    青検知
    # [Control] Parallel(SuccessOnOne)
    BigCircle_Linetrace_CenterEdge_parallel = Parallel(name="BigCircle_Linetrace_CenterEdge",policy=ParallelPolicy.SuccessOnOne())
    BigCircle_Linetrace_CenterEdge_parallel.add_children([
        DetectBlue(name="detect_blue"),
        IsDistancePassed(name="distance_passed", target_distance=800),
        TraceLineCam(name="BigCircle_Linetrace_CenterEdge",power=42, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=40,trace_side=TraceSide.CENTER),
    ])

    # --- ダブルループ脱出 ---
    # [Purpose] 青検知後に外エッジに切り替えることで元のメインのラインへ戻る
    # [Exit]    距離800
    # [Control] Parallel(SuccessOnOne)
    Escape_double_loop_Parallel = Parallel(name="Escape_double_loop", policy=ParallelPolicy.SuccessOnOne())
    Escape_double_loop_Parallel.add_children([
        IsDistancePassed(name="distance_passed", target_distance=800),
        TraceLineCam(name="traceline_cam_center_egde",power=48, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.OPPOSITE),
    ])

# =========================================================== スマートキャリーツイン ===========================================================

    # --- 最初のボトルまでライントレース（ボトル下の青検知） ---
    # [Purpose] 最初のボトルまでライントレースをする
    # [Exit]    青検知
    # [Control] Parallel(SuccessOnOne)
    traceline_cam_smacary_Parallel = Parallel(name="detectblue_or_trace", policy=ParallelPolicy.SuccessOnOne())
    traceline_cam_smacary_Parallel.add_children([
        DetectBlue(name="detect_blue"),
        TraceLineCam(name="detectblue_or_trace",power=50, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.CENTER),
    ])

    # --- ボトルを捕まえるために、少し円弧走行 ---
    # [Purpose] ボトルをしっかり捕まえにいく
    # [Exit]    距離450 or 750（ゲートの位置によって変化 ※front or back）
    # [Control] Parallel(SuccessOnOne)
    BringObject_to_Gate_Parallel = Parallel(name="BringObject_to_Gate", policy=ParallelPolicy.SuccessOnOne())
    BringObject_to_Gate_Parallel.add_children([
        # -----ゲートの位置で距離が変わるようになっている⇒gate_value(300=front, 500=back)
        IsDistancePassed(name="distance_passed", target_distance=gate_value(450, 700)),
        RunByGyro(name="run straight_SpinAndRun", target=-160, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- ゲート通過 ---
    # [Purpose] ゲートを通過する
    # [Exit]    距離2150 or 2000（ゲートの位置によって変化 ※front or back）
    # [Control] Parallel(SuccessOnOne)
    SpinAndRun_Parallel = Parallel(name="SpinAndRun", policy=ParallelPolicy.SuccessOnOne())
    SpinAndRun_Parallel.add_children([
        # -----ゲートの位置で距離が変わるようになっている⇒gate_value(300=front, 500=back)
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=gate_value(1900, 2000)),
        RunByGyro(name="run straight_SpinAndRun", target=-93, power=80,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- ゲート通過後にボトルを取りこぼさない ---
    # [Purpose] 45度で少し走ることで取りこぼさない
    # [Exit]    距離50
    # [Control] Parallel(SuccessOnOne)
    Spintotarget_45degree_Parallel = Parallel(name="Spintotarget_45degree", policy=ParallelPolicy.SuccessOnOne())
    Spintotarget_45degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=50),
        RunByGyro(name="run straight_SpinAndRun", target=-45, power=70,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 2段階右折で確実に運ぶ ---
    # [Purpose] 45度で少し走ったあとに90度にすることで取りこぼさない
    # [Exit]    距離50
    # [Control] Parallel(SuccessOnOne)
    Spintotarget_90degree_Parallel = Parallel(name="Spintotarget_90degree", policy=ParallelPolicy.SuccessOnOne())
    Spintotarget_90degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=gate_value(50, 100)),
        RunByGyro(name="run straight_SpinAndRun", target=0, power=70,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 青検知でスマートキャリーに入ってからターゲットに向かうまで ---
    # [Purpose] 各処理をSuccesseさせる
    # [Exit]    シーケンスがsuccessで完了する
    # [Control] Sequence
    SpinAndRun_Sequence = Sequence(name="SpinAndRun_Sequence", memory=True)
    SpinAndRun_Sequence.add_children([
        BringObject_to_Gate_Parallel,#     ゲート位置までオブジェクトを運ぶ（ゲート位置によって距離制御あり）
        SpinAndRun_Parallel,#              ゲートを通過する
        Spintotarget_45degree_Parallel,
        Spintotarget_90degree_Parallel,
    ])
    """
    # --- 灰色検知までジャイロで直進 ---
    ForwardUntilGraybyGyro = Parallel(name="ForwardUntilGraybyGyro", policy=ParallelPolicy.SuccessOnOne())
    ForwardUntilGraybyGyro.add_children([
        DetectBlackCount(name="detect_black_count_smartcarry_start", target_count=1, gray_brightness=85, gray_saturation=15),
        RunByGyro(name="run_straight_until_gray_smartcarry_start", target=0, power=40,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
    ])
    # --- ほんの少しジャイロで直進 ---
    GostraightbyGyro_short = Parallel(name="GostraightbyGyro_short", policy=ParallelPolicy.SuccessOnOne())
    GostraightbyGyro_short.add_children([
        IsDistancePassed(name="distance_passed_short", target_distance=50),
        RunByGyro(name="run_straight_short_smartcarry_start", target=0, power=40,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.RELATIVE),
    ])
    """
    # --- 最初のボトルを置く処理 ---
    first_landing_prepare_sequence = Sequence(name="first_landing_prepare", memory=True)
    first_landing_prepare_sequence.add_children([
    #    DetectBlueDot(name="blue_detected_for_smartcarry_start"),
    #    TurnToBlueDot(name="turn_to_blue_dot_start"),
    #    GetCurrentYaw(name="get_current_yaw_smartcarry_start"),使わない
    #    ForwardUntilGraybyGyro,
        make_forward_until_gray_by_gyro(),
        make_gostraightbygyro_short(),
        make_forward_until_gray_by_gyro(),
        make_gostraightbygyro_short(),
        make_forward_until_gray_by_gyro(),
    ])

    # --- 最初のボトルをターゲットに置く ---
    # [Purpose] 中心に近づけるようにボトルを置く
    # [Exit]    距離330 or 580（ゲートの位置によって変化）
    # [Control] Parallel(SuccessOnOne)
    smart_carry_puton_first_Parallel = Parallel(name="smart_carry_puton", policy=ParallelPolicy.SuccessOnOne())
    smart_carry_puton_first_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=gate_value(330, 550)),
        RunByGyro(name="run straight_smart_carry_puton", target=0, power=75,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- ボトルをバックすることで置く ---
    # [Purpose] ボトルを置く
    # [Exit]    距離250
    # [Control] Parallel(SuccessOnOne)
    After_puton_back_first_Parallel = Parallel(name="After_puton_back", policy=ParallelPolicy.SuccessOnOne())
    After_puton_back_first_Parallel.add_children([
        IsDistancePassed(name="distance_passed_back", target_distance=250),
        RunAsInstructed(name="go_straight_3", pwm_l=60, pwm_r=60),
    ])

    # --- 次のボトルまで進む ---
    # [Purpose] 次のボトルの後ろまで進むように距離を調整
    # [Exit]    距離750
    # [Control] Parallel(SuccessOnOne)
    Go_to_next_bottle_Parallel = Parallel(name="Go_to_next_bottle", policy=ParallelPolicy.SuccessOnOne())
    Go_to_next_bottle_Parallel.add_children([
        IsDistancePassed(name="Go_to_next_bottle", target_distance=400),
        RunByGyro(name="Go_to_next_bottle_by_Gyro", target=-180, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 次のボトル前の黒線を検知 ---
    # [Purpose] 黒線を検知する
    # [Exit]    距離300 or 黒線検知
    # [Control] Parallel(SuccessOnOne)
    DetectBlackline_before_bottle_Parallel = Parallel(name="DetectBlackline_before_bottle", policy=ParallelPolicy.SuccessOnOne())
    DetectBlackline_before_bottle_Parallel.add_children([
        IsOnBlackLine_running(name="detect_blackline", threshold=20),
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=300),
        RunByGyro(name="run straight_SpinAndRun", target=-265, power=42,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 次のボトル下の赤線を検知 ---
    # [Purpose] 赤線を検知する
    # [Exit]    距離500 or 赤線検知
    # [Control] Parallel(SuccessOnOne)
    traceline_cam_Detectred_Parallel = Parallel(name="traceline_cam_Detectred", policy=ParallelPolicy.SuccessOnOne())
    traceline_cam_Detectred_Parallel.add_children([
        DetectRed(name="detect_red"),
        IsDistancePassed(name="distance_passed_GoBlackLine", target_distance=500),
        TraceLineCam(name="traceline_cam_DetectBlue_GOAL",power=45, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.CENTER),
    ])

    # --- 2段階右折で確実に運ぶ ---
    # [Purpose] 45度で少し走ったあとに90度にすることで取りこぼさない
    # [Exit]    距離50
    # [Control] Parallel(SuccessOnOne)
    Spintotarget_180degree_Parallel = Parallel(name="Spintotarget_180degree", policy=ParallelPolicy.SuccessOnOne())
    Spintotarget_180degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=50),
        RunByGyro(name="run straight_SpinAndRun", target=180, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 2段階右折で確実に運ぶ ---
    # [Purpose] 45度で少し走ったあとに90度にすることで取りこぼさない
    # [Exit]    距離50
    # [Control] Parallel(SuccessOnOne)
    Spintotarget_225degree_Parallel = Parallel(name="Spintotarget_225degree", policy=ParallelPolicy.SuccessOnOne())
    Spintotarget_225degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=50),
        RunByGyro(name="run straight_SpinAndRun", target=225, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 2段階右折で確実に運ぶ ---
    # [Purpose] 45度で少し走ったあとに90度にすることで取りこぼさない
    # [Exit]    距離50
    # [Control] Parallel(SuccessOnOne)
    Spintotarget_315degree_Parallel = Parallel(name="Spintotarget_315degree", policy=ParallelPolicy.SuccessOnOne())
    Spintotarget_315degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=50),
        RunByGyro(name="run straight_SpinAndRun", target=315, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 2段階右折で確実に運ぶ ---
    # [Purpose] 帰りのゲート前まで進む
    # [Exit]    距離50
    # [Control] Parallel(SuccessOnOne)
    Spintotarget_360degree_Parallel = Parallel(name="Spintotarget_360degree", policy=ParallelPolicy.SuccessOnOne())
    Spintotarget_360degree_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=gate_value(1100, 800)),
        RunByGyro(name="run straight_SpinAndRun", target=360, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 帰りのゲートを通る ---
    # [Purpose] 帰りのゲートを通過し、ターゲットの直角位置まで進む
    # [Exit]    距離1200
    # [Control] Parallel(SuccessOnOne)
    Spintotarget_returngate_Parallel = Parallel(name="Spintotarget_returngate_Parallel", policy=ParallelPolicy.SuccessOnOne())
    Spintotarget_returngate_Parallel.add_children([
        IsDistancePassed(name="distance_passed_ThroughTheGate", target_distance=1600),
        RunByGyro(name="run straight_SpinAndRun", target=90, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- 青検知でスマートキャリーに入ってからターゲットに向かうまで ---
    # [Purpose] 各処理をSuccesseさせる
    # [Exit]    シーケンスがsuccessで完了する
    # [Control] Sequence
    ReturnGate_Sequence = Sequence(name="ReturnGate", memory=True)
    ReturnGate_Sequence.add_children([
        Spintotarget_180degree_Parallel,
        Spintotarget_225degree_Parallel,
        Spintotarget_315degree_Parallel,
        Spintotarget_360degree_Parallel,
        Spintotarget_returngate_Parallel,#              ゲートを通過する
    ])

    # --- ジャイロで次のターゲットまで進む ---
    # [Purpose] ゲート通過後、次のターゲットまで進むように距離を調整
    # [Exit]    距離600
    # [Control] Parallel(SuccessOnOne)
    smart_carry_puton_second_Parallel = Parallel(name="smart_carry_puton", policy=ParallelPolicy.SuccessOnOne())
    smart_carry_puton_second_Parallel.add_children([
        IsDistancePassed(name="smart_carry_puton", target_distance=gate_value(850, 500)),
        RunByGyro(name="Gyro_straight_smart_carry_puton_second", target=180, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- バックすることでボトルを置く ---
    # [Purpose] ボトルを置く
    # [Exit]    距離200
    # [Control] Parallel(SuccessOnOne)
    After_puton_back_second_Parallel = Parallel(name="After_puton_back", policy=ParallelPolicy.SuccessOnOne())
    After_puton_back_second_Parallel.add_children([
        IsDistancePassed(name="distance_passed_back", target_distance=400),
        RunAsInstructed(name="go_straight_3", pwm_l=60, pwm_r=60),
    ])

    # --- ボトルを置いた後のバック後に、斜めに走ることでメインのラインに戻ろうとする ---
    # [Purpose] ゲートや障害物にぶつからないように斜めに進む
    # [Exit]    距離800
    # [Control] Parallel(SuccessOnOne)
    DiagonalRun_Parallel = Parallel(name="DiagonalRun", policy=ParallelPolicy.SuccessOnOne())
    DiagonalRun_Parallel.add_children([
        IsDistancePassed(name="DiagonalRun", target_distance=800),
        RunByGyro(name="DiagonalRun_by_Gyro", target=-315, power=60,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- メインのラインに向かって垂直に走る ---
    # [Purpose] メインのラインに向かって垂直に走る
    # [Exit]    距離800 or 黒色検知
    # [Control] Parallel(SuccessOnOne)
    Go_to_blackline_Parallel = Parallel(name="Go_to_blackline", policy=ParallelPolicy.SuccessOnOne())
    Go_to_blackline_Parallel.add_children([
        IsOnBlackLine_running(name="detect_blackline", threshold=5),
        IsDistancePassed(name="distance_passed_GoBlackLine", target_distance=700),
        RunByGyro(name="run_back_DetectBlackLine", target=-270, power=40,
                pid_p=1.1, pid_i=0.001, pid_d=0.03, target_type=HeadingType.ABSOLUTE),
    ])

    # --- ゴールに向かってライントレース ---
    # [Purpose] ライントレースしながらゴールゾーンで止まる
    # [Exit]    距離850 or 青色検知
    # [Control] Parallel(SuccessOnOne)
    traceline_cam_DetectBlue_GOAL_Parallel = Parallel(name="traceline_cam_DetectBlue_GOAL", policy=ParallelPolicy.SuccessOnOne())
    traceline_cam_DetectBlue_GOAL_Parallel.add_children([
        DetectBlue(name="detect_blue"),
        IsDistancePassed(name="distance_passed_GoBlackLine", target_distance=750),
        TraceLineCam(name="traceline_cam_DetectBlue_GOAL",power=48, pid_p=1.75, pid_i=0.0012, pid_d=0.18,
        gs_min=0, gs_max=80,trace_side=TraceSide.NORMAL),
    ])

# =========================================================== loop_01 Start ===========================================================
    # 直線走行⇒オブジェクト回避⇒LAP走行⇒ダブルループ

    loop_01 = Sequence(name="loop_01_with_obstacle_and_doubleloop", memory=True)
    loop_01.add_children([
        #色や明るさを検知できる（ずっとRUNNINGで無限ループ）※次の処理にはいかない仕様
        Detectcolor(name="detectcolor"),
    # ========= LAP走行 ========
        # --- スタートから一定距離直進⇒オブジェクト回避
        obstacle_Parallel,
        obstacle_avoid_start_Parallel,
        obstacle_avoid_middle_Parallel,
        # obstacle_avoid_end_Parallel,
        SpinAround(name="spin_by_before_avoid",
                    target=0,max_power=50,min_power=MIN_POWER,
                    pid_p=1.1,pid_i=0.001,pid_d=0.03,target_type=HeadingType.ABSOLUTE),
        # --- 一定距離走行⇒カーブを曲がる処理⇒向正面走行⇒カーブを曲がる処理⇒LAPまで直進
        gyro_obstacle_end_to_first_curve_Parallel,
        gyro_first_curve_45degree_Parallel,
        gyro_mukoujoumen_Parallel,
        gyro_second_curve_135degree_Parallel,
        # gyro_second_curve_180degree_Parallel,
        gyro_gotolap_Parallel
    ])

# =========================================================== loop_02 Start ===========================================================

    loop_02 = Sequence(name="loop_02_with_doubleloop", memory=True)
    loop_02.add_children([
    # ========= ダブルループ ========     
        # --- LAP完了から大円に移る
        traceline_cam_start_doubleloop_Parallel,
        SpinAround(name="spin_by_start_doubleloop",
                    target=180,max_power=50,min_power=MIN_POWER,
                    pid_p=1.1,pid_i=0.001,pid_d=0.03,target_type=HeadingType.ABSOLUTE),
        Doubleloop_start_Parallel,
        Bigcircle_Linetrace_InnerEdge_parallel,
        # --- 小円に移るときの処理
        SmallCircleEntryTuning_Parallel,
        SmallCircle_Linetrace_InnerEdge_parallel,
        # --- 小円から大円に移るときの処理
        BigCircleEntryTuning_Parallel,
        BigCircle_Linetrace_CenterEdge_parallel,
        # --- ダブルループを抜ける処理
        Escape_double_loop_Parallel,
    ])

# =========================================================== loop_03 Start ===========================================================
    # スマートキャリーツイン⇒ゴールに向かう処理

    loop_03 = Sequence(name="loop_03_with_smart_carry_twin", memory=True)
    loop_03.add_children([
    # ========= スマートキャリーツイン ========
        first_landing_prepare_sequence,
        StopNow(name="stop"),
        TheEnd(name="end"),
        # --- 最初のボトルまでライントレース
        traceline_cam_smacary_Parallel,
        SpinAndRun_Sequence,
        # --- ターゲットにオブジェクトを置く
        smart_carry_puton_first_Parallel,
        # --- バック
        After_puton_back_first_Parallel,
        SpinAround(name="spin by 90 degrees_After_puton_back_first",
                    target=180,max_power=50,min_power=MIN_POWER,
                    pid_p=1.1,pid_i=0.001,pid_d=0.03,target_type=HeadingType.ABSOLUTE),
        # --- 次のボトルへ
        Go_to_next_bottle_Parallel,
        SpinAround(name="spin by 90 degrees_After_puton_back_second",
                    target=-270,max_power=50,min_power=MIN_POWER,
                    pid_p=1.1,pid_i=0.001,pid_d=0.03,target_type=HeadingType.ABSOLUTE),
        DetectBlackline_before_bottle_Parallel,
        SpinAround(name="spin by 90 degrees_detect_red",
                    target=-190,max_power=50,min_power=MIN_POWER,
                    pid_p=1.1,pid_i=0.001,pid_d=0.03,target_type=HeadingType.ABSOLUTE),
        traceline_cam_Detectred_Parallel,
        # --- ゲート復路通過処理
        ReturnGate_Sequence,
        # --- ターゲットに向かう
        smart_carry_puton_second_Parallel,
        # --- バックしてボトルを置く
        After_puton_back_second_Parallel,
        # SpinAround(name="spin by 90 degrees_After_puton_back_second",
        #             target=-320,max_power=50,min_power=MIN_POWER,
        #             pid_p=1.1,pid_i=0.001,pid_d=0.03,target_type=HeadingType.ABSOLUTE),
        # # --- 45度斜めに走る
        # DiagonalRun_Parallel,
        # SpinAround(name="spin by 90 degrees_GoBlackLine_1",
        #             target=-315,max_power=50,min_power=MIN_POWER,
        #             pid_p=1.1,pid_i=0.001,pid_d=0.03,target_type=HeadingType.ABSOLUTE),
        # --- メインのラインまで垂直に走る
        Go_to_blackline_Parallel,
        SpinAround(name="spin by 90 degrees_DetectBlackLine",
                    target=160, max_power=50, min_power=MIN_POWER,
                    pid_p=1.1, pid_i=0.001, pid_d=0.03,target_type=HeadingType.ABSOLUTE),
        # --- ゴールに向かう
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
        # loop_01,#LAP
        # loop_02,#ダブルループ
        loop_03,#スマートキャリーからゴールまで
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
