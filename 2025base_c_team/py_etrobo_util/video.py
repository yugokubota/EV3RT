import sys
import platform
if platform.python_implementation() == 'CPython':
    sys.path.append('/usr/lib/python3/dist-packages')
elif platform.python_implementation() == 'PyPy':
    sys.path.append('/usr/local/lib/pypy3/dist-packages')
import cv2
import math
import numpy as np
from enum import Enum
from .plotter import Plotter
from etrobo_python import Hub, Motor, ColorSensor, SonarSensor

def round_up_to_odd(f) -> int:
    return int(np.ceil(f / 2.) * 2 + 1)

# frame size for Raspberry Pi USB camera capture
IN_FRAME_WIDTH  = 640
IN_FRAME_HEIGHT = 480

# frame size for OpenCV
#FRAME_WIDTH  = 160
#FRAME_HEIGHT = 120
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

CROP_WIDTH     = int(13*FRAME_WIDTH/16) # for full angle
#CROP_WIDTH     = int(9*FRAME_WIDTH/16)
CROP_HEIGHT    = int(3*FRAME_HEIGHT/8) # for full angle
#CROP_HEIGHT    = int(2*FRAME_HEIGHT/8)
CROP_U_LIMIT   = FRAME_HEIGHT-CROP_HEIGHT
CROP_D_LIMIT   = FRAME_HEIGHT
CROP_L_LIMIT   = int((FRAME_WIDTH-CROP_WIDTH)/2)
CROP_R_LIMIT   = (CROP_L_LIMIT+CROP_WIDTH)
MORPH_KERNEL_SIZE = round_up_to_odd(int(FRAME_WIDTH/48))
ROI_BOUNDARY   = int(FRAME_WIDTH/10)
LINE_THICKNESS = int(FRAME_WIDTH/80)
CIRCLE_RADIUS  = int(FRAME_WIDTH/40)
SCAN_V_POS     = int(13*FRAME_HEIGHT/16 - LINE_THICKNESS) # for full angle
#SCAN_V_POS     = int(16*FRAME_HEIGHT/16 - LINE_THICKNESS)

# frame size for X11 painting
OUT_FRAME_WIDTH  = 160
OUT_FRAME_HEIGHT = 120


class TraceSide(Enum):
    NORMAL = "Normal"
    OPPOSITE = "Opposite"
    RIGHT = "Right"
    LEFT = "Left"
    CENTER = "Center"


class Video(object):
    def __init__(self):
        cv2.setLogLevel(3) # LOG_LEVEL_WARNING
        # set number of threads
        #cv2.setNumThreads(0)
        # prepare the camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        self.cap.set(cv2.CAP_PROP_FPS,90)

        # initial region of interest
        self.roi = (CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT)
        # prepare and keep kernel for morphology
        self.kernel = np.ones((MORPH_KERNEL_SIZE,MORPH_KERNEL_SIZE), np.uint8)
        # initial trace target
        self.cx = int(FRAME_WIDTH/2)
        self.cy = SCAN_V_POS
        self.mx = self.cx
        # default values
        self.gsmin = 0
        self.gsmax = 100
        self.trace_side = TraceSide.NORMAL
        self.range_of_edges = 0
        self.theta:float = 0.0
        self.target_insight = False

        # --- 追加: 青ターゲットの情報 ---
        self.blue_found = False
        self.blue_cx = 0
        self.blue_cy = 0
        self.blue_area = 0

    def __del__(self):
        cv2.destroyAllWindows
        self.cap.release()


    def process(self,
                plotter: Plotter,
                hub: Hub,
                arm_motor: Motor,
                right_motor: Motor,
                left_motor: Motor,
                color_sensor: ColorSensor,
                sonar_sensor: SonarSensor,
                ) -> None:
        ret, frame = self.cap.read()

        # clone the image if exists, otherwise use the previous image
        if len(frame) != 0:
            img_orig = frame.copy()
            # resize the image for OpenCV processing
        if FRAME_WIDTH != IN_FRAME_WIDTH or FRAME_HEIGHT != IN_FRAME_HEIGHT:
            img_orig = cv2.resize(img_orig, (FRAME_WIDTH,FRAME_HEIGHT))
            if img_orig.shape[1] != FRAME_WIDTH or img_orig.shape[0] != FRAME_HEIGHT:
                sys.exit(-1)
        # convert the image from BGR to grayscale
        img_gray = cv2.cvtColor(img_orig, cv2.COLOR_BGR2GRAY)
        # crop a part of image for binarization
        img_gray_part = img_gray[CROP_U_LIMIT:CROP_D_LIMIT, CROP_L_LIMIT:CROP_R_LIMIT]
        # binarize the image
        img_bin_part = cv2.inRange(img_gray_part, self.gsmin, self.gsmax)
        # prepare an empty matrix
        img_bin = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), np.uint8)
        # copy img_bin_part into img_bin
        img_bin[CROP_U_LIMIT:CROP_U_LIMIT+img_bin_part.shape[0], CROP_L_LIMIT:CROP_L_LIMIT+img_bin_part.shape[1]] = img_bin_part
        # remove noise
        img_bin_mor = cv2.morphologyEx(img_bin, cv2.MORPH_CLOSE, self.kernel)
        # convert the binary image from grayscale to BGR for later
        img_bin_rgb = cv2.cvtColor(img_bin_mor, cv2.COLOR_GRAY2BGR)

        # focus on the region of interest
        x, y, w, h = self.roi
        img_roi = img_bin_mor[y:y+h, x:x+w]
        # find contours in the roi with offset
        contours, hierarchy = cv2.findContours(img_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, offset=(x,y))
        # identify the largest contour
        if len(contours) >= 1:
            i_target = 0
            if len(contours) >= 2:
                cnt_idx = np.empty((0,2), int)
                for i, cnt in enumerate(contours):
                    area = cv2.contourArea(cnt)
                    cnt_idx = np.append(cnt_idx, np.array([[i, int(area)]]), axis=0)
                # sort cnt_idx by area in descending order
                cnt_idx = cnt_idx[np.argsort(cnt_idx[:, 1])[::-1]]
                # calculate the bounding box around the two largest contours,
                # either of which is to be set as the new region of interest 
                x1, y1, w1, h1 = cv2.boundingRect(contours[cnt_idx[0,0]])
                x2, y2, w2, h2 = cv2.boundingRect(contours[cnt_idx[1,0]])
                x, y, w, h = self.roi
                # the second largest contour touches the roi bottom
                # and the second largest contour is large enough?
                if y2 + h2 >= h and 2*cnt_idx[1,1] >= cnt_idx[0,1]:
                    if self.trace_side == TraceSide.LEFT:
                        # look for the left rectangle
                        if x1 <= x2:
                            i_target = cnt_idx[0,0]
                        else:
                            i_target = cnt_idx[1,0]
                    elif self.trace_side == TraceSide.RIGHT:
                        # look for the right rectangle
                        if x1 + w1 >= x2 + w2:
                            i_target = cnt_idx[0,0]
                        else:
                            i_target = cnt_idx[1,0]
                    else: # tracing the line center goes after the largest contour
                        i_target = cnt_idx[0,0]
                else: # the second largest contour does not touch the roi bottom and shall be ignored
                    i_target = cnt_idx[0,0]

            # draw the target contour on the original image
            img_orig = cv2.polylines(img_orig, [contours[i_target]], 0, (0,255,0), LINE_THICKNESS)

            # calculate the bounding box around the largest contour
            # and set it as the new region of interest
            x, y, w, h = cv2.boundingRect(contours[i_target])
            # adjust the region of interest
            x = x - ROI_BOUNDARY
            y = y - ROI_BOUNDARY
            w = w + 2*ROI_BOUNDARY
            h = h + 2*ROI_BOUNDARY
            if x < CROP_L_LIMIT:
                x = CROP_L_LIMIT
            if y < CROP_U_LIMIT:
                y = CROP_U_LIMIT
            if x + w > CROP_R_LIMIT:
                w = CROP_R_LIMIT - x
            if y + h > CROP_D_LIMIT:
                h = CROP_D_LIMIT - y
            # set the new region of interest
            self.roi = (x, y, w, h)
        
            # prepare for trace target calculation
            img_cnt = np.zeros_like(img_orig)
            img_cnt = cv2.drawContours(img_cnt, [contours[i_target]], 0, (0,255,0), 1)
            img_cnt_gray = cv2.cvtColor(img_cnt, cv2.COLOR_BGR2GRAY)
            # scan the line at SCAN_V_POS to find edges
            scan_line = img_cnt_gray[SCAN_V_POS]
            edges = np.flatnonzero(scan_line)
            # calculate the trace target using the edges
            if len(edges) >= 2:
                self.range_of_edges = edges[len(edges)-1] - edges[0]
                if self.trace_side == TraceSide.LEFT:
                    self.cx = edges[0]
                elif self.trace_side == TraceSide.RIGHT:
                    self.cx = edges[len(edges)-1]
                else:
                    self.cx = int((edges[0]+edges[len(edges)-1]) / 2)
            elif len(edges) == 1:
                self.range_of_edges = 1
                self.cx = edges[0]
            self.mx = self.cx
            self.target_insight = True
        else: # len(contours) == 0
            self.range_of_edges = 0
            self.roi = (CROP_L_LIMIT, CROP_U_LIMIT, CROP_WIDTH, CROP_HEIGHT)
            # keep mx in order to maintain the current move of robot
            self.cx = int(FRAME_WIDTH/2)
            self.cy = SCAN_V_POS
            self.target_insight = False
            

        # draw the area of interest on the original image
        x, y, w, h = self.roi
        if self.target_insight:
            cv2.rectangle(img_orig, (x,y), (x+w,y+h), (255,0,0), LINE_THICKNESS)
        else:
            cv2.rectangle(img_orig, (x,y), (x+w,y+h), (0,0,255), LINE_THICKNESS)
        # draw the trace target on the image
        cv2.circle(img_orig, (self.mx, SCAN_V_POS), CIRCLE_RADIUS, (0,0,255), -1)
        # calculate variance of mx from the center in pixel
        vxp = self.mx - int(FRAME_WIDTH/2)
        # convert the variance from pixel to milimeters
        # 138 is length of the closest horizontal line on ground within the camera vision
        vxm = vxp * 138 / FRAME_WIDTH
        # calculate the rotation in radians (z-axis)
        # 205 is distance from axle to the closest horizontal line on ground the camera can see
        self.theta = 180 * math.atan(vxm / 205) / math.pi
        #print(f"mx = {self.mx}, vxm = {vxm}, theta = {self.theta}")

        # prepare text area
        img_text = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), np.uint8)
        if not plotter is None:
            try:
                cv2.putText(img_text, f"ODO={plotter.get_distance():+06}", (0,20), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"x={plotter.get_loc_x():+05} y={plotter.get_loc_y():+05}", (0,40), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"deg={plotter.get_degree():+04} gyro={gyro_sensor.get_angle():+04}", (0,60), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"cx={self.cx:} cy={self.cy} theta={self.theta:+06.1f}", (0,80), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"roe={self.range_of_edges:03}", (0,100), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img_text, f"mV={hub.get_battery_voltage():04} mA={hub.get_battery_current():04}", (0,120), cv2.FONT_HERSHEY_DUPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
            except Exception as e:
                pass
        # concatinate the images - original + text area
        img_comm = cv2.vconcat([img_orig,img_text])
        # shrink the image to avoid delay in transmission
        if OUT_FRAME_WIDTH != FRAME_WIDTH or OUT_FRAME_HEIGHT != FRAME_HEIGHT:
            img_comm = cv2.resize(img_comm, (OUT_FRAME_WIDTH,2*OUT_FRAME_HEIGHT))
        # transmit and display the image
        cv2.imshow("video monitor", img_comm)

        c = cv2.waitKey(1) # show the window

        # ===== 青丸検出（HSV）開始 =====

        # BGR画像をHSV画像に変換（色検出に適した空間）
        hsv = cv2.cvtColor(img_orig, cv2.COLOR_BGR2HSV)

        # 青色のしきい値を設定（環境に応じて調整）
        lower_blue = np.array([95, 80, 90])   # H:色相, S:彩度, V:明度
        upper_blue = np.array([115, 255, 170])

        # 指定範囲の青色だけを抽出して2値マスクを作成
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # マスクにモルフォロジー処理を施し、小さなノイズを除去
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        # === 検出エリアの制限設定 ===
        # 検出対象とする縦の範囲：画像の上端（0）～ 高さの0.67倍まで
        lower_limit = int(FRAME_HEIGHT * 0.67)

        # 上側のみを許可するマスクを作成（検出範囲外の下部を除外）
        upper_mask = np.zeros_like(mask)
        upper_mask[0:lower_limit, :] = 1  # 上のエリアのみ1に設定（マスク適用）

        # 青色マスクと上側マスクをANDして、最終的な検出マスクを生成
        mask_upper = cv2.bitwise_and(mask, mask, mask=upper_mask)

        # 青色領域の輪郭を検出（複数あれば後で面積で最大を選ぶ）
        cnts, _ = cv2.findContours(mask_upper, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 初期化（青が見つからなければ False のまま）
        self.blue_found = False

        self.latest_frame = img_orig.copy()

        if cnts:
            # 面積が最大の輪郭を選択（最も大きな青い物体）
            cnt = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(cnt)

            if area > 80:  # 小さい面積（ノイズ）は無視
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    # 輪郭の重心位置（中心座標）を算出
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])

                    # 青点の情報を保存（クラスのメンバ変数）
                    self.blue_cx = cx
                    self.blue_cy = cy
                    self.blue_area = int(area)

                    # 検出領域内のHSV平均値（デバッグ用）
                    mask_cnt = np.zeros(mask.shape, np.uint8)
                    cv2.drawContours(mask_cnt, [cnt], -1, 255, -1)  # 輪郭内だけをマスク
                    hsv_pixels = hsv[mask_cnt == 255]
                    if len(hsv_pixels) > 0:
                        h_mean = int(np.mean(hsv_pixels[:,0]))
                        s_mean = int(np.mean(hsv_pixels[:,1]))
                        v_mean = int(np.mean(hsv_pixels[:,2]))
                        print(f"[BlueDetect] HSV mean: H={h_mean}, S={s_mean}, V={v_mean}, area={area}")

                    center_x = FRAME_WIDTH // 2
                    center_y = FRAME_HEIGHT // 2

                    # === 青点検出エリアの可視化（常時表示） ===
                    max_cy_ratio = 0.66  # ※ DetectBlueDot ビヘイビアと必ず一致させる！
                    debug_limit_y = int(FRAME_HEIGHT * max_cy_ratio)

                    # 赤い矩形で検出エリア（画像上端～debug_limit_y）を囲う
                    cv2.rectangle(img_orig,
                                (0, 0),  # 左上
                                (FRAME_WIDTH - 1, debug_limit_y),  # 右下
                                (0, 0, 255), 1)  # 赤色, 線の太さ1

                    # エリアの下端に説明ラベルを描画
                    cv2.putText(img_orig,
                                f"Detection Area <= y={debug_limit_y}",
                                (10, debug_limit_y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 0, 255), 1, cv2.LINE_AA)

                    # 青点の重心が検出範囲内にあるか確認
                    if cy < debug_limit_y:
                        self.blue_found = True
                        # デバッグ描画：検出した青点の中心を青丸で表示
                        cv2.circle(img_orig, (cx, cy), 5, (255, 0, 0), -1)
                        print(f"Blue center: cx={cx}, cy={cy}, center_x={center_x}, center_y={center_y}")
                    else:
                        self.blue_found = False

        # ===== 青丸検出 終了 =====
        ...
        cv2.imshow("video monitor", img_comm)
        c = cv2.waitKey(1)
        
    def get_theta(self) -> float:
        return self.theta

    def get_range_of_edges(self) -> int:
        return self.range_of_edges

    def set_thresholds(self, gs_min: int, gs_max: int) -> None:
        self.gs_min = gs_min
        self.gs_max = gs_max

    def set_trace_side(self, trace_side: TraceSide) -> None:
        self.trace_side = trace_side

    def is_target_insight(self) -> bool:
        return self.target_insight

    # --- 追加: 取得用メソッド ---
    def get_blue_info(self):
        return (self.blue_found, self.blue_cx, self.blue_cy, self.blue_area)

    def detect_blue_ellipse(img_bgr: np.ndarray):
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if len(cnt) >= 5:
                ellipse = cv2.fitEllipse(cnt)
                (cx, cy), (major_axis, minor_axis), angle = ellipse
                area = math.pi * (major_axis/2) * (minor_axis/2)
                if area > 80:
                    return True, int(cx), int(cy), int(area)
        return False, 0, 0, 0