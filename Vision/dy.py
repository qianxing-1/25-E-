# K230 / CanMV vision code for TI Cup 2025 E style auto-aiming.
# Detects a rectangular target, optionally detects a red laser spot, and sends
# target/aim data over UART2. Keep this file as ASCII for CanMV compatibility.
# 电赛TI杯2025 E型自瞄K230视觉程序
# 功能：矩形装甲目标识别、可选激光光斑跟踪、UART2向上位机/STM32主控发送瞄准数据
# 编码：纯ASCII文本，兼容CanMV固件解析

import time
import os
import gc
import sys
import image


from media.sensor import *
from media.display import *
from media.media import *
from ybUtils.YbUart import YbUart

uart = YbUart(baudrate=115200)


# 尝试导入硬件加速图像库cv_lite，失败则标记关闭硬件加速
try:
    import cv_lite
    HAS_CV_LITE = True
except BaseException:
    cv_lite = None
    HAS_CV_LITE = False


# =========================
# 用户可调参数区【比赛调试仅修改此处】
# =========================

# 屏幕输出分辨率（LCD显示画面大小）
FRAME_W = 640
FRAME_H = 480
# 算法处理分辨率（灰度识别流，越小帧率越高、算力占用越低）
PROC_W = 320
PROC_H = 240

# 摄像头硬件参数（适配LCKFB K230配套摄像头模组，实测稳定参数）
SENSOR_ID = 2                  # 摄像头硬件通道ID
SENSOR_INPUT_W = 1280          # 传感器原生输出分辨率宽
SENSOR_INPUT_H = 960           # 传感器原生输出分辨率高
SENSOR_FPS = 90                # 摄像头输出帧率
SENSOR_ANALOG_GAIN = 20        # 模拟增益，暗光可适度调高
IDE_JPEG_QUALITY = 50          # IDE在线预览图压缩质量



# 串口数据包模式切换，三种协议适配不同主控程序
# "legacy"：发送滤波后目标坐标格式 $cx,cy# 适配原电机云台程序
# "pid"：直接发送PID计算输出速度 $ux,uy# 适用于速度环接收端
# "aim"：完整多数据帧 $flag,cx,cy,lx,ly,dx,dy,area# 包含激光、误差、面积
PACKET_MODE = "legacy"

# 图像翻转镜像设置，画面倒置/左右颠倒时修改
HMIRROR = False                # 水平镜像开关
VFLIP = False                  # 垂直翻转开关

# 矩形装甲目标过滤参数（纯软件识别降级路径使用）
RECT_THRESHOLD = 6000          # 灰度二值化阈值
RECT_MIN_AREA = 150            # 允许远距离小目标，同时过滤极小噪点
RECT_MAX_AREA = PROC_W * PROC_H * 7 // 10  # 识别矩形最大像素面积，过滤大面积背景框
RECT_MAX_ASPECT = 4.0          # 矩形最大允许长宽比，过滤长条杂物

# cv_lite硬件加速识别配置（高速识别优先走此路径）
USE_CV_LITE = True             # 开启硬件加速矩形检测
CANNY_THRESH1 = 50             # Canny边缘检测低阈值
CANNY_THRESH2 = 150            # Canny边缘检测高阈值
APPROX_EPSILON = 0.04          # 多边形轮廓逼近系数，越小轮廓越精细
AREA_MIN_RATIO = 0.002         # 320x240下约154像素，兼顾远距离识别
MAX_ANGLE_COS = 0.35           # 矩形内角余弦阈值，过滤非直角不规则轮廓
GAUSSIAN_BLUR_SIZE = 3         # 高斯模糊核尺寸，降噪预处理

# 激光光斑跟踪配置（灰度图无法识别红色，仅跟踪高亮白点）
LASER_ENABLE = False           # 激光跟踪总开关，不需要则关闭节省算力
LASER_THRESHOLDS = [
    (220, 255),                # 光斑亮度阈值区间，仅220~255高亮像素判定为光斑
]
LASER_PIXELS_THRESHOLD = 1     # 光斑最小像素点数
LASER_AREA_THRESHOLD = 1       # 光斑最小连通域面积

# 激光坐标平滑系数（仅激光跟踪生效）
# 0=关闭平滑；55=55%上一帧值 + 45%新值，抑制光斑跳动
SMOOTH_PREVIOUS_PERCENT = 55

# 目标中心自适应平滑滤波参数（输出稳定坐标给串口）
# 在320×240处理分辨率内计算平滑，再缩放至640×480屏幕坐标输出
# 小抖动强平滑，真实快速移动时快速跟随
CENTER_FILTER_ENABLE = True    # 自适应一阶平滑总开关
CENTER_NEAR_DISTANCE = 1.5     # 近距离判定阈值（像素），目标小幅晃动
CENTER_FAR_DISTANCE = 8.0      # 远距离判定阈值（像素），目标大幅移动
CENTER_ALPHA_NEAR = 0.45       # 小幅运动提高新坐标权重，降低连续跟踪滞后
CENTER_ALPHA_MIDDLE = 0.72     # 中等运动快速跟随
CENTER_ALPHA_FAR = 0.92        # 大幅运动优先响应当前测量
CENTER_HOLD_DETECTIONS = 0     # 丢失目标立即发送(0,0)，禁止旧坐标触发瞄准

# 匀速卡尔曼跟踪器（过滤矩形中心抖动，丢帧可短时预测位置）
KALMAN_ENABLE = True           # 启用位置速度估计和短时超前补偿
KALMAN_PROCESS_NOISE = 60.0    # 提高对小车速度变化的响应
KALMAN_MEASUREMENT_NOISE = 6.0 # 保留适量测量平滑
KALMAN_INITIAL_UNCERTAINTY = 50.0 # 初始位置不确定度
KALMAN_LEAD_SECONDS = 0.025    # 补偿图像处理、串口和电机响应延迟
TARGET_HOLD_DETECTIONS = 0     # 丢帧保持预测的最大帧数

# 云台PID控制参数（基于画面误差计算输出速度）
# legacy模式下OSD显示PID数值，串口仍输出坐标保证旧程序兼容
PID_ENABLE = True              # PID计算总开关
PID_KP = 0.42                  # 比例系数：消除当前误差，越大响应越快
PID_KI = 0.015                 # 积分系数：消除静态瞄准偏差
PID_KD = 0.08                  # 微分系数：抑制超调、减小云台抖动
PID_D_FILTER = 0.75            # 微分低通滤波系数，抑制图像噪声带来的微分跳变
PID_DEADBAND_X = 5             # X轴死区：误差小于该像素不输出速度，消除微小抖动
PID_DEADBAND_Y = 4             # Y轴死区
PID_INTEGRAL_LIMIT = 180.0     # 积分限幅，防止积分饱和导致云台大幅冲过目标
PID_OUTPUT_LIMIT_X = FRAME_W // 2 # X轴最大输出速度限制
PID_OUTPUT_LIMIT_Y = FRAME_H // 2 # Y轴最大输出速度限制
PID_X_SIGN = 1                 # X轴输出符号翻转，云台转向相反时设为-1
PID_Y_SIGN = 1                 # Y轴输出符号翻转

# 串口发送帧间隔，1=每帧都发，最低延迟自瞄
UART_SEND_INTERVAL = 1

# 目标识别间隔，1=每帧都执行识别，低延迟；大于1隔帧识别减轻算力
DETECT_INTERVAL = 1

# OSD屏幕刷新间隔，LCD叠加文字、矩形、十字标记刷新频率
OSD_REFRESH_INTERVAL = 1

# 串口打印调试信息间隔，0=关闭打印（比赛必关，串口打印拖慢帧率）
DEBUG_PRINT_INTERVAL = 0

# 垃圾回收间隔，每帧gc会严重占用算力，间隔执行平衡内存与卡顿
GC_INTERVAL = 15

# 开机状态日志文件配置，写入SD卡用于离线调试，无需串口打印查看启动状态
STATUS_FILES_ENABLE = True
STATUS_DIR = "/data"           # 日志存储根目录
STATUS_RUNNING_FRAME = 30      # 第30帧写入运行状态日志

# 全局外设句柄预定义
sensor = None                   # 摄像头对象全局句
osd_img = None                  # LCD叠加OSD画布对象
laser_smooth = None             # 激光坐标平滑缓存点


# 数值限幅工具函数，将value强制限制在[low,high]区间
def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value

# 拼接状态日志文件完整路径
def status_path(name):
    return STATUS_DIR + "/ti_vision_" + name + ".txt"

# 写入状态日志到SD卡
def write_status(name, text):
    if not STATUS_FILES_ENABLE:
        return
    status_file = None
    try:
        status_file = open(status_path(name), "w")
        status_file.write(str(text))
    except BaseException:
        # 目录不存在/SD卡故障直接忽略，不阻塞主程序
        pass
    finally:
        if status_file is not None:
            try:
                status_file.close()
            except BaseException:
                pass

# 程序启动前清空旧状态日志
def clear_status_files():
    if not STATUS_FILES_ENABLE:
        return
    for name in ("boot", "camera_ok", "running", "error", "stopped"):
        try:
            os.remove(status_path(name))
        except BaseException:
            # 文件不存在无需报错
            pass

# 计算两次时间戳的间隔秒数，强制限制最小/最大间隔，防止滤波dt异常
def elapsed_seconds(now_ms, previous_ms):
    dt = time.ticks_diff(now_ms, previous_ms) / 1000.0
    return clamp(dt, 0.005, 0.25)

# 一维卡尔曼滤波器类，用于单轴坐标预测滤波
class Kalman1D:
    # 构造函数：过程噪声、测量噪声、初始位置不确定度
    def __init__(self, process_noise, measurement_noise, initial_uncertainty):
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.initial_uncertainty = initial_uncertainty
        self.reset()

    # 重置滤波器所有状态，丢失目标/初始化时调用
    def reset(self):
        self.position = 0.0         # 滤波输出位置
        self.velocity = 0.0         # 估计运动速度
        # 协方差矩阵四元素
        self.p00 = self.initial_uncertainty
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = self.initial_uncertainty
        self.initialized = False    # 是否收到有效测量值标记

    # 卡尔曼更新迭代函数
    # measurement：当前帧检测坐标，None代表无检测目标
    # dt：距离上一帧的时间间隔（秒）
    def update(self, measurement, dt):
        dt = clamp(dt, 0.005, 0.25)
        # 首次收到有效测量，直接赋值初始化
        if not self.initialized:
            if measurement is None:
                return None
            self.position = float(measurement)
            self.velocity = 0.0
            self.initialized = True
            return self.position

        # 预测步：根据速度推算下一时刻位置
        self.position += self.velocity * dt

        # 协方差矩阵预测更新
        dt2 = dt * dt
        p00 = self.p00 + dt * (self.p01 + self.p10) + dt2 * self.p11
        p01 = self.p01 + dt * self.p11
        p10 = self.p10 + dt * self.p11
        p11 = self.p11
        # 叠加过程噪声
        p00 += self.process_noise * dt2
        p11 += self.process_noise * dt

        # 有有效测量值，执行校正步
        if measurement is not None:
            innovation = float(measurement) - self.position # 残差：预测与实际检测差值
            innovation_cov = p00 + self.measurement_noise    # 残差协方差
            k0 = p00 / innovation_cov                       # 卡尔曼增益-位置
            k1 = p10 / innovation_cov                      # 卡尔曼增益-速度

            # 更新位置、速度
            self.position += k0 * innovation
            self.velocity += k1 * innovation
            # 更新后验协方差矩阵
            self.p00 = (1.0 - k0) * p00
            self.p01 = (1.0 - k0) * p01
            self.p10 = p10 - k1 * p00
            self.p11 = p11 - k1 * p01
        # 无测量值，仅保留预测协方差，不校正
        else:
            self.p00 = p00
            self.p01 = p01
            self.p10 = p10
            self.p11 = p11

        return self.position

# 二维点卡尔曼滤波，封装X、Y轴两个一维卡尔曼
class KalmanPoint:
    def __init__(self):
        # X、Y轴独立卡尔曼实例
        self.x = Kalman1D(
            KALMAN_PROCESS_NOISE,
            KALMAN_MEASUREMENT_NOISE,
            KALMAN_INITIAL_UNCERTAINTY,
        )
        self.y = Kalman1D(
            KALMAN_PROCESS_NOISE,
            KALMAN_MEASUREMENT_NOISE,
            KALMAN_INITIAL_UNCERTAINTY,
        )

    # 重置XY双轴滤波状态
    def reset(self):
        self.x.reset()
        self.y.reset()

    # 判断滤波器是否完成初始化（收到过有效坐标）
    def is_ready(self):
        return self.x.initialized and self.y.initialized

    # 二维坐标迭代更新，返回滤波平滑后的点
    def update(self, measurement, dt):
        # 无目标传入None，XY测量值均为空
        if measurement is None:
            mx = None
            my = None
        else:
            mx = measurement[0]
            my = measurement[1]

        # 分别更新XY轴卡尔曼
        x = self.x.update(mx, dt)
        y = self.y.update(my, dt)
        # 任一轴无有效输出，返回空
        if x is None or y is None:
            return None

        # 超前补偿：利用估计速度抵消图像传输延迟
        x += self.x.velocity * KALMAN_LEAD_SECONDS
        y += self.y.velocity * KALMAN_LEAD_SECONDS
        # 限制坐标在处理画面范围内，防止越界
        x = clamp(x, 0, PROC_W - 1)
        y = clamp(y, 0, PROC_H - 1)
        return (int(round(x)), int(round(y)))

# 自适应一阶低通平滑滤波器（无卡尔曼复杂运算，轻量防抖）
class StableCenterFilter:
    def __init__(self):
        self.reset()

    # 清空滤波缓存，丢失目标时重置
    def reset(self):
        self.x = 0.0                # 平滑后X坐标
        self.y = 0.0                # 平滑后Y坐标
        self.initialized = False    # 是否初始化标记
        self.missed = 0             # 连续丢失目标计数

    # 单帧更新平滑坐标，measurement为检测原始点，None代表无目标
    def update(self, measurement):
        # 当前帧无目标，丢失计数+1
        if measurement is None:
            self.missed += 1
            # 未超过保持帧数，返回上一帧平滑坐标
            if self.initialized and self.missed <= CENTER_HOLD_DETECTIONS:
                return (self.x, self.y)
            # 超过保持帧数，清空缓存返回空
            self.reset()
            return None

        # 收到有效目标，重置丢失计数
        measured_x = float(measurement[0])
        measured_y = float(measurement[1])
        self.missed = 0

        # 平滑功能关闭/首次检测，直接赋值不做滤波
        if not CENTER_FILTER_ENABLE or not self.initialized:
            self.x = measured_x
            self.y = measured_y
            self.initialized = True
            return (self.x, self.y)

        # 计算新检测点与平滑缓存点的像素距离
        distance = max(abs(measured_x - self.x), abs(measured_y - self.y))
        # 根据距离切换平滑系数，近慢平滑、远快速跟随
        if distance <= CENTER_NEAR_DISTANCE:
            alpha = CENTER_ALPHA_NEAR
        elif distance <= CENTER_FAR_DISTANCE:
            alpha = CENTER_ALPHA_MIDDLE
        else:
            alpha = CENTER_ALPHA_FAR

        # 一阶平滑公式：输出 = 旧值*(1-alpha) + 新值*alpha
        self.x += (measured_x - self.x) * alpha
        self.y += (measured_y - self.y) * alpha
        return (self.x, self.y)

# 增量式PID控制器类，用于云台速度计算
class PIDController:
    # deadband：误差死区；output_limit：输出速度限幅；sign：输出符号翻转
    def __init__(self, deadband, output_limit, sign=1):
        self.deadband = deadband
        self.output_limit = output_limit
        self.sign = sign
        self.reset()

    # 重置积分、微分缓存，丢失目标时调用防止积分累积饱和
    def reset(self):
        self.integral = 0.0                # 积分累加值
        self.previous_error = None          # 上一帧误差
        self.filtered_derivative = 0.0     # 低通滤波后的微分项

    # PID迭代计算，error为画面像素误差，dt为帧间隔秒数
    def update(self, error, dt):
        dt = clamp(dt, 0.005, 0.25)
        # 误差小于死区，清零误差、衰减积分防止累积
        if abs(error) <= self.deadband:
            error = 0.0
            self.integral *= 0.9

        # 积分项累加，并限制积分上下限防饱和
        self.integral += error * dt
        self.integral = clamp(
            self.integral,
            -PID_INTEGRAL_LIMIT,
            PID_INTEGRAL_LIMIT,
        )

        # 计算微分项，无历史误差则微分置0
        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt
        # 微分低通滤波，抑制噪声尖峰
        self.filtered_derivative = (
            PID_D_FILTER * self.filtered_derivative
            + (1.0 - PID_D_FILTER) * derivative
        )
        # 更新历史误差缓存
        self.previous_error = error

        # 标准PID公式计算输出
        output = (
            PID_KP * error
            + PID_KI * self.integral
            + PID_KD * self.filtered_derivative
        )
        # 输出符号翻转，适配云台转向
        output *= self.sign
        # 输出速度限幅
        output = clamp(output, -self.output_limit, self.output_limit)
        return int(round(output))

# 矩形向外扩充margin像素，用于激光识别ROI裁剪
def expand_rect(rect, margin):
    x, y, w, h = rect
    x0 = clamp(x - margin, 0, PROC_W - 1)
    y0 = clamp(y - margin, 0, PROC_H - 1)
    x1 = clamp(x + w + margin, 0, PROC_W)
    y1 = clamp(y + h + margin, 0, PROC_H)
    return (x0, y0, x1 - x0, y1 - y0)

# 根据目标中心点，将矩形平移至该坐标（丢帧预测矩形位置）
def move_rect_to_center(rect, center):
    if rect is None or center is None:
        return None
    _, _, w, h = rect
    x = clamp(int(center[0] - w // 2), 0, max(0, PROC_W - w))
    y = clamp(int(center[1] - h // 2), 0, max(0, PROC_H - h))
    return (x, y, w, h)

# 将处理分辨率(320×240)的点缩放至屏幕输出分辨率(640×480)
def scale_point_to_frame(point):
    if point is None:
        return None
    x = point[0] * FRAME_W / PROC_W
    y = point[1] * FRAME_H / PROC_H
    return (int(round(x)), int(round(y)))

# 将处理分辨率矩形缩放至屏幕分辨率矩形，用于OSD绘制
def scale_rect_to_frame(rect):
    if rect is None:
        return None
    x, y, w, h = rect
    sx = x * FRAME_W // PROC_W
    sy = y * FRAME_H // PROC_H
    sw = w * FRAME_W // PROC_W
    sh = h * FRAME_H // PROC_H
    return (int(sx), int(sy), int(sw), int(sh))

# 激光坐标平滑加权函数，新旧点加权平均抑制跳动
def smooth_point(old_point, new_point):
    if new_point is None:
        return None
    # 无历史缓存，直接返回新点
    if old_point is None or SMOOTH_PREVIOUS_PERCENT <= 0:
        return new_point
    p = SMOOTH_PREVIOUS_PERCENT
    x = (old_point[0] * p + new_point[0] * (100 - p)) // 100
    y = (old_point[1] * p + new_point[1] * (100 - p)) // 100
    return (int(x), int(y))

# 统一提取矩形元组(x,y,w,h)，兼容cv_lite输出元组与image矩形对象
def rect_tuple(rect_obj):
    try:
        values = rect_obj.rect()
    except BaseException:
        values = rect_obj
    return (int(values[0]), int(values[1]), int(values[2]), int(values[3]))


def rect_corners(rect_obj):
    try:
        values = rect_obj.corners()
        if values is not None and len(values) >= 4:
            return [
                (int(values[0][0]), int(values[0][1])),
                (int(values[1][0]), int(values[1][1])),
                (int(values[2][0]), int(values[2][1])),
                (int(values[3][0]), int(values[3][1])),
            ]
    except BaseException:
        pass

    try:
        if len(rect_obj) >= 12:
            return [
                (int(rect_obj[4]), int(rect_obj[5])),
                (int(rect_obj[6]), int(rect_obj[7])),
                (int(rect_obj[8]), int(rect_obj[9])),
                (int(rect_obj[10]), int(rect_obj[11])),
            ]
    except BaseException:
        pass
    return None

# ============================================================
# 透视补偿模块 — 单应矩阵反算真实靶心
# ============================================================
HOMO_PAPER_W = 210.0
HOMO_PAPER_H = 297.0
HOMO_ENABLE   = True

def _homo_solve(A, b):
    n = len(b)
    for i in range(n):
        max_row = i
        for k in range(i + 1, n):
            if abs(A[k][i]) > abs(A[max_row][i]):
                max_row = k
        A[i], A[max_row] = A[max_row], A[i]
        b[i], b[max_row] = b[max_row], b[i]
        div = A[i][i]
        if abs(div) < 1e-8:
            return None
        for j in range(i, n):
            A[i][j] /= div
        b[i] /= div
        for k in range(n):
            if k != i:
                mul = A[k][i]
                for j in range(i, n):
                    A[k][j] -= mul * A[i][j]
                b[k] -= mul * b[i]
    return b

def _homo_matrix(src, dst):
    A, b = [], []
    for i in range(4):
        x, y = src[i]; u, v = dst[i]
        A.append([x,y,1,0,0,0,-u*x,-u*y])
        b.append(u)
        A.append([0,0,0,x,y,1,-v*x,-v*y])
        b.append(v)
    h = _homo_solve(A, b)
    if h is None: return None
    return [h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7],1.0]

def _homo_invert(H):
    a00,a01,a02=H[0],H[1],H[2];a10,a11,a12=H[3],H[4],H[5];a20,a21,a22=H[6],H[7],H[8]
    b00=a11*a22-a12*a21;b01=a02*a21-a01*a22;b02=a01*a12-a02*a11
    b10=a12*a20-a10*a22;b11=a00*a22-a02*a20;b12=a02*a10-a00*a12
    b20=a10*a21-a11*a20;b21=a01*a20-a00*a21;b22=a00*a11-a01*a10
    det=a00*b00+a01*b10+a02*b20
    if abs(det) < 1e-10: return None
    d=1.0/det
    return [b00*d,b01*d,b02*d,b10*d,b11*d,b12*d,b20*d,b21*d,b22*d]

def _homo_map(H,x,y):
    z=H[6]*x+H[7]*y+H[8]
    if abs(z)<1e-6: return None,None
    return (H[0]*x+H[1]*y+H[2])/z, (H[3]*x+H[4]*y+H[5])/z

def perspective_center(rect_obj):
    corners = rect_corners(rect_obj)
    if corners is None:
        x, y, w, h = rect_tuple(rect_obj)
        return (int(x + w // 2), int(y + h // 2))

    x1, y1 = corners[0]
    x2, y2 = corners[2]
    x3, y3 = corners[1]
    x4, y4 = corners[3]
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    if abs(denominator) < 0.001:
        return (
            int((x1 + x2 + x3 + x4) // 4),
            int((y1 + y2 + y3 + y4) // 4),
        )

    t = float((x1 - x3) * (y3 - y4) -
              (y1 - y3) * (x3 - x4)) / denominator
    return (int(x1 + t * (x2 - x1)), int(y1 + t * (y2 - y1)))

# 计算矩形几何中心点，兼容轮廓角点与普通矩形
def rect_center(rect_obj):
    corners = rect_corners(rect_obj)
    if corners is not None:
        return perspective_center(rect_obj)
    # 普通矩形直接计算中心
    x, y, w, h = rect_tuple(rect_obj)
    return (int(x + w // 2), int(y + h // 2))

# 从全部识别矩形中筛选最优目标：面积大、靠近画面中心得分更高
def choose_target_rect(rects):
    best = None
    best_score = -99999999
    frame_cx = PROC_W // 2
    frame_cy = PROC_H // 2

    for rect_obj in rects:
        x, y, w, h = rect_tuple(rect_obj)
        if w <= 0 or h <= 0:
            continue

        area = w * h
        # 过滤面积超出阈值的矩形
        if area < RECT_MIN_AREA or area > RECT_MAX_AREA:
            continue

        short_side = min(w, h)
        long_side = max(w, h)
        if short_side <= 0:
            continue
        # 过滤长宽比超标长条杂物
        aspect = long_side / short_side
        if aspect > RECT_MAX_ASPECT:
            continue

        # 计算得分：面积越大分越高，离画面中心越远扣分越多
        cx, cy = rect_center(rect_obj)
        center_penalty = abs(cx - frame_cx) + abs(cy - frame_cy)
        score = area - center_penalty // 4
        # 扫描时目标通常先从画面边缘进入，中心偏置不能压过面积得分。
        score = area - center_penalty // 4

        if score > best_score:
            best = rect_obj
            best_score = score

    return best

# 目标矩形识别主函数，优先cv_lite硬件加速，失败降级软件find_rects
def find_target_rect(img):
    global HAS_CV_LITE

    if USE_CV_LITE and HAS_CV_LITE:
        try:
            # 调用硬件加速灰度矩形检测接口
            raw = cv_lite.rgb888_find_rectangles_with_corners(
                [PROC_H, PROC_W],
                img.to_numpy_ref(),
                CANNY_THRESH1,
                CANNY_THRESH2,
                APPROX_EPSILON,
                AREA_MIN_RATIO,
                MAX_ANGLE_COS,
                GAUSSIAN_BLUR_SIZE,
            )

            # 筛选最优目标返回
            target = choose_target_rect(raw)
            if target is not None:
                return target
            return None
        except BaseException as error:
            # 硬件加速异常，关闭标记切换软件路径
            HAS_CV_LITE = False
            print("cv_lite fallback:", error)

    # 软件降级识别路径
    target = choose_target_rect(img.find_rects(threshold=RECT_THRESHOLD))
    if target is None:
        return None
    return target

# 从光斑列表中筛选最优激光点，优先高亮、靠近装甲目标
def choose_laser_blob(blobs, target_point=None):
    best = None
    best_score = -99999999
    for blob in blobs:
        area = blob.w() * blob.h()
        pixels = blob.pixels()
        # 基础得分：像素亮度点数权重更高
        score = pixels * 12 - area
        # 有装甲目标时，距离装甲越近加分
        if target_point is not None:
            score -= (abs(blob.cx() - target_point[0]) + abs(blob.cy() - target_point[1])) // 4
        if score > best_score:
            best = blob
            best_score = score
    return best

# 激光光斑识别函数，仅在装甲目标ROI内搜索减少算力
def find_laser(img, target_rect=None, target_point=None):
    if not LASER_ENABLE:
        return None

    roi = None
    # 有装甲目标，仅在装甲扩大区域内搜索激光，缩小识别范围
    if target_rect is not None:
        roi = expand_rect(target_rect, 20)

    try:
        # 无ROI全图搜索，有ROI限定区域搜索光斑
        if roi is None:
            blobs = img.find_blobs(
                LASER_THRESHOLDS,
                pixels_threshold=LASER_PIXELS_THRESHOLD,
                area_threshold=LASER_AREA_THRESHOLD,
                merge=True,
                margin=2,
            )
        else:
            blobs = img.find_blobs(
                LASER_THRESHOLDS,
                LASER_THRESHOLDS,
                roi=roi,
                pixels_threshold=LASER_PIXELS_THRESHOLD,
                area_threshold=LASER_AREA_THRESHOLD,
                merge=True,
                margin=2,
            )
    except BaseException:
        return None

    blob = choose_laser_blob(blobs, target_point)
    if blob is None:
        return None
    # 返回光斑坐标+blob原始对象
    return (int(blob.cx()), int(blob.cy()), blob)

# OSD图层绘制函数：屏幕叠加十字、矩形、文字信息
def draw_osd(
    canvas,
    target_rect_out,
    target_out,
    laser_out,
    dx,
    dy,
    pid_x,
    pid_y,
    fps_value,
):
    # 清空上一帧OSD画布
    canvas.clear()

    # 颜色定义
    red = (255, 0, 0)
    yellow = (255, 255, 0)
    white = (255, 255, 255)
    black = (0, 0, 0)

    # 画面中心瞄准十字（黄色固定基准）
    canvas.draw_cross(FRAME_W // 2, FRAME_H // 2, color=yellow, size=16, thickness=2)

    # 绘制识别到的装甲矩形+目标中心红色十字
    if target_rect_out is not None:
        x, y, w, h = target_rect_out
        canvas.draw_rectangle(x, y, w, h, color=red, thickness=5)
        canvas.draw_cross(target_out[0], target_out[1], color=red, size=20, thickness=4)

    # 绘制激光光斑黄色十字
    if laser_out is not None:
        canvas.draw_cross(laser_out[0], laser_out[1], color=yellow, size=14, thickness=2)

    # 左上角黑色半透明背景，显示调试文字
    canvas.draw_rectangle(0, 0, 390, 102, color=black, fill=True)
    canvas.draw_string_advanced(10, 3, 30, "FPS:{:.1f}".format(fps_value), color=white)
    # 打印目标坐标与画面误差
    if target_out is not None:
        canvas.draw_string_advanced(
            10,
            39,
            24,
            "T:{},{}  d:{},{}".format(target_out[0], target_out[1], dx, dy),
            color=red,
        )
    else:
        canvas.draw_string_advanced(10, 39, 24, "NO TARGET", color=red)
    # 打印PID输出速度
    canvas.draw_string_advanced(
        10,
        69,
        22,
        "PID:{},{}".format(pid_x, pid_y),
        color=yellow,
    )

# 串口数据包打包发送函数，根据PACKET_MODE输出不同ASCII字符串帧
# 串口数据包打包发送函数，输出二进制7字节帧：0x2C 0x12 X高 X低 Y高 Y低 0x5B
def send_packet(target_point, laser_point, dx, dy, area, pid_x, pid_y):
    global uart
    if uart is None:
        return



    # 无目标固定发送 X=0,Y=0，通知STM32立即恢复搜索。
    if target_point is None:
        x_val = 0
        y_val = 0
    else:
        cx, cy = target_point
        x_val = int(cx)
        y_val = int(cy)

    # 坐标限幅 0~FRAME分辨率，防止数值溢出uint16
    x_clamp = max(0, min(FRAME_W, x_val))
    y_clamp = max(0, min(FRAME_H, y_val))
    # 拆分高低字节（大端，和STM32接收解析一致）
    x_high = (x_clamp >> 8) & 0xFF
    x_low  = x_clamp & 0xFF
    y_high = (y_clamp >> 8) & 0xFF
    y_low  = y_clamp & 0xFF
    # 固定7字节帧头+数据+帧尾
    frame = bytes([0x2C, 0x12, x_high, x_low, y_high, y_low, 0x5B])
    uart.send(frame)




# 摄像头、显示、媒体硬件初始化
def camera_init():
    global sensor, uart, osd_img

    # 开启系统退出捕获，异常可正常释放硬件
    os.exitpoint(os.EXITPOINT_ENABLE)


    # 创建摄像头实例，设置原生分辨率与帧率
    sensor = Sensor(
        id=SENSOR_ID,
        width=SENSOR_INPUT_W,
        height=SENSOR_INPUT_H,
        fps=SENSOR_FPS,
    )
    sensor.reset()
    # 尝试设置画面镜像翻转，部分模组不支持则跳过
    try:
        sensor.set_hmirror(HMIRROR)
        sensor.set_vflip(VFLIP)
    except BaseException:
        pass

    # 通道0：彩色640×480画面，硬件直通LCD显示，不占用CPU
    sensor.set_framesize(width=FRAME_W, height=FRAME_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
    bind_info = sensor.bind_info()
    Display.bind_layer(**bind_info, layer=Display.LAYER_VIDEO1)

    # 通道1：灰度320×240画面，专门用于目标识别算法
    sensor.set_framesize(width=PROC_W, height=PROC_H, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB888, chn=CAM_CHN_ID_1)

    # 显示屏初始化ST7701 LCD，开启OSD图层
    Display.init(
        Display.ST7701,
        width=FRAME_W,
        height=FRAME_H,
        to_ide=True,
        quality=IDE_JPEG_QUALITY,
        osd_num=2,
    )
    # 创建OSD叠加画布ARGB8888透明图层
    osd_img = image.Image(FRAME_W, FRAME_H, image.ARGB8888)
    # 媒体管理器初始化
    MediaManager.init()
    # 启动摄像头数据流
    sensor.run()

    # 设置摄像头模拟增益，无该接口模组直接跳过
    try:
        gain = k_sensor_gain()
        gain.gain[0] = SENSOR_ANALOG_GAIN
        sensor.again(gain)
    except BaseException as error:
        print("sensor gain skipped:", error)

# 硬件资源反初始化：摄像头、显示屏、媒体库释放
def camera_deinit():
    global sensor
    try:
        if isinstance(sensor, Sensor):
            sensor.stop()
    except BaseException as error:
        print("sensor stop error:", error)

    try:
        Display.deinit()
    except BaseException as error:
        print("display deinit error:", error)

    # 休眠退出点
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)

    try:
        MediaManager.deinit()
    except BaseException as error:
        print("media deinit error:", error)

# 程序主循环入口
def main():
    global laser_smooth

    camera_is_init = False          # 摄像头初始化完成标记
    frame_id = 0                    # 全局帧计数
    last_target_rect = None         # 上一帧识别到的装甲矩形缓存
    target_point = None             # 当前平滑后目标中心点
    missed_detections = 0           # 连续丢失目标计数
    center_filter = StableCenterFilter() # 轻量自适应平滑滤波器实例
    # 根据开关选择卡尔曼或仅使用轻量平滑
    target_filter = KalmanPoint() if KALMAN_ENABLE else None
    # X/Y轴独立PID控制器实例
    pid_x_controller = PIDController(PID_DEADBAND_X, PID_OUTPUT_LIMIT_X, PID_X_SIGN)
    pid_y_controller = PIDController(PID_DEADBAND_Y, PID_OUTPUT_LIMIT_Y, PID_Y_SIGN)
    last_detect_ms = time.ticks_ms()# 上一次识别的时间戳（卡尔曼dt计算）
    last_pid_ms = last_detect_ms    # 上一次PID计算时间戳
    clock = time.clock()            # 帧率计时时钟

    try:
        # 清空旧日志，写入开机启动标记
        clear_status_files()
        write_status("boot", "main entered")
        # 初始化全部硬件
        camera_init()
        camera_is_init = True
        write_status("camera_ok", "camera initialized")
        print("ti_cup_e_vision_k230 start")

        # 无限主循环
        while True:
            # 捕获IDE停止、系统异常退出信号
            os.exitpoint()
            clock.tick()
            frame_id += 1

            # 抓取通道1灰度识别帧
            img = sensor.snapshot(chn=CAM_CHN_ID_1)
            # 判断本帧是否需要执行目标识别
            run_detection = frame_id == 1 or frame_id % DETECT_INTERVAL == 0
            if run_detection:
                detect_dt = 0.0
                # 卡尔曼开启时计算帧间隔dt
                if KALMAN_ENABLE:
                    now_detect_ms = time.ticks_ms()
                    detect_dt = elapsed_seconds(now_detect_ms, last_detect_ms)
                    last_detect_ms = now_detect_ms
                # 执行矩形装甲识别
                detected_target = find_target_rect(img)

                # 识别到有效装甲
                if detected_target is not None:
                    measured_point = perspective_center(detected_target)
                    detected_rect = rect_tuple(detected_target)
                    # 卡尔曼滤波平滑原始坐标
                    if KALMAN_ENABLE:
                        target_point = target_filter.update(measured_point, detect_dt)
                    else:
                        target_point = center_filter.update(measured_point)
                    # 将矩形跟随平滑中心点平移，用于OSD绘制
                    last_target_rect = move_rect_to_center(detected_rect, target_point)
                    missed_detections = 0
                # 本帧无识别目标
                else:
                    missed_detections += 1
                    # 传入None更新平滑滤波器（丢帧保持输出）
                    target_point = center_filter.update(None)
                    # 仍有缓存矩形，平移至上一帧平滑坐标
                    if target_point is not None and last_target_rect is not None:
                        last_target_rect = move_rect_to_center(last_target_rect, target_point)
                    # 连续丢帧超过阈值，清空全部缓存、重置卡尔曼
                    else:
                        target_point = None
                        last_target_rect = None
                        if target_filter is not None:
                            target_filter.reset()
            # 使用缓存的矩形作为当前帧绘制矩形
            target_rect = last_target_rect

            target_out = None
            area = 0
            target_rect_out = None
            # 存在有效目标，缩放坐标至屏幕分辨率
            if target_rect is not None:
                # 计算目标屏幕像素面积
                area = target_rect[2] * target_rect[3] * FRAME_W * FRAME_H // (PROC_W * PROC_H)
                target_out = scale_point_to_frame(target_point)
                target_rect_out = scale_rect_to_frame(target_rect)

            # 激光光斑识别
            laser_info = find_laser(img, target_rect, target_point)
            laser_point = None
            laser_out = None
            if laser_info is not None:
                # 平滑激光坐标缓存
                laser_point = smooth_point(laser_smooth, (laser_info[0], laser_info[1]))
                laser_smooth = laser_point
                laser_info = (laser_point[0], laser_point[1], laser_info[2])
                laser_out = scale_point_to_frame(laser_point)
            else:
                laser_smooth = None

            # 计算画面误差dx dy：目标坐标 - 画面中心坐标
            dx = 0
            dy = 0
            if target_out is not None:
                # 有激光光斑则误差为激光相对装甲的偏移，无激光则为目标相对屏幕中心偏移
                if laser_out is not None:
                    dx = target_out[0] - laser_out[0]
                    dy = target_out[1] - laser_out[1]
                else:
                    dx = target_out[0] - FRAME_W // 2
                    dy = target_out[1] - FRAME_H // 2

            # PID计算帧间隔dt
            now_pid_ms = time.ticks_ms()
            pid_dt = elapsed_seconds(now_pid_ms, last_pid_ms)
            last_pid_ms = now_pid_ms
            pid_x = 0
            pid_y = 0
            # 存在目标执行PID运算，无目标重置积分缓存
            if target_out is not None and PID_ENABLE:
                pid_x = pid_x_controller.update(dx, pid_dt)
                pid_y = pid_y_controller.update(dy, pid_dt)
            elif target_out is None:
                pid_x_controller.reset()
                pid_y_controller.reset()

            # 按发送间隔执行串口发包
            if frame_id % UART_SEND_INTERVAL == 0:
                send_packet(target_out, laser_out, dx, dy, area, pid_x, pid_y)

            # 调试打印帧信息
            if DEBUG_PRINT_INTERVAL and frame_id % DEBUG_PRINT_INTERVAL == 0:
                if target_out is not None:
                    print(
                        "target:", target_out,
                        "laser:", laser_out,
                        "dxdy:", dx, dy,
                        "pid:", pid_x, pid_y,
                        "fps:", clock.fps(),
                    )
                else:
                    print("target: none", "fps:", clock.fps())

            # 刷新OSD屏幕叠加图层
            if OSD_REFRESH_INTERVAL and frame_id % OSD_REFRESH_INTERVAL == 0:
                draw_osd(
                    osd_img,
                    target_rect_out,
                    target_out,
                    laser_out,
                    dx,
                    dy,
                    pid_x,
                    pid_y,
                    clock.fps(),
                )
                Display.show_image(osd_img, layer=Display.LAYER_OSD1)

            # 第30帧写入运行状态日志
            if frame_id == STATUS_RUNNING_FRAME:
                write_status(
                    "running",
                    "frame={} fps={:.1f}".format(frame_id, clock.fps()),
                )
            # 释放灰度帧内存
            img = None
            # 定时垃圾回收释放碎片内存
            if GC_INTERVAL and frame_id % GC_INTERVAL == 0:
                gc.collect()

    # IDE手动停止程序捕获
    except KeyboardInterrupt as error:
        write_status("stopped", repr(error))
        print("user stop:", error)
    # 全局异常捕获，记录故障日志不直接死机
    except BaseException as error:
        error_text = repr(error)
        if "IDE interrupt" in error_text:
            write_status("stopped", error_text)
            print("ide stop:", error_text)
        else:
            write_status("error", error_text)
            print("exception:", error_text)
            raise
    # 无论是否异常，退出前释放全部硬件
    finally:
        if camera_is_init:
            camera_deinit()


# 程序入口执行
if __name__ == "__main__":
    main()
