import logging
import re
import struct
import json
from collections import deque
from mqtt_client import MQTTSubscribeClient, MQTTPublishClient
import time
import threading
import serial
import numpy as np

import config
from coordinate_transform import camera_to_enu, gps_to_enu
from rtk_reader import RTKReader
from fusion import PositionFusion

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s.%(msecs)03d - %(levelname)s - %(name)s - %(lineno)d - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

# 写入优先级: 手动控制 > 前端控制 > 路径规划控制 > 坐标
# 数据写入进程锁
write_thread_lock = threading.Lock()
# 前端控制进程锁
web_thread_lock = threading.Lock()
# 手动控制进程锁
manual_thread_lock = threading.Lock()
# 路径执行结束标志：1表示执行完成，可以发送下一个路径点；0表示正在执行路径点
route_finish_flag = 1

# 小车坐标 (原始摄像头数据，保留用于兼容)
location_deque = deque(maxlen=1)
# 摄像头 ENU 坐标 (供 fusion_loop 消费)
camera_enu_deque = deque(maxlen=1)
# RTK 数据 (由 RTKReader 填充)
rtk_deque = deque(maxlen=1)
# 融合后位置 (ENU)
fused_position_deque = deque(maxlen=1)
# 路径控制数据
route_deque = deque(maxlen=100)
# 前端控制数据
web_deque = deque(maxlen=1)
# 方向盘控制数据
manual_deque = deque(maxlen=1)
# 融合滤波器实例
fusion_filter = PositionFusion()
# 摄像头最后更新时间
camera_last_time = 0.0
# 卡尔曼/融合状态上报
kalman_deque = deque(maxlen=1)
kalman_deque.extend([{
    "Q": 0,
    "R": 0,
    "P": 0,
    "I": 0,
    "D": 0,
    "really_angle": 0,
    "kalman_location_x": 0,
    "kalman_location_y": 0
}])
# XT32数据
XT32_DATA = ""

# 控制模式切换时间
switch_time = config.SWITCH_TIME
location_time = time.time() - switch_time
route_time = time.time() - switch_time
web_time = time.time()
manual_time = time.time()


def str_to_json(input_string):
    # 创建空字典
    params_dict = {}

    # 切割字符串并生成键值对
    split_pairs = input_string.split(";")

    split_pairs = split_pairs[:-1]
    for pair in split_pairs:
        key, value = pair.split(":")
        params_dict[key] = float(value)

    return params_dict


def location_message_callback(client, userdata, msg):
    """坐标通信回调(MQTT)

    接收摄像头数据，转换为 ENU 坐标推入 camera_enu_deque 供融合使用。

    :param client: MQTT客户端实例
    :param userdata: 用户自定义数据
    :param msg: 接收到的消息
    """
    global location_deque, camera_enu_deque, camera_last_time

    message = eval(str(msg.payload, encoding="utf-8"))
    if message["x"] > 0 and message["y"] > 0:
        logging.debug(f"received location message:{str(msg.payload, encoding='utf-8')}")
        location_deque.append(message)

        # 转换为 ENU 坐标
        e, n = camera_to_enu(
            message["x"], message["y"],
            config.CAMERA_ORIGIN_ENU_E,
            config.CAMERA_ORIGIN_ENU_N,
            config.CAMERA_TO_ENU_ROTATION,
        )
        camera_enu_deque.append({"e": e, "n": n})
        camera_last_time = time.time()
    else:
        logging.debug("skip error location")


def control_message_callback(client, userdata, msg):
    """控制通信回调

    手动控制和路径控制
    最大油门/刹车是80，最小油门/刹车是0

    :param client: MQTT客户端实例
    :param userdata: 用户自定义数据
    :param msg: 接收到的消息
    :return:
    """
    global web_thread_lock, manual_thread_lock
    global route_finish_flag
    global route_time, manual_time, web_time
    global route_deque, web_deque, manual_deque

    message = eval(str(msg.payload, encoding="utf-8"))
    message_data = message["data"]

    if msg.topic == "track/control/route":
        route_time = time.time()
        print("recv route message: ", message_data)
        route_deque.clear()
        route_finish_flag = 1
        # 统一转换为 ENU 坐标存储
        for point in message_data:
            coord_type = point.get("coord_type", "")
            if coord_type == "gps":
                e, n, _ = gps_to_enu(
                    point["lat"], point["lon"], 0.0,
                    config.ENU_ORIGIN_LAT, config.ENU_ORIGIN_LON, config.ENU_ORIGIN_ALT,
                )
                route_deque.append({"e": e, "n": n})
            elif coord_type == "enu":
                route_deque.append({"e": point["e"], "n": point["n"]})
            else:
                # 兼容旧格式 {"x", "y"} — 摄像头厘米坐标
                e, n = camera_to_enu(
                    point["x"], point["y"],
                    config.CAMERA_ORIGIN_ENU_E,
                    config.CAMERA_ORIGIN_ENU_N,
                    config.CAMERA_TO_ENU_ROTATION,
                )
                route_deque.append({"e": e, "n": n})
    elif msg.topic == "track/control/web":
        car_number = message_data[0]["car_number"]
        if not car_number == config.CAR_NUMBER:
            print("not command this car")
            return
        web_time = time.time()
        logging.info(f"receive web message: {message_data}")
        if not web_thread_lock.locked():
            logging.info("switch to web control mode")
            web_thread_lock.acquire()
            route_deque.clear()
            route_finish_flag = 1
        angle = message_data[0]["angle"]
        accelerator = message_data[0]["accelerator"]
        brake = message_data[0]["brake"]
        web_control_data = {"angle": angle, "accelerator": accelerator, "brake": brake}
        web_deque.clear()
        web_deque.append(web_control_data)
    elif msg.topic == "track/control/manual":
        manual_time = time.time()
        print("recv manual message: ", message_data)
        if not manual_thread_lock.locked():
            print("switch to manual control mode")
            manual_thread_lock.acquire()
            route_deque.clear()
            route_finish_flag = 1

        angle = message_data[0]["angle"]
        if angle >= 0:
            angle = (angle / 32767) * 180 * 1.25
        else:
            angle = (angle / 32768) * 180 * 1.25

        accelerator = message_data[0]["accelerator"]
        accelerator = 80 * (1 - (accelerator + 32768) / 65535)

        brake = message_data[0]["brake"]
        brake = 80 * (1 - (brake + 32768) / 65535)
        manual_data = {"angle": angle, "accelerator": accelerator, "brake": brake}
        # TODO 改成append了，因为队列长度是一
        # manual_deque.extend(manual_data)
        manual_deque.append(manual_data)


def fusion_loop():
    """核心融合循环 (20Hz)

    单线程消费所有传感器数据，避免线程安全问题。
    """
    global fusion_filter, rtk_deque, camera_enu_deque, fused_position_deque

    while True:
        fusion_filter.predict()

        # 消费 RTK 数据
        if rtk_deque:
            rtk = rtk_deque[0]
            e, n, _ = gps_to_enu(
                rtk["lat"], rtk["lon"], rtk["alt"],
                config.ENU_ORIGIN_LAT, config.ENU_ORIGIN_LON, config.ENU_ORIGIN_ALT,
            )
            fusion_filter.update_rtk(e, n, rtk["fix_quality"])

        # 消费摄像头数据 (超时则不再使用，自动退化为 RTK-only)
        if camera_enu_deque and (time.time() - camera_last_time) < config.CAMERA_TIMEOUT:
            cam = camera_enu_deque[0]
            fusion_filter.update_camera(cam["e"], cam["n"])

        # 输出融合位置
        e, n = fusion_filter.get_position()
        fused_position_deque.append({"e": e, "n": n})

        time.sleep(0.05)  # 20Hz


def mqtt_client_start():
    """启动MQTT客户端连接.

    FIXME 没有配置账户密码

    """
    global location_subscribe, control_subscribe
    global kalman_publish, route_status_publish

    # ======== 订阅 ========
    # 小车位置
    location_subscribe = MQTTSubscribeClient(
        broker=config.BROKER,
        port=config.PORT,
        topic=[("track/data/location", 0)]
    )

    # 控制指令(路径/手动模式)·
    control_subscribe = MQTTSubscribeClient(
        broker=config.BROKER,
        port=config.PORT,
        topic=[
            ("track/control/route", 0),
            ("track/control/web", 0),
            ("track/control/manual", 0)
        ]
    )

    # ======== 推送 ========
    # 卡尔曼滤波数据
    kalman_publish = MQTTPublishClient(
        broker=config.BROKER,
        port=config.PORT,
        topic=[("track/data/kalman", 0)]
    )

    # 路径完成状态
    route_status_publish = MQTTPublishClient(
        broker=config.BROKER,
        port=config.PORT,
        topic=[("track/control/route_status", 0)]
    )

    # ======== 回调 ========
    # 位置信息
    location_subscribe.client.on_message = location_message_callback
    # 控制指令
    control_subscribe.client.on_message = control_message_callback

    # 启动所有MQTT客户端线程
    location_subscribe.start()
    control_subscribe.start()
    kalman_publish.start()
    route_status_publish.start()


# 获取小车坐标数据并封包 (使用融合后 ENU 坐标)
def get_car_location_package(fused_deque):
    e = fused_deque[0]["e"]
    n = fused_deque[0]["n"]

    if config.STM32_COORD_MODE == "dm":
        # 分米模式：ENU 米 × 10 → 有符号 16 位
        loc_x = int(round(e * 10))
        loc_y = int(round(n * 10))
        loc_x = max(-32768, min(32767, loc_x))
        loc_y = max(-32768, min(32767, loc_y))
        location_data_package = struct.pack(
            ">BBBhhBB",
            0xAA, 0xFD, 0x0A,
            loc_x, loc_y,
            0x00,  # angle placeholder
            0xBB,
        )
    else:
        # cm10 模式 (兼容旧固件)：ENU 米 × 1000 → 无符号 16 位
        loc_x = int(round(e * 1000))
        loc_y = int(round(n * 1000))
        loc_x = max(0, min(65535, loc_x))
        loc_y = max(0, min(65535, loc_y))
        location_data_package = struct.pack(
            "10B",
            0xAA, 0xFD, 0x0A,
            (loc_x >> 8), (loc_x & 0xFF),
            (loc_y >> 8), (loc_y & 0xFF),
            0x00, 0x00,
            0xBB,
        )

    return location_data_package


def get_car_route_package(route_deque):
    """路径控制数据封包 (使用融合后 ENU 坐标)"""
    global fused_position_deque

    # 起点：融合后位置
    fused = fused_position_deque[0]
    start_e = fused["e"]
    start_n = fused["n"]
    # 终点：路由点 (已统一转换为 ENU)
    target = route_deque[0]
    target_e = target["e"]
    target_n = target["n"]

    if config.STM32_COORD_MODE == "dm":
        # 分米模式：ENU 米 × 10 → 有符号 16 位
        s_x = max(-32768, min(32767, int(round(start_e * 10))))
        s_y = max(-32768, min(32767, int(round(start_n * 10))))
        t_x = max(-32768, min(32767, int(round(target_e * 10))))
        t_y = max(-32768, min(32767, int(round(target_n * 10))))
        route_data_package = struct.pack(
            ">BBBBhhhhB",
            0xAA, 0xFE, 0x0D, 0x02,
            s_x, s_y, t_x, t_y,
            0xBB,
        )
    else:
        # cm10 模式 (兼容旧固件)
        s_x = max(0, min(65535, int(round(start_e * 1000))))
        s_y = max(0, min(65535, int(round(start_n * 1000))))
        t_x = max(0, min(65535, int(round(target_e * 1000))))
        t_y = max(0, min(65535, int(round(target_n * 1000))))
        route_data_package = struct.pack(
            "13B",
            0xAA, 0xFE, 0x0D, 0x02,
            (s_x >> 8), (s_x & 0xFF),
            (s_y >> 8), (s_y & 0xFF),
            (t_x >> 8), (t_x & 0xFF),
            (t_y >> 8), (t_y & 0xFF),
            0xBB,
        )

    print("location e & n:", start_e, start_n)
    print("route e & n:", target_e, target_n)
    print("route package:", route_data_package.hex())

    return route_data_package


def get_car_web_package(web_deque):
    """获取前端控制数据

    :param web_deque:
    :return:
    """
    angle = web_deque[0]['angle']
    accelerator = web_deque[0]['accelerator']
    brake = web_deque[0]['brake']

    if angle < 0:
        turn_flag = 0x00
    else:
        turn_flag = 0x01

    angle = abs(angle)

    web_data_package = struct.pack(
        '8B',
        0xAA,
        0xF0,
        0x08,
        turn_flag,
        int(angle),
        int(accelerator),
        int(brake),
        0xBB
    )
    logging.info(f"web package: {web_data_package.hex()}")

    return web_data_package


def get_car_manual_package(manual_deque):
    """方向盘控制数据封包

    """
    angle = manual_deque[0]['angle']
    if angle < 0:
        turn_flag = 0x00
    else:
        turn_flag = 0x01
    angle = abs(angle)
    accelerator = manual_deque[0]['accelerator'] / 10
    brake = manual_deque[0]['brake'] / 10
    manual_data_package = struct.pack(
        '8B',
        0xAA,
        0xF0,
        0x08,
        turn_flag,
        int(angle),
        int(accelerator),
        int(brake),
        0xBB
    )
    print("manual package: ", manual_data_package.hex())

    return manual_data_package


# 卡尔曼数据上传
def kalman_message_publish(topic=None, index=0):
    global kalman_publish
    global kalman_deque

    if topic is None:
        topic = kalman_publish.topic[index]
    else:
        topic = topic[index]
    while True:
        message_data = kalman_deque[0]
        message = json.dumps({"data": [message_data]})
        message = f"{message}"
        kalman_publish.client.publish(topic[0], message, qos=topic[1])
        # print("publish kalman package: ", message)
        time.sleep(0.5)


# Done数据上传
def route_done_publish():
    global route_status_publish

    message_data = {"route_done": 1}
    message = json.dumps(message_data)
    message = str(message)
    route_status_publish_data = route_status_publish.topic[0]
    topic = route_status_publish_data[0]
    qos = route_status_publish_data[1]
    route_status_publish.client.publish(topic, message, qos=qos)
    logging.info(f"publish route done: {message}")


# 读取串口回传数据
def on_serial_data_received(ser):
    global XT32_DATA
    global kalman_deque
    global route_finish_flag

    # pattern_1 = re.compile(r"\n", re.I)
    while True:
        count = ser.inWaiting()
        if count > 0:
            data = ser.read(count).decode("utf-8")
            logging.info(f"receive XT 32 : {data}")
            # XT32_DATA += data
            # data_list = re.split(pattern_1, XT32_DATA)
            data_list = data.split("\n")
            # logging.debug(len(data_list))
            if len(data_list) == 1:
                for data in data_list:
                    if "RD" in data:
                        time.sleep(0.5)
                        route_finish_flag = 1
                        logging.warning(f"route done")
                        route_done_publish()


# 串口写入位置包 (使用融合后位置)
def write_actual_location_data(ser):
    global write_thread_lock
    global fused_position_deque

    while True:
        if not write_thread_lock.locked() and len(fused_position_deque) >= 1:
            package = get_car_location_package(fused_position_deque)
            ser.write(package)
            time.sleep(0.5)


# 串口写入控制包
def serial_write_control(ser):
    global write_thread_lock
    global web_thread_lock
    global manual_thread_lock
    global route_finish_flag
    global route_deque
    global manual_deque
    global web_deque

    while True:
        if manual_thread_lock.locked() and len(manual_deque) >= 1:
            package = get_car_manual_package(manual_deque)
            manual_deque.popleft()
            write_thread_lock.acquire()
            ser.write(package)
            logging.info(f"manual package write success: {manual_deque}")
            write_thread_lock.release()
            # time.sleep(0.1)
        elif web_thread_lock.locked() and len(web_deque) >= 1:
            package = get_car_web_package(web_deque)
            web_deque.popleft()
            write_thread_lock.acquire()
            ser.write(package)
            logging.info(f"web package write success, {web_deque}")
            write_thread_lock.release()
        elif not manual_thread_lock.locked() and not web_thread_lock.locked() and len(route_deque) >= 1 and route_finish_flag:
            logging.debug(f"route deque: {route_deque}, {len(route_deque)}, \n")
            if len(route_deque[0]) == 0:
                logging.warning("route deque null")
                route_deque.popleft()
                continue
            package = get_car_route_package(route_deque)
            route_deque.popleft()
            write_thread_lock.acquire()
            ser.write(package)
            logging.info(f"route package write success, {route_deque}")
            route_finish_flag = 0
            write_thread_lock.release()
            # time.sleep(0.1)


# 控制模式转换
def control_mode_switch():
    global route_finish_flag
    global manual_thread_lock
    global web_thread_lock, web_time
    global switch_time
    global manual_time

    while True:
        if time.time() - manual_time >= switch_time and manual_thread_lock.locked():
            manual_thread_lock.release()

            route_deque.clear()
            route_finish_flag = 1
            package = struct.pack(
                "8B",
                int("AA", 16),
                int("F0", 16),
                int("08", 16),
                int("02", 16),
                int(0),
                int(0),
                int(0),
                int("BB", 16)
            )

            # ser.write(package)
            print("switch to route control mode")

        if time.time() - web_time >= switch_time and web_thread_lock.locked():
            web_thread_lock.release()
            logging.info("switch to route control mode")
            route_deque.clear()
            route_finish_flag = 1
            package = struct.pack(
                "8B",
                int("AA", 16),
                int("F0", 16),
                int("08", 16),
                int("02", 16),
                int(0),
                int(0),
                int(0),
                int("BB", 16)
            )
            # ser.write(package)


if __name__ == "__main__":
    try:
        ser = serial.Serial(config.PI_SERIAL, config.BAUD_RATE, timeout=0.5)
        mqtt_client_start()

        # 启动 RTK 读取线程
        rtk_reader = RTKReader(config.RTK_SERIAL, config.RTK_BAUD_RATE)
        rtk_deque = rtk_reader.output_deque
        rtk_reader.start()

        threads = [
            threading.Thread(target=fusion_loop, daemon=True),
            threading.Thread(target=write_actual_location_data, args=(ser,), daemon=True),
            threading.Thread(target=serial_write_control, args=(ser,), daemon=True),
            threading.Thread(target=on_serial_data_received, args=(ser,), daemon=True),
            threading.Thread(target=control_mode_switch, daemon=True),
            # threading.Thread(target=kalman_message_publish, daemon=True)
        ]
        for thread in threads:
            thread.start()

        while True:
            time.sleep(1)
    finally:
        if "ser" in locals():
            ser.close()
