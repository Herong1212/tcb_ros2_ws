#!/usr/bin/env python3
"""
协作七自由度机械臂SDK(2207版本)
文件名: robotarm_sdk.py
作者: 杜宇坤
最后修改时间: 2026-02-10
"""
import glob
import json
import logging
import os
import re
import select
import socket
import struct
import threading
import time
from datetime import datetime

import colorlog
import numpy as np
import yaml
import zlib

from typing import Any, Dict, List, Optional, Tuple

class RobotArmSDK:
    def __init__(self, ip: str, 
                 port: int, 
                 log_level: int = 0, 
                 color_log: bool = True):
        """
        初始化SDK

        Args:
            ip(str): 机械臂IP地址
            port(int): 机械臂端口号
            log_level(int): 日志等级
                - 0: info
                - 1: debug
            color_log(bool): 是否使用彩色日志输出
                - True: 使用彩色日志(默认)
                - False: 使用无颜色日志
        
        Returns:
            None
        """
        self.ip = ip
        self.port = port
        self.socket: Optional[socket.socket] = None
        self._connect_flag = False 
        self.return_status: Dict[str, Any] = {} 
        self._receive_thread: Optional[threading.Thread] = None
        self._status_lock = threading.Lock()
        np.set_printoptions(suppress=True, precision=4)
        self.delay_time = 0.01
        self.color_log = color_log

        SDK_path = os.path.dirname(os.path.abspath(__file__))
        self.constants_path = SDK_path + "/config/constants.yaml"
        self.CONSTANTS = self._load_constants(self.constants_path)
        self.log_path = SDK_path + "/log"
        self.tcb_config_path = SDK_path + "/config/tcb_cfg.yaml"

        with open(self.tcb_config_path, "r", encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)

        self.dof = self.cfg["DOF"]

        if log_level == 0:
            self._setup_logger(logging.INFO)
            self.logger.info(f"初始化TCB SDK(22.07版本), 日志等级为 INFO")
        elif log_level == 1:
            self._setup_logger(logging.DEBUG)
            self.logger.info(f"初始化TCB SDK(22.07版本), 日志等级为 DEBUG")
        else:
            self.logger.error(f"日志等级错误, 检查是否为 0(INFO) 或 1 (DEBUG)")

        if self.port == 6001 or self.port ==7000:
            self.logger.info(f"连接ip: {self.ip}, 连接port: {self.port}")
        else:
            self.logger.error(f'连接port错误, 检查是否为 6001 或者 7000')

    def _setup_logger(self, log_level: int) -> None:
        os.makedirs(self.log_path, exist_ok=True)

        logger_name = f'TCB_{self.dof}dof_2207_{self.port}'

        # 使用统一的时间戳格式（年月日_小时）
        today_str = datetime.now().strftime("%Y-%m-%d_%H")
        log_file_path = os.path.join(self.log_path, f"{today_str}.log")

        if self.color_log:
            log_colors = {
                'DEBUG': 'cyan',
                'INFO': 'green',
                'WARNING': 'yellow',
                'ERROR': 'red',
                'CRITICAL': 'bold_red',
            }

            formatter = colorlog.ColoredFormatter(
                '%(log_color)s%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                log_colors=log_colors
            )
        else:
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)

        file_handler = logging.FileHandler(log_file_path, encoding='utf-8')
        file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(file_formatter)

        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(log_level)

        if not self.logger.handlers:
            self.logger.addHandler(console_handler)
            self.logger.addHandler(file_handler)
        else:
            self.logger.propagate = False

        # 删除多余日志文件（保留最近 50 个）
        log_files = sorted(glob.glob(os.path.join(self.log_path, "*.log")), key=os.path.getmtime)
        if len(log_files) > 50:
            for old_file in log_files[:-50]:
                try:
                    os.remove(old_file)
                except Exception as e:
                    self.logger.warning(f"删除旧日志文件失败: {old_file}, 错误: {e}")

    def __enter__(self):
        """支持with上下文管理器"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出时自动断开连接"""
        self.disconnect()
        
    def connect(self, retries: int = 3, delay: float = 2.0) -> bool:
        for attempt in range(1, retries + 1):
            try:
                self.socket = socket.socket()
                self.socket.settimeout(5)
                self.socket.connect((self.ip, self.port))
                self._connect_flag = False
                self._start_receive_thread()
                self.logger.info(f"成功连接到 {self.ip}:{self.port}")

                if self.port == 7000:
                    self.delay_time = 0.001
                elif self.port == 6001:
                    self.logger.info(f"当前SDK支持 {self.dof} 自由度的 TCB 机械臂")
                    self.delay_time = self.control_cycle_inquire() / 1000
                
                self.return_status = {}
                return True

            except socket.error as e:
                self.logger.warning(f"第 {attempt} 次连接失败: {e}")
                if attempt < retries:
                    self.logger.info(f"{delay} 秒后重试连接...")
                    time.sleep(delay)
                else:
                    self.logger.error(f"连接失败, 已重试 {retries} 次仍未成功")
                    return False

    def disconnect(self) -> bool:
        """断开连接并清理资源"""
        self._connect_flag = True
        
        current_thread = threading.current_thread()
        
        try:
            if self.socket:
                self.socket.close()
                self.socket = None
        except Exception as e:
            self.logger.error(f"关闭socket时出错: {e}")
            return False
        
        if self._receive_thread and current_thread != self._receive_thread:
            if self._receive_thread.is_alive():
                self._receive_thread.join(timeout=2)
                if self._receive_thread.is_alive():
                    self.logger.warning("接收线程未能在超时时间内停止")
                    return False
        
        time.sleep(1)
        self.logger.info(f"{self.ip}:{self.port} 连接已断开")
        return True

    def reconnect(self, retries=3, delay=2.0):
        """自动断开后重连"""
        self.logger.info("尝试重新连接控制器...")
        self.disconnect()
        return self.connect(retries=retries, delay=delay)

    def _start_receive_thread(self) -> None:
        """启动接收线程"""
        self._receive_thread = threading.Thread(
            target=self._receive_loop, 
            daemon=True
        )
        self._receive_thread.start()

        if self._receive_thread and self._receive_thread.is_alive():
            self.logger.info("接收线程已启动")
            return

    def _decode_robot_data(self, raw_data: bytes) -> Tuple[List[str], str]:
        
        cmd_word = f"{raw_data[4]:02X}{raw_data[5]:02X}"
        match = re.search(b'\{.*?\}', raw_data[6:])
        if match:
            json_str = match.group()
            try:
                decoded_messages = [json.loads(json_str)]
            except json.JSONDecodeError:
                match = re.search(b'\{.*\}', raw_data[6:])
                json_str = match.group()

            decoded_messages = [json_str]

        return decoded_messages, cmd_word
    
    def _receive_loop(self) -> None:
        while True:
            if self._connect_flag or not self.socket:
                break
            try:
                while not self._connect_flag and self.socket:
                    try:
                        readable, _, _ = select.select([self.socket], [], [], 1)
                        if readable:
                            data = self.socket.recv(2048)

                            self.logger.debug(f'收到的原始数据: {data}')
                            if data:
                                result, cmd_word = self._decode_robot_data(data)
                                for i in range(0, len(result)):
                                    self._parse_status(result[i], cmd_word)
                    except (OSError, ConnectionAbortedError):
                        pass 
            except Exception as e:
                # self.logger.error(f"接收线程异常: {e}")
                pass 

    def _crc(self, data_to: bytes, command: bytes, data_segment: str) -> bytes:
        length_bytes = struct.pack('>H', len(data_segment))
        crc32 = zlib.crc32(length_bytes + command + data_segment)
        crc_bytes = struct.pack('>I', crc32)
        return data_to + length_bytes + command + data_segment + crc_bytes

    def _send_command(self, cmd_word: bytes, cmd_data: str = None) -> None:
        if not self.socket:
            self.logger.error("未建立有效连接")
            raise ConnectionError("未建立有效连接")

        data_to_send = self._crc(self.CONSTANTS['FRAME_HEADER'], cmd_word, json.dumps(cmd_data).encode("GBK"))
        self.logger.debug(f'发送给控制器的数据为: {data_to_send}')

        try:
            self.socket.send(data_to_send)
            time.sleep(self.delay_time)

        except (BrokenPipeError, ConnectionResetError) as e:
            self.logger.error(f"命令发送失败: {e}, 尝试重连")
            if self.reconnect():
                try:
                    self.socket.send(data_to_send) 
                    time.sleep(self.delay_time)
                    self.logger.info("命令在重连后成功发送")
                except Exception as e2:
                    self.logger.error(f"重连后发送失败: {e2}")
            else:
                self.logger.error("重连失败, 命令丢失")
                self.reconnect()
                return 

    def _parse_status(self, data: bytes, cmd_word) -> None:
        with self._status_lock:
            json_str = data.decode('utf-8').strip()
            json_str_fixed = re.sub(r'\b(nan|NaN)\b', 'null', json_str)
            # Some frames include a "{*" or "{*#" prefix before the JSON object.
            # json_str_fixed = re.sub(r'^\{\*\#?', '', json_str_fixed)
            # json_str_fixed = re.sub(r'\{\*', '', json_str_fixed)

            try:
                parsed = json.loads(json_str_fixed)
                for key, value in parsed.items():
                    status_key = cmd_word + key

                    if status_key == "5534jobfilelist":
                        # '5534jobfilelist' 数据, 追加新的数据
                        if status_key in self.return_status:
                            self.return_status[status_key].extend(value)  # 添加新数据到已有列表中
                        else:
                            self.return_status[status_key] = value
                    elif status_key == "5534listnum":
                        # '5534listnum', 进行累加
                        if status_key in self.return_status:
                            self.return_status[status_key] += value 
                        else:
                            self.return_status[status_key] = value  
                    elif status_key == "5073servo":
                        # '5073servo' 数据, 追加新的数据
                        if status_key in self.return_status:
                            self.return_status[status_key].update(value) # 添加新数据到已有字典中
                        else:
                            self.return_status[status_key] = value 
                    else:
                        self.return_status[status_key] = value

                self.logger.debug(f"状态更新: {self.return_status}\n")

            except json.JSONDecodeError:
                self.logger.warning(f"部分解析失败（但不影响执行）, 原始数据: {json_str}")

    def _load_constants(self, file_path: str) -> bytes:
        with open(file_path, "r", encoding='utf-8') as f:
            data = yaml.safe_load(f)

        def convert_hex_to_bytes(obj):
            if isinstance(obj, dict):
                return {k: convert_hex_to_bytes(v) for k, v in obj.items()}
            elif isinstance(obj, str):
                return bytes.fromhex(obj)
            else:
                return obj

        return convert_hex_to_bytes(data)
    
    def _return_get(self, inquiry_key_name: str, key_value_clear: bool = True, timeout: float = 0.5) -> Optional[Any]:
        start = time.time()

        while True:
            key_value = self.return_status.get(inquiry_key_name, None)

            if key_value is not None:
                self.logger.debug(f"获取到 {inquiry_key_name} 对应的键值: {key_value}")
                break

            if time.time() - start > timeout:
                self.logger.debug(f"获取 {inquiry_key_name} 对应的键值超时")
                break

            time.sleep(0.05)

        if key_value_clear:
            self.logger.debug(f"清除 {inquiry_key_name} 对应的键值")
            self.return_status[inquiry_key_name] = None

        return key_value
    
    ########################################
    # API接口（6001端口）
    ########################################
    def heartbeat(self) -> bool:
        """
        心跳包

        Args:
            None
        
        Returns:
            bool: 心跳包状态标志
                - True:  心跳包正常
                - False: 心跳包异常
        """
        tolerance = 1

        timestamp = round(time.time(), 6)
        self._send_command(self.CONSTANTS['HEARTBEAT'], {"time": timestamp})
        self.logger.debug(f"发送心跳包的时间戳: {timestamp}")

        heartbeat_return = self._return_get('7267time', key_value_clear=False)
        time.sleep(self.delay_time)
        self.logger.debug(f"心跳包返回的时间戳: {heartbeat_return}")

        if heartbeat_return is None:
            self.logger.error("心跳包异常, 返回为空")
            return False

        if abs(heartbeat_return - timestamp) <= tolerance:
            self.logger.debug("心跳包正常")
            return True
        else:
            self.logger.error("心跳包异常")
            return False
        
    def fault_reset(self, robot: int =1) -> bool:
        """
        清除伺服错误

        Args:
            robot: 机器人号码, 默认为 1
        
        Returns:
            bool: 清错状态标志
                - True:  清错成功
                - False: 清错失败 (通过示教器查看无法清除的错误类型, 告知专业人员处理)
        """
        self._send_command(self.CONSTANTS['FAULT_RESET'], {"robot":robot})
        clearErrflag = self._return_get('3202clearErrflag')
        if clearErrflag == True:
            self.logger.info(f"清除伺服错误 成功")
        else:
            self.logger.info(f"清除伺服错误 失败")
        return clearErrflag
    
    def robot_stop(self, robot: int=1) -> None:
        """
        紧急停止

        Args:
            robot: 机器人号码, 默认为 1

        Returns:
            None
        """
        self._send_command(self.CONSTANTS['ROBOT_STOP'], {"robot":robot})
        self.logger.info(f"机器人 紧急停止！")

    def reboot_controller(self) -> None:
        """
        重启控制器

        Args:
            None

        Returns:
            None
        """
        self._send_command(self.CONSTANTS['REBOOT_CONTROLLER'])
        self.logger.info(f"重启控制器")
        return None
    
    def controller_init_finish_inquire(self) -> bool:
        """
        控制器初始化是否完成

        Args:
            None
        
        Returns:
            bool:控制器初始化状态
                - True: 控制器初始化完成
                - False: 控制器初始化未完成
        """
        self._send_command(self.CONSTANTS['CONTROLLER_INIT_FINISH'])
        finishinit = self._return_get('4306finishinit')
        if finishinit == True:
            self.logger.info(f'控制器初始化 完成')
        else:
            self.logger.info(f'控制器初始化 未完成')
        return finishinit
        
    def controller_ip_set(self, name: str, address: str, gateway: str= "", dns: str= "") -> None:
        """
        设置控制器ip

        Args:
            name(str): 网口名称
            address(str): ip地址
            gateway(str): 网关
            dns(str): DNS
        
        Returns:
            None
        """
        cmd_data = {
            "name": name,
            "address": address,
            "gateway": gateway,
            "dns": dns
            }
        self._send_command(self.CONSTANTS['CONTROLLER_IP']['SET'], cmd_data)
        time.sleep(0.5)
        self.logger.info(f'修改控制器网口 {name} 的ip为 {address}, dns为 {dns}, gateway为 {gateway}')
        self.logger.info(f'修改后控制器自动重启(ip修改前后相同不重启)')
        return None

    def controller_ip_inquire(self) -> dict:
        """
        查询控制器ip

        Args:
            None
        
        Returns:
            dict: 网口信息, 包含以下字段:
                - num(int): 网口数量
                - network(list): 每个网口的 ip、dns、gateway 以及网口名称
            返回 network(list) 数据示例: 
                [
                    {
                        "name": "eth0",
                        "address": "192.168.1.13",
                        "gateway": "192.168.1.1",
                        "dns": "114.114.114.114"
                    },
                    {
                        "name": "eth1",
                        "address": "192.168.1.14",
                        "gateway": "192.168.1.1",
                        "dns": "114.114.114.114"
                    }
                ]
        """
        self._send_command(self.CONSTANTS['CONTROLLER_IP']['INQUIRE'])
        num = self._return_get('4303num')
        network = self._return_get('4303network')
        self.logger.info(f'查询控制器IP')
        self.logger.info(f'> 网络ip数量: {num}')

        for number in range(0, num):
            address = network[number]['address']
            eth = network[number]['name']
            dns = network[number]['dns']
            gateway = network[number]['gateway']
            self.logger.info(f'> 网口 {eth}: {address}, dns: {dns}, gateway: {gateway}')
        
        return {"num": num, "network": network}

    def control_cycle_set(self, controlCycle: int) -> None:
        """
        设置控制器通讯周期

        Args:
            controlCycle: 参数为 1、2、4、8 毫秒(ms), 控制器重启生效

        Returns:
            None  
        """
        if controlCycle not in [1, 2, 4, 8]:
            self.logger.warning(f"控制器通讯周期 只能为 1、2、4、8 毫秒之一, 当前为: {controlCycle}")
            return
        else:
            self._send_command(self.CONSTANTS['CONTROL_CYCLE']['SET'], {"controlCycle":controlCycle})
            time.sleep(0.5)
            self.logger.info(f"机器人通讯周期设置为: {controlCycle}")
            self.logger.info(f"机器人通讯周期设置后, 控制器重启生效")
            return None
        
    def control_cycle_inquire(self) -> int:
        """
        查询控制器通讯周期

        Args:
            None
        
        Returns:
            int: 控制器通讯周期, 单位 毫秒(ms)
        """
        self._send_command(self.CONSTANTS['CONTROL_CYCLE']['INQUIRE'])
        controlCycle = self._return_get('2E09controlCycle')
        self.logger.info(f"机器人通讯周期为 {controlCycle} ms")
        return controlCycle

    def coord_mode_set(self, coord: int, robot: int=1) -> int:
        """
        设置坐标模式状态

        Args:
            robot(int): 机器人号码
            coord(int): 坐标模式状态
                - 0: 关节坐标(Joint)
                - 1: 直角坐标(Cart)
                - 2: 工具坐标(Tool)
                - 3: 用户坐标(User)
        
        Returns:
            int: 坐标模式状态
                - 0: 关节坐标(Joint)
                - 1: 直角坐标(Cart)
                - 2: 工具坐标(Tool)
                - 3: 用户坐标(User)
        """
        if coord not in [0, 1, 2, 3]:
            self.logger.warning(f"坐标模式状态设置参数只能为0(Joint)、1(Cart)、2(Tool)、3(User)之一, 当前为: {coord}")
            return 
        else:
            cmd_data = {"coord":coord,"robot": robot}
            self._send_command(self.CONSTANTS['COORD_MODE']['SET'], cmd_data)
            coord_return = self._return_get('2203coord')
            coord_map = {
                0: "关节坐标(Joint)",
                1: "直角坐标(Cart)",
                2: "工具坐标(Tool)",
                3: "用户坐标(User)"
            }
            self.logger.info(f"设置坐标模式为: {coord_map.get(coord_return)}")
            return coord_return

    def coord_mode_inquire(self, robot: int=1) -> int:
        """
        查询坐标模式状态

        Args:
            robot(int): 机器人号码

        Returns:
            int: 坐标模式状态
                - 0: 关节坐标(Joint)
                - 1: 直角坐标(Cart)
                - 2: 工具坐标(Tool)
                - 3: 用户坐标(User)
        """
        self._send_command(self.CONSTANTS['COORD_MODE']['INQUIRE'], {"robot": robot})
        coord_return = self._return_get('2203coord')
        coord_map = {
            0: "关节坐标(Joint)",
            1: "直角坐标(Cart)",
            2: "工具坐标(Tool)",
            3: "用户坐标(User)"
        }
        self.logger.info(f"查询当前坐标模式为: {coord_map.get(coord_return)}")
        return coord_return

    def deadman_status_set(self, deadman: int) -> int:
        """
        设置伺服上下电状态

        Args:
            deadman: 上下电状态
                - 0: 机器人下电
                - 1: 机器人上电
        
        Returns:
            int: 上下电状态
                - 0: 机器人下电
                - 1: 机器人上电
        """
        if deadman not in [0, 1]:
            self.logger.warning(f'上下电状态设置参数只能为0(下电)或者1(上电), 当前为: {deadman}')
            return
        else:
            cmd_data = {"deadman":deadman}
            self._send_command(self.CONSTANTS['DEADMAN_STATUS']['SET'], cmd_data)
            time.sleep(0.1)
            deadman_return = self._return_get('2303deadman')
            deadman_map = {
                0: "下电状态",
                1: "上电状态",
            }
            self.logger.info(f"设置上下电状态: {deadman_map.get(deadman_return, '未知状态')}")
            return deadman_return

    def deadman_status_inquire(self) -> int:
        """
        查询伺服上下电状态

        Args:
            None
        
        Returns:
            int: 上下电状态
                - 0: 机器人下电
                - 1: 机器人上电
        """
        self._send_command(self.CONSTANTS['DEADMAN_STATUS']['INQUIRE'])
        deadman_return = self._return_get('2303deadman')
        deadman_map = {
            0: "下电状态",
            1: "上电状态",
        }
        self.logger.info(f"查询上下电状态: {deadman_map.get(deadman_return, '未知状态')}")
        return deadman_return

    def servo_status_set(self, status: int, robot: int=1) -> int:
        """
        设置伺服状态

        Args:
            robot(int): 机器人号码
            status(int): 需要设置的伺服状态
                - 0: 停⽌
                - 1: 就绪
            
        Returns:
            int: 设置后的伺服状态
                - 0: 停⽌
                - 1: 就绪
                - 2: 错误
                - 3: 运⾏
        """
        if status not in [0, 1, 2, 3]:
            self.logger.warning(f"伺服状态设置参数只能为0、1、2、3, 其中只有0(停止)、1(就绪)可以设置, 当前为: {status}")
            return
        
        cmd_data = {
            "robot":robot,
            "status":status
            }
        
        self._send_command(self.CONSTANTS['SERVO_COMMANDS']['SET'], cmd_data)
        status_return = self._return_get('2003status')
        status_map = {
            0: "伺服 停止",
            1: "伺服 就绪",
            2: "伺服 错误",
            3: "伺服 运行"
        }
        self.logger.info(f"设置伺服状态: {status_map.get(status_return, '未知状态')}")
        return status_return

    def servo_status_inquire(self, robot: int = 1) -> int:
        """
        查询伺服状态

        Args:
            robot(int): 机器人号码
        
        Returns:
            int: 机器人当前伺服状态
                - 0: 停⽌
                - 1: 就绪
                - 2: 错误
                - 3: 运⾏
        """
        self._send_command(self.CONSTANTS['SERVO_COMMANDS']['INQUIRE'], {"robot":robot})
        status_return = self._return_get('2003status')
        status_map = {
            0: "伺服 停止",
            1: "伺服 就绪",
            2: "伺服 错误",
            3: "伺服 运行"
        }
        self.logger.info(f"查询伺服状态: {status_map.get(status_return, '未知状态')}")
        return status_return
    
    def servo_connect_inquire(self) -> int:
        """
        查询伺服连接状态

        Args: 
            None
        
        Returns:
            int: 伺服连接状态
                - 0: 真实伺服
                - 1: 虚拟伺服
                - 2: 无伺服
        """
        self._send_command(self.CONSTANTS['SERVO_CONNECT_INQUIRE'])
        servoType = self._return_get('5043servoConnect')
        if servoType == 0:
            self.logger.info(f"伺服连接状态: 真实伺服")
        elif servoType ==1:
            self.logger.info(f"伺服连接状态: 虚拟伺服")
        elif servoType == 2:
            self.logger.warning(f"伺服连接状态: 无伺服")
        return servoType
    
    def currentvel_inquire(self, robot: int=1) -> dict:
        """
        查询电机速度

        Args:
            robot(int): 机器人号码
        
        Returns:
            dict: 电机转速, 包括以下字段: 
                - vel: List[int], 电机速度, 单位: RPM
                - velSync: List[int], 外部轴电机速度, 单位: RPM
                - maxVel: List[int], 最大电机速度, 单位: RPM
                - maxVelSync: List[int], 最大外部轴电机速度, 单位: RPM
        """
        self._send_command(self.CONSTANTS['CURRENTVEL_INQUIRE'], {"robot":robot})
        print(self.return_status)
        vel = self._return_get('2A05vel')
        velSync = self._return_get('2A05velSync')
        maxVel = self._return_get('2A05maxVel')
        maxVelSync = self._return_get('2A05maxVelSync')
        self.logger.info(f'查询电机速度')
        self.logger.info(f'> 电机速度({len(vel)}轴): {vel} RPM')
        self.logger.info(f'> 外部轴电机速度: {velSync} RPM')
        self.logger.info(f'> 最大电机速度({len(maxVel)}轴): {maxVel} RPM')
        self.logger.info(f'> 最大外部轴电机速度: {maxVelSync} RPM')
        result = {
            "vel": vel,
            "velSync": velSync,
            "maxVel": maxVel,
            "maxVelSync": maxVelSync
        }
        return result
    
    def currenttorq_inquire(self, robot: int=1) -> dict:
        """
        查询电机扭矩

        Args: 
            robot(int): 机器人号码
        
        Returns:
            dict: 电机扭矩, 包括以下字段: 
                - torq: List[float], 机器人当前电机扭矩列表, 单位: ‰
                - theoTorq: List[float], 理论电机扭矩, 单位: ‰
                - maxTorq: List[float], 电机最大扭矩, 单位: ‰
                - maxTheoTorq: List[float], 理论电机最大扭矩, 单位: ‰
                - torqSync: List[float], 外部轴电机扭矩, 单位: ‰
                - maxTorqSync: List[float], 外部轴电机最大扭矩, 单位: ‰
        """
        self._send_command(self.CONSTANTS['CURRENTTORQ_INQUIRE'], {"robot":robot})
        torq = self._return_get('2A07torq')
        theoTorq = self._return_get('2A07theoTorq')
        maxTorq = self._return_get('2A07maxTorq')
        maxTheoTorq = self._return_get('2A07maxTheoTorq')
        torqSync = self._return_get('2A07torqSync')        
        maxTorqSync = self._return_get('2A07maxTorqSync')

        # self.logger.info(f'查询电机扭矩')
        # self.logger.info(f'> 电机扭矩({len(torq)}轴): {torq}')
        # self.logger.info(f'> 理论电机扭矩: {theoTorq}')
        # self.logger.info(f'> 电机最大扭矩({len(maxTorq)}轴): {maxTorq}')
        # self.logger.info(f'> 理论电机最大扭矩: {maxTheoTorq}')
        # self.logger.info(f'> 外部轴电机扭矩: {torqSync}')
        # self.logger.info(f'> 外部轴电机最大扭矩: {maxTorqSync}')
        result = {
            "torq": torq,
            "theoTorq": theoTorq,
            "maxTorq": maxTorq,
            "maxTheoTorq": maxTheoTorq,
            "torqSync": torqSync,
            "maxTorqSync": maxTorqSync
        }
        return result

    def axis_actual_vel_inquire(self, robot: int=1) -> dict:
        """
        查询轴速度

        Args:
            robot(int): 机器人号码

        Returns:
            dict: 轴速度, 包括以下字段: 
                - actualLineVel: float, 当前末端线速度, 单位 mm/s
                - maxActualLineVel: float, 当前末端最大线速度, 单位 mm/s
                - axisActualVel: List[float], 当前轴速度, 单位 °/s 
                - maxAxisActualVel: List[float], 当前最大轴速度, 单位 °/s
                - axisActualVelSync: List[float], 外部轴当轴速度, 单位 °/s
                - maxAxisActualVelSync: List[float], 外部轴最大轴速度, 单位 °/s
        """
        self._send_command(self.CONSTANTS['AXISACTUALVEL_INQUIRE'], {"robot":robot})
        actualLineVel = self._return_get('2A23actualLineVel')
        maxActualLineVel = self._return_get('2A23maxActualLineVel')
        axisActualVel = self._return_get('2A23axisActualVel')
        maxAxisActualVel = self._return_get('2A23maxAxisActualVel')
        axisActualVelSync = self._return_get('2A23axisActualVelSync')
        maxAxisActualVelSync = self._return_get('2A23maxAxisActualVelSync')
        # self.logger.info(f'查询轴速度')
        # self.logger.info(f'> 当前末端线速度: {actualLineVel} mm/s')
        # self.logger.info(f'> 当前末端最大线速度: {maxActualLineVel} mm/s')
        # self.logger.info(f'> 1-{len(axisActualVel)} 轴当前轴速度: {axisActualVel} , 单位: °/s')
        # self.logger.info(f'> 1-{len(maxAxisActualVel)} 轴当前最大轴速度: {maxAxisActualVel} , 单位: °/s')
        # self.logger.info(f'> 外部轴当轴速度: {axisActualVelSync} , 单位: °/s')
        # self.logger.info(f'> 外部轴最大轴速度: {maxAxisActualVelSync} , 单位: °/s')
        result = {
            "actualLineVel": actualLineVel,
            "maxActualLineVel": maxActualLineVel,
            "axisActualVel": axisActualVel,
            "maxAxisActualVel": maxAxisActualVel,
            "axisActualVelSync": axisActualVelSync,
            "maxAxisActualVelSync": maxAxisActualVelSync
        }
        return result

    def interpolation_mode_set(self, interpolationMethod: int, 
                               absolutePosResolution: float, 
                               runDelayTime:int, 
                               stopTime:int) -> None:
        """
        设置运动参数
         
        Args:
            interpolationMethod(int): 机器人插补方式 
                - 0: S型
                - 1: 梯形
                - 2: 加加插补
            absolutePosResolution(float): 绝对位置分辨率, 范围0.0001~0.1, 单位: 度
            runDelayTime(int): 运行延时时间, 范围 50~20000, 单位: 毫秒
            stopTime(int): 暂停时间, 范围 240~2000, 单位: 毫秒
        
        Returns:
            None
        """
        if interpolationMethod not in [0, 1, 2]:
            self.logger.warning(f"机器人插补方式应为0(S型)、1(梯形)、2(加加插补)之一, 当前为: {interpolationMethod}")
            return
        
        if not (0.0001 <= absolutePosResolution <= 0.1):
            self.logger.warning(f"绝对位置分辨率应在0.0001~0.1之间, 当前为: {absolutePosResolution}")
            return
        
        if not (500 <= runDelayTime <= 20000):
            self.logger.warning(f"运行延时时间应在500~20000之间, 当前为: {runDelayTime}")
            return

        if not (240 <= stopTime <= 2000):
            self.logger.warning(f"暂停时间应在240~2000之间, 当前为: {stopTime}")
            return
        
        cmd_data = {
            "interpolationMethod":interpolationMethod,         
            "absolutePosResolution":absolutePosResolution,
            "runDelayTime":runDelayTime,
            "stopTime":stopTime
            }
        method_map = {
            0:"S型",
            1:"梯形",
            2:"加加插补"
        }
        self._send_command(self.CONSTANTS['INTERPOLATION_MODE']['SET'], cmd_data)
        time.sleep(1)
        self.logger.info(f'设置运动参数')
        self.logger.info(f'> 机器人插补方式: {method_map.get(interpolationMethod)}')
        self.logger.info(f'> 绝对位置分辨率: {absolutePosResolution} 度')
        self.logger.info(f'> 运行延时时间: {runDelayTime} 毫秒')
        self.logger.info(f'> 暂停时间:  {stopTime} 毫秒\n')

    def interpolation_mode_inquire(self) -> dict:
        """
        查询机器人运动参数

        Args:
            None

        Returns:
            dict: 机器人运动参数字典, 包含以下键值: 
                - absolutePosResolution (float): 绝对位置分辨率, 单位度, 取值范围: [0.0001, 0.1]
                - interpolationMethod (int): 插补方式标识: 
                    - 0: S型插补
                    - 1: 梯形插补
                    - 2: 加加速度插补
                - minTrajectTime (dict): 最小轨迹时间配置字典: 
                    - minAccTime (float): 最小加速度时间, 单位秒, 取值范围: [0.05, 1.0]
                    - minDecTime (float): 最小减速度时间, 单位秒, 取值范围: [0.05, 1.0]
                - runDelayTime (int): 运行延迟时间, 单位毫秒, 取值范围: [500, 20000]
                - stopTime (int): 暂停时间, 单位毫秒, 取值范围: [240, 2000]
        """
        self._send_command(self.CONSTANTS['INTERPOLATION_MODE']['INQUIRE'])
        time.sleep(1)
        print(self.return_status)
        interpolationMethod = self._return_get('2803interpolationMethod')
        minTrajectTime = self._return_get('2803minTrajectTime')
        minAccTime = minTrajectTime['minAccTime']
        minDecTime = minTrajectTime['minDecTime']
        method_map = {
            0:"S型",
            1:"梯形",
            2:"加加插补"
        }
        absolutePosResolution = self._return_get('2803absolutePosResolution')
        runDelayTime = self._return_get('2803runDelayTime')
        stopTime = self._return_get('2803stopTime')
        self.logger.info(f'查询运动参数')
        self.logger.info(f'> 机器人插补方式: {method_map.get(interpolationMethod)}')
        self.logger.info(f'> 绝对位置分辨率: {absolutePosResolution} 度')
        self.logger.info(f"> 最小加速度时间: {minAccTime} s")
        self.logger.info(f"> 最小减速度时间: {minDecTime} s")
        self.logger.info(f"> 运行延迟时间: {runDelayTime} ms")
        self.logger.info(f"> 暂停时间: {stopTime} ms")
        result = {
            "interpolationMethod": interpolationMethod,
            "absolutePosResolution": absolutePosResolution,
            "runDelayTime": runDelayTime,
            "stopTime": stopTime,
            "minTrajectTime": {
                "minAccTime": minAccTime,
                "minDecTime": minDecTime,
            },
        }
        return result

    def servo_inside_parm_inqure(self, servoNum: int, robot: int=1) -> tuple[int, int, int, int, int]:
        """
        查询伺服参数

        Args:
            robot(int): 机器人号码
            servoNum(int): 伺服号
        
        Returns:
            tuple[int, int, int, int, int]:
                - int: 编码器错误码(这里返回的十进制, 需要自己转为十六进制查询)
                - int: 编码器状态寄存器1
                - int: 编码器状态寄存器2
                - int: 编码器单圈值
                - int: 抱闸手动控制状态
                    - 0: 抱闸关闭
                    - 1: 抱闸打开
        """
        if not (1 <= servoNum <= self.dof):
            self.logger.warning(f"输入的伺服号应该为 1 ~ {self.dof} 的关节号, 当前为: {servoNum}")
            return
        
        cmd_data = {
            "robot":robot,
            "servoNum":servoNum
        }
        self._send_command(self.CONSTANTS['SERVO_INSIDE_PARM']['INQUIRE'], cmd_data)
        time.sleep(2)

        def extract_values():
            """从 return_status 中提取值"""
            try:
                servo_data = self.return_status.get('5073servo', {})
                return (
                    servo_data.get('编码器错误码', {}).get('value'),
                    servo_data.get('编码器状态寄存器1', {}).get('value'),
                    servo_data.get('编码器状态寄存器2', {}).get('value'),
                    servo_data.get('编码器单圈值', {}).get('value'),
                    servo_data.get('抱闸手动控制', {}).get('value'),
                )
            except Exception as e:
                self.logger.warning(f'提取伺服参数异常: {e}')
                return None, None, None, None, None
        
        error_code, encoder_status_register1, encoder_status_register2, encoder_single_turn_value, holding_brake_status = extract_values()
        
        if error_code is None or encoder_status_register1 is None or encoder_status_register2 is None or encoder_single_turn_value is None or holding_brake_status is None:
            self.logger.warning("部分伺服参数为空, 等待 2 秒后重试一次查询")
            time.sleep(2.8)
            error_code, encoder_status_register1, encoder_status_register2, encoder_single_turn_value, holding_brake_status = extract_values()

        self.logger.info(f'查询伺服参数')
        # self.logger.info(f'> 伺服 {servoNum} 编码器错误码: {hex(error_code)}')
        self.logger.info(f'> 伺服 {servoNum} 编码器单圈值: {encoder_single_turn_value}')
        self.logger.info(f'> 伺服 {servoNum} 编码器状态寄存器1: {encoder_status_register1}')
        self.logger.info(f'> 伺服 {servoNum} 编码器状态寄存器2: {encoder_status_register2}')
        self.logger.info(f'> 伺服 {servoNum} 抱闸手动控制状态: {holding_brake_status}')

        return error_code, encoder_status_register1, encoder_status_register2, encoder_single_turn_value, holding_brake_status

    def servo_inside_parm_set(self, servoNum: int, 
                              key_name: str, 
                              key_value: int, 
                              temporary_save: int=1,
                              robot: int=1) -> None:
        """
        伺服参数设置

        Args:
            robot(int): 机器人号码
            servoNum(int): 伺服号
            key_name(str): 伺服参数名
            key_value(int): 伺服参数值
            temporary_save(int):
                - 0: 修改
                - 1: 临时存储(上下电之后恢复原来值)

        Returns:
            None
        """
        if not (1 <= servoNum <= self.dof):
            self.logger.warning(f"输入的伺服号应为 1 ~ {self.dof} 的关节号, 当前为: {servoNum}")
            return
        
        VALID_SERVO_PARAMS = ["6041", "6072", "60E0", "60E1",
                              "位置环比例增益1", "初始化指令", "抱闸关闭延时", "抱闸启动延时", "抱闸手动控制",
                              "母线电压值", "电压峰值", "电压最低值", "电机硬件版本", "电机编码", "电机软件版本", 
                              "电流环比例增益", "电流环积分时间常数", "编码器单圈值", "编码器命令", "编码器多圈值",
                              "编码器状态寄存器1", "编码器状态寄存器2", "编码器状态寄存器3", "编码器错误码", "警告状态",
                              "输入侧温度值", "速度环增益", "速度环积分时间常数"] 

        if key_name not in VALID_SERVO_PARAMS:
            # raise ValueError(f"无效的伺服参数名: {key_name}, 有效伺服参数名为: {VALID_SERVO_PARAMS}")
            self.logger.warning(f"无效的伺服参数名: {key_name}, 有效伺服参数名为: {VALID_SERVO_PARAMS}")
            return
        else:
            cmd_data = {
                "robot": robot,
                "servo": {key_name:{"value":key_value}},
                "servoNum": servoNum,
                "temporary_save": temporary_save 
            }
            self._send_command(self.CONSTANTS['SERVO_INSIDE_PARM']['SET'], cmd_data)
            self.logger.info(f'修改伺服 {servoNum} 的伺服参数 "{key_name}" 的值为 {key_value}')
    
    def slavetype_list_respond(self) -> dict:
        """
        查询从站列表

        Args:
            None

        Returns:
            dict: 从站列表
                - IO编号 (List[int]): 从站的 I/O 编号
                - 伺服编号 (List[int]): 从站的伺服编号
                - 伺服型号 中文 (List[str]): 从站的伺服型号（中文）
                - 伺服型号 英文 (List[str]): 从站的伺服型号（英文）
        """
        self._send_command(self.CONSTANTS['SLAVETYPE_LIST_INQUIRE'])
        IONum = self._return_get('2E0FIONum')
        servoNum = self._return_get('2E0FservoNum')
        slaveType = self._return_get('2E0FslaveType')
        slaveTypeEnglish = self._return_get('2E0FslaveTypeEnglish')
        self.logger.info(f'查询从站列表')
        self.logger.info(f'> IO编号: {IONum}')
        self.logger.info(f'> 伺服编号: {servoNum}')
        self.logger.info(f'> 伺服型号 中文: {slaveType}')
        self.logger.info(f'> 伺服型号 英文: {slaveTypeEnglish}')
        result = {
            "IONum": IONum,
            "servoNum": servoNum,
            "slaveType": slaveType,
            "slaveTypeEnglish": slaveTypeEnglish
        }
        return result
        
    def operation_mode_set(self, mode: int) -> int:
        """
        设置操作模式状态

        Args: 
            mode(int):
                - 0: ⽰教模式(Teach)
                - 1: 远程模式(Circle)
                - 2: 运⾏模式(Repeat)

        Returns:
            int: 操作模式状态
                - 0: ⽰教模式(Teach)
                - 1: 远程模式(Circle)
                - 2: 运⾏模式(Repeat)
        """
        if mode not in [0, 1, 2]:
            self.logger.warning(f"操作模式状态设置参数应该为0(示教)、1(远程)、2(运行)之一, 当前为: {mode}")
            return

        self._send_command(self.CONSTANTS['OPERATION_MODE']['SET'], {"mode": mode})
        mode_return = self._return_get('2103mode')
        mode_map = {
            0: "⽰教模式(Teach)",
            1: "远程模式(Circle)",
            2: "运⾏模式(Repeat)",
        }
        self.logger.info(f"设置操作模式: {mode_map.get(mode_return)}")
        return mode_return

    def operation_mode_inquire(self) -> int:
        """
        查询操作模式状态

        Args:
            None
        
        Returns:
            int: 操作模式状态
                - 0: ⽰教模式(Teach)
                - 1: 远程模式(Circle)
                - 2: 运⾏模式(Repeat)
        """
        self._send_command(self.CONSTANTS['OPERATION_MODE']['INQUIRE'])
        mode_return = self._return_get('2103mode')
        mode_map = {
            0: "⽰教模式(Teach)",
            1: "远程模式(Circle)",
            2: "运⾏模式(Repeat)",
        }
        self.logger.info(f"查询操作模式: {mode_map.get(mode_return, '未知模式')}")      
        return mode_return

    def jointparameter_set(self, AxisNum: int, 
                           PosSWLimit: float, 
                           NegSWLimit: float, 
                           Direction:int) -> None:
        """
        设置关节参数

        Args:
            AxisNum(int): 关节轴数
            PosSWLimit(float):关节正限位
            NegSWLimit(float):关节反限位
            Direction(int):模型方向, 七轴的时候，四号关节(六轴的时候为三号关节)设置前查询一下是否与其他关节相反
                - 1: 正向
                - -1: 反向
        
        Returns:
            None
        """
        if not (1 <= AxisNum <= self.dof):
            self.logger.warning(f"关节轴数应为 1 ~ {self.dof} 的关节号, 当前为: {AxisNum}")
            return

        cmd_data = {
            "Joint":{
                "AxisDirection":1,
                "AxisNum":AxisNum,
                "BackLash":0.0,
                "DeRatedVel":-180.0,
                "Direction":Direction,
                "EncoderResolution":19,
                "MaxAcc":1.0,
                "MaxDeRotSpeed":-1.0,
                "MaxDecel":-1.0,
                "MaxJerkAcc":1.0,
                "MaxJerkDec":-1.0,
                "MaxRotSpeed":1.0,
                "NegSWLimit":NegSWLimit,
                "PosSWLimit":PosSWLimit,
                "RatedDeRotSpeed":-3000.0,
                "RatedRotSpeed":3000,
                "RatedVel":180,
                "ReducRatio":100,
                "reduce_ratio_enable":True
                }
            }
        self._send_command(self.CONSTANTS['JOINTPARAMETER']['SET'], cmd_data)
        self.logger.info(f'关节 {AxisNum} 参数已经设置, 关节正限位: {PosSWLimit} °, 关节反限位: {NegSWLimit} °')

    def jointparameter_inquery(self, AxisNum: int) -> dict:
        """
        查询关节参数

        Args:
            AxisNum(int): 代表关节轴号

        Returns:
            dict: 关节参数
                - AxisDirection(int): 关节实际方向
                - AxisNum(int): 查询参数关节
                - BackLash(float): 齿轮反向间隙
                - DeRatedVel(float): 关节额定反速度
                - Direction(int): 模型方向
                - EncoderResolution(int): 编码器位数
                - MaxAcc(float): 最大加速度
                - MaxDeRotSpeed(float): 最大反转速
                - MaxDecel(float): 最大减速度
                - MaxJerkAcc(float): 最大加加速度
                - MaxJerkDec(float): 最大减减速度
                - MaxRotSpeed(float): 最大正转速
                - PosSWLimit(float): 关节正限位
                - NegSWLimit(float): 关节反限位
                - RatedRotSpeed(float): 关节额定正转速
                - RatedDeRotSpeed(float): 关节额定反转速
                - RatedVel(float): 关节额定正速度
                - ReducRatio(float): 减速比
                - reduce_ratio_enable(bool): 编码器是否经过减速机
        """
        if not (1 <= AxisNum <= self.dof):
            self.logger.warning(f"关节轴数应为 1 ~ {self.dof} 的关节号, 当前为: {AxisNum}")
            return
        
        self._send_command(self.CONSTANTS['JOINTPARAMETER']['INQUIRE'], {"AxisNum": AxisNum})
        time.sleep(0.1)
        AxisDirection       = self.return_status.get('3B03Joint')['AxisDirection']
        AxisNum             = self.return_status.get('3B03Joint')['AxisNum']
        BackLash            = self.return_status.get('3B03Joint')['BackLash']
        DeRatedVel          = self.return_status.get('3B03Joint')['DeRatedVel']
        Direction           = self.return_status.get('3B03Joint')['Direction']
        EncoderResolution   = self.return_status.get('3B03Joint')['EncoderResolution']
        MaxAcc              = self.return_status.get('3B03Joint')['MaxAcc']
        MaxDeRotSpeed       = self.return_status.get('3B03Joint')['MaxDeRotSpeed']
        MaxDecel            = self.return_status.get('3B03Joint')['MaxDecel']
        MaxJerkAcc          = self.return_status.get('3B03Joint')['MaxJerkAcc']
        MaxJerkDec          = self.return_status.get('3B03Joint')['MaxJerkDec']
        MaxRotSpeed         = self.return_status.get('3B03Joint')['MaxRotSpeed']
        NegSWLimit          = self.return_status.get('3B03Joint')['NegSWLimit']
        PosSWLimit          = self.return_status.get('3B03Joint')['PosSWLimit']
        RatedDeRotSpeed     = self.return_status.get('3B03Joint')['RatedDeRotSpeed']
        RatedRotSpeed       = self.return_status.get('3B03Joint')['RatedRotSpeed']        
        RatedVel            = self.return_status.get('3B03Joint')['RatedVel']
        ReducRatio          = self.return_status.get('3B03Joint')['ReducRatio']
        reduce_ratio_enable = self.return_status.get('3B03Joint')['reduce_ratio_enable']
        result = {
            "AxisDirection": AxisDirection,
            "AxisNum": AxisNum,
            "BackLash": BackLash,
            "DeRatedVel": DeRatedVel,
            "Direction": Direction,
            "EncoderResolution": EncoderResolution,
            "MaxAcc": MaxAcc,
            "MaxDeRotSpeed": MaxDeRotSpeed,
            "MaxDecel": MaxDecel,
            "MaxJerkAcc": MaxJerkAcc,
            "MaxJerkDec": MaxJerkDec,
            "MaxRotSpeed": MaxRotSpeed,
            "NegSWLimit": NegSWLimit,
            "PosSWLimit": PosSWLimit,
            "RatedDeRotSpeed": RatedDeRotSpeed,
            "RatedRotSpeed": RatedRotSpeed,
            "RatedVel": RatedVel,
            "ReducRatio": ReducRatio,
            "reduce_ratio_enable": reduce_ratio_enable
        }
        self.logger.info(f'查询关节参数')
        self.logger.info(f'> 查询参数关节: {AxisNum} 关节')        
        self.logger.info(f'> 关节正限位: {PosSWLimit} 度')
        self.logger.info(f'> 关节反限位: {NegSWLimit} 度')
        self.logger.info(f'> 减速比: {ReducRatio}')
        self.logger.info(f'> 编码器位数: {EncoderResolution}')
        self.logger.info(f'> 关节额定正转速: {RatedRotSpeed} 转/分钟')           
        self.logger.info(f'> 关节额定反转速: {RatedDeRotSpeed} 转/分钟')
        self.logger.info(f'> 最大正转速: {MaxRotSpeed} 倍数')
        self.logger.info(f'> 最大反转速: {MaxDeRotSpeed} 倍数')        
        self.logger.info(f'> 关节额定正速度: {RatedVel} 度/秒')
        self.logger.info(f'> 关节额定反速度: {DeRatedVel} 度/秒')
        self.logger.info(f'> 最大加速度: {MaxAcc} 倍数')
        self.logger.info(f'> 最大减速度: {MaxDecel} 倍数')
        self.logger.info(f'> 最大加加速度: {MaxJerkAcc}')
        self.logger.info(f'> 最大减减速度: {MaxJerkDec}')
        self.logger.info(f'> 关节实际方向: {AxisDirection}')
        self.logger.info(f'> 模型方向: {Direction}')
        self.logger.info(f'> 齿轮反向间隙: {BackLash}')
        self.logger.info(f'> 编码器是否经过减速机: {reduce_ratio_enable}')
        return result

    def decareparameter_set(self, MaxVel: int, 
                            MaxAcc: int, 
                            MaxDec: int,
                            MaxJerk: int, 
                            MaxAttitudeVel: int, 
                            SpeedLimitMode: int) -> None:
        """
        设置机器人笛卡尔参数

        Args:
            MaxVelint(int): 最大速度, 范围[1,5000], 单位: mm/s
            MaxAcc(int): 最大加速度, 范围[1,15], 单位: 倍数
            MaxDec(int): 最大减速度, 范围[-15,-1], 单位: 倍数
            MaxJerk(int): 最大加加速度, 单位: mm/s³
            MaxAttitudeVel(int): 姿态运动最大速度, 范围[1-1000], 单位: °/s
            SpeedLimitMode(int): 速度限制方式
                    - 0: 位姿
                    - 1: 位置
        
        Returns:
            None
        """
        cmd_data ={"Decare":{
            "MaxVel": MaxVel,
            "MaxAcc": MaxAcc,
            "MaxDec": MaxDec,
            "MaxJerk": MaxJerk,
            "MaxAttitudeVel": MaxAttitudeVel,
            "SpeedLimitMode": SpeedLimitMode
        }}
        self._send_command(self.CONSTANTS['DECAREPARAMETER']['SET'], cmd_data)
        time.sleep(1)
        self.logger.info(f"设置笛卡尔坐标参数")
        self.logger.info(f"> 最大速度: {MaxVel} mm/s")
        self.logger.info(f"> 最大加速度: {MaxAcc} 倍数")
        self.logger.info(f"> 最大减速度: {MaxDec} 倍数")
        self.logger.info(f"> 最大加加速度: {MaxJerk} mm/s³")
        self.logger.info(f"> 姿态运动最大速度: {MaxAttitudeVel} °/s")
        SpeedLimitMode_map = {
            0: "位姿",
            1: "位置"
        }
        self.logger.info(f"> 速度限制方式: {SpeedLimitMode_map.get(SpeedLimitMode)}")
    
    def decareparameter_inquire(self) -> dict:
        """
        查询机器人笛卡尔参数

        Args:
            None
        
        Returns:
            dict: 笛卡尔参数
                - MaxVel: int, 最大速度, 单位: mm/s
                - MaxAcc: int, 最大加速度, 单位: 倍数
                - MaxDec: int, 最大减速度, 单位: 倍数
                - MaxJerk: int, 最大加加速度, 单位: mm/s³
                - MaxAttitudeVel: int, 姿态运动最大速度, 单位: °/s
                - SpeedLimitMode: int, 速度限制方式
                    - 0: 位姿
                    - 1: 位置
        """
        self._send_command(self.CONSTANTS['DECAREPARAMETER']['INQUIRE'])
        decareparameter = self._return_get('3B06Decare')
        MaxAcc = decareparameter.get('MaxAcc')
        MaxAttitudeVel = decareparameter.get('MaxAttitudeVel')
        MaxDec = decareparameter.get('MaxDec')
        MaxJerk = decareparameter.get('MaxJerk')
        MaxVel = decareparameter.get('MaxVel')
        SpeedLimitMode = decareparameter.get('SpeedLimitMode')
        self.logger.info(f"查询笛卡尔坐标参数")
        self.logger.info(f"> 最大速度: {MaxVel} mm/s")
        self.logger.info(f"> 最大加速度: {MaxAcc} 倍数")
        self.logger.info(f"> 最大减速度: {MaxDec} 倍数")
        self.logger.info(f"> 最大加加速度: {MaxJerk} mm/s³")
        self.logger.info(f"> 姿态运动最大速度: {MaxAttitudeVel} °/s")
        SpeedLimitMode_map = {
            0: "位姿",
            1: "位置"
        }
        self.logger.info(f"> 速度限制方式: {SpeedLimitMode_map.get(SpeedLimitMode)}")
        return decareparameter

    def job_open_inquire(self, robot: int = 1) -> str:
        """
        当前打开的作业文件获取

        Args:
            robot(int): 机器人号码
        
        Returns:
            str: 作业文件名称
        """
        self._send_command(self.CONSTANTS['JOB_CONTROL']['JOB_OPEN_INQUIRE'], {"robot":robot})
        job_name = self._return_get('3A05openedJobName')
        self.logger.info(f'当前打开作业文件为: {job_name}')
        return job_name

    def jobsend_done(self, jobname: str, 
                     line: int=1, 
                     continueRun: int=0, 
                     startOver: int=1, 
                     robot: int=1) -> None:
        """
        开始运行作业文件

        Args:
            robot(int): 机器人号码
            jobname(str): 作业文件名字
            line(int): 作业⽂件指令⾏数,不能为零, 不能超过总⾏数
            continueRun(int): 
                - 1: 继续运⾏
                - 0: 不继续运⾏
            startOver(int):
                - 1: 重头运行
                - 0: 继续运行
            "continueRun"和"startOver"的组合使用:
                - "continueRun":1 "startOver":1 断点执行
                - "continueRun":0 "startOver":1 从头 运行
                - "continueRun":0 "startOver":0 当前行 运行
            
        Returns: 
            None
        """
        cmd_data = {
            "robot":robot,
            "jobname":jobname,
            "line":line,
            "continueRun":continueRun,
            "startOver": startOver
            }
        self.logger.info(f"运行作业文件: {jobname}")
        self._send_command(self.CONSTANTS['JOB_CONTROL']['JOBSEND_DONE'], cmd_data)

    def stop_job_run(self, robot: int = 1) -> None:
        """
        停止正在运行的作业文件

        Args:
            robot(int): 机器人号码    

        Returns: 
            None   
        """
        self._send_command(self.CONSTANTS['JOB_CONTROL']['STOP_JOB_RUN'], {"robot":robot})
        self.logger.info(f'停止正在运行的作业文件')

    def jobfile_list_inquire(self) -> list:
        """
        获取作业文件列表

        Args:
            None
        
        Returns:
            list:作业文件列表
        """
        self._send_command(self.CONSTANTS['JOBFILE_LIST_INQUIRE'])
        absolutepath = self.return_status.get('5533absolutepath')
        jobfilenum   = self.return_status.get('5533jobfilenum')
        jobfilelist  = self.return_status.get('5534jobfilelist')
        listnum      = self.return_status.get('5534listnum')
        self.logger.info(f'获取作业文件列表')
        self.logger.info(f'> 作业文件路径: {absolutepath}')
        self.logger.info(f'> 各作业文件路径下的作业文件数量: {jobfilenum}')
        self.logger.info(f'> {listnum}个文件为: {jobfilelist}')
        return jobfilelist

    def speed_set(self, speed: int, robot: int=1) -> int:
        """
        设置全局速度

        Args:
            robot(int): 机器人号码
            speed(int): 全局速度设置   0-100
                - 0-100: 速度值, 百分比
                - 101: 0.1°微动档
                - 102: 0.01°微动档
                - 103: 0.001°微动档
        
        Returns: 
            int: 控制器返回设置后的速度值
                - 0-100: 速度值, 百分比
                - 101: 0.1°微动档
                - 102: 0.01°微动档
                - 103: 0.001°微动档
        """
        if not 0 <= speed <= 103:
            # raise ValueError("速度值必须在0-103之间")
            self.logger.warning(f'速度值必须在0~103之间, 当前为: {speed}')
            return
        else:
            cmd_data = {"robot":robot,"speed":speed}
            self._send_command(self.CONSTANTS['SPEED_CONTROL']['SET'], cmd_data)
            speed_return = self._return_get('2603speed')
            self.logger.info(f"设置全局速度: {speed_return} %")
            return speed_return
    
    def speed_inquire(self, robot: int = 1) -> int:
        """
        查询全局速度

        Args:
            robot(int): 机器人号码

        Returns: 
            int: 控制器返回当前的速度值
                - 0-100: 速度值, 百分比
                - 101: 0.1°微动档
                - 102: 0.01°微动档
                - 103: 0.001°微动档
        """
        self._send_command(self.CONSTANTS['SPEED_CONTROL']['INQUIRE'], {"robot":robot})
        speed = self._return_get('2603speed')
        self.logger.info(f"查询全局速度: {speed} %")
        return speed

    def currentpos_inquiry(self, coord: int, robot: int=1) -> list:
        """
        获取当前位置

        Args:
            robot(int): 机器人号码
            coord(int): 坐标模式
                - -1: 控制器当前坐标
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            
        Returns:
            List[float]: 
                - 关节坐标分别代表1-7关节角度值
                - 直角坐标系分别x,y,z,a,b,c工具、用户同直角
        """
        if coord not in [-1, 0, 1, 2, 3]:
            self.logger.warning(f"坐标参数应在0(关节坐标)、1(直角坐标)、2(工具坐标)、3(用户坐标)之一, 当前为: {coord}")
            return

        cmd_data = {"robot":robot,"coord":coord}
        self._send_command(self.CONSTANTS['CURRENTPOS_INQUIRE'], cmd_data)
        position = self._return_get('2A03pos')

        if coord in {1, 2, 3}:
            position = [round(angle, 4) for angle in np.delete(position, 6).tolist()]
            self.logger.debug(f"当前位置: {position}")
        else:
            position = [round(angle, 4) for angle in position]
            self.logger.debug(f"当前位置: {position}")
        return position

    def directmotion_mode_set(self, open: bool, robot: int=1) -> bool:
        """
        开启/关闭socket直接控制运动模式
        (开启后会进去特殊的运行模式, 关闭后需要手动切回示教模式)

        Args:
            robot(int): 机器人号码
            open(bool): 
                - True:  开启socket直接控制运动模式
                - False: 关闭socket直接控制运动模式

        Returns:
            bool: 开启/关闭状态
                - True:  socket直接控制运动模式开启
                - False: socket直接控制运动模式关闭
        """
        if not isinstance(open, bool):
            self.logger.warning(f"实际收到: {open}, 类型: {type(open)}, open 必须是布尔类型")
            return
        
        cmd_data = {"robot":robot, "open":open}
        self._send_command(self.CONSTANTS['DIRECTMOTION']['MODE_SET'], cmd_data)
        open_return = self._return_get('50B3open')
        open_map = {
            True: "开启",
            False: "关闭"
        }
        self.logger.info(f"直接控制运动模式: {open_map.get(open_return, '未知状态')}")
        time.sleep(1)
        return open_return

    def directmotion_mode_inquire(self, robot: int=1) -> bool:
        """
        查询socket直接控制运动模式

        Args:
            None

        Returns:
            bool: 开启/关闭状态
                - True:  socket直接控制运动模式开启
                - False: socket直接控制运动模式关闭
        """
        self._send_command(self.CONSTANTS['DIRECTMOTION']['MODE_INQUIRE'], {"robot":robot})
        open_status = self._return_get('50B3open')
        open_map = {
            True: "开启",
            False: "关闭"
        }
        self.logger.info(f"查询 直接控制运动模式: {open_map.get(open_status, '未知状态')}")
        return open_status

    def directmotion_insert_instrvec(self, trajectory: List[float], 
                                     acc: int =100, 
                                     dec: int=100, 
                                     pl: int=4, 
                                     velocity: int=100, 
                                     imovecoord: str ="RF", 
                                     move_type:int =1) -> None:
        """
        发送指令队列控制机械臂移动, 采用自动点位

        Args:
            ParaACC(int): 加速度  1-100    
            ParaDEC(int): 减速度  1-100   
            ParaPL(int): 平滑系数  0-5      不需要平滑填写0
            ParaSPIN(int): 圆弧和整圆指令, 0=姿态不变 1=六轴不转 2=六轴旋转  不需要填写0
            ParaTIME(int): 提前跳出该点往下执行的时间设置  不需要填写0
            ParaV(int): 速度1-100 这个是全局速度的百分比
            m_vUnit(int): 速度单位: 0 表示 cm/s, 1 表示 mm/s, 2 表示 百分比, 注意: 关节坐标填写2, 直角等坐标填写 0 或者 1 建议填写1

            data(List[float]): [0.0,0.0,0.0,0.0,0.0,0.0,0.0,%s,%s,%s,%s,%s,%s,%s,0.0,0.0,0.0,0.0,0.0,0.0,0.0 ]
                - 第 1、2 位表示坐标, 0 0 : 表示关节坐标(其中第二位角度-0 、弧度-1), 1 1: 表示直角坐标, 2 1: 工具坐标, 3 1: 用户坐标
                - 第 3 位 左右手 1-左 2-右 0-无左右手 默认为 0
                - 第 4, 5, 6, 7 位备用, 默认为 0
                - 第 8 至 14 位保存机器人本体 坐标值(7 位)
                    - 关节坐标下, 分别表示 1 到 7 轴的角度值
                    - 其他坐标下, 分别表示 x,y,z,a,b,c 六个轴的坐标
                    (按顺序填写 后面无值默认为 0.0 示例中,1.0,2.0,3.0,4.0,5.0,6.0 为关节坐标值)
                - 第 15 至 19 位 保存外部轴坐标值（最大支持五个外部轴, 外部轴只有关节值, 不足五个外部轴后关节坐标值补零）

            imovecoord(str): 
                - "RF": 关节坐标
                - "BF": 直角坐标
                - "TF": 工具坐标
                - "UF": 用户坐标
            move_type(int): 
                - 1: 点到
                - 2: 直线
                - 3: 圆弧
                - 4: 整圆
        
        Returns:
            None
        """
        motion_queue = []

        DATA_LEN = 21
        POS_START_IDX = 7
        dof = self.dof

        for pos in trajectory:
            if len(pos) != dof:
                raise ValueError(f"pos 长度应为 {dof}, 当前为 {len(pos)}")

            data = [0.0] * DATA_LEN
            data[POS_START_IDX:POS_START_IDX + dof] = pos

            para_var_data = [
                {"data": 0.0, "secondvalue": 0, "value": 0, "varname": ""}
                for _ in range(DATA_LEN - 1)
            ]
            for i in range(dof):
                para_var_data[POS_START_IDX + i]["data"] = pos[i]

            q = {
                "ParaACC":  {"data": acc, "secondvalue": 0, "value": 0, "varname": ""},
                "ParaDEC":  {"data": dec, "secondvalue": 0, "value": 0, "varname": ""},
                "ParaPL":   {"data": pl, "secondvalue": 0, "value": 0, "varname": ""},
                "ParaSPIN": {"data": 0.0, "secondvalue": 0, "value": 0, "varname": ""},
                "ParaSYNC": {"data": 0.0, "secondvalue": 0, "value": 0, "varname": ""},
                "ParaTIME": {"data": 0.0, "secondvalue": 0, "value": 0, "varname": ""},
                "ParaV":    {"data": velocity, "m_vUnit": 2, "secondvalue": 0, "value": 0, "varname": ""},

                "RobotPos": {
                    "ctype": 1,
                    "data": data,
                    "key": "",
                    "paraVarData": para_var_data
                },

                "ctype": 0,
                "imovecoord": imovecoord,
                "length": 0.0,
                "logout": False,
                "margin": 0.0,
                "offsetAxis": 0,
                "para": 0,
                "polish": 0,
                "polishAngle": 0.0,
                "polishID": 1,
                "posidname": "",
                "posidtype": 0,
                "positionId": "",
                "radius": 0.0,
                "side": 0.0,
                "type": move_type,
                "userParamInt": 0,
                "userParamString": "",
                "width": 0.0
            }

            motion_queue.append(q)

        self._send_command(
            self.CONSTANTS['DIRECTMOTION']['INSERT_INSTRVEC'],
            {"data": motion_queue, "robot": 1}
        )

        self.logger.info(f"已发送队列 {len(motion_queue)} 组")
        self.logger.info(f"队列内容: {trajectory}")
        return None
        
    def directmotion_mode_suspend(self, robot: int = 1) -> None:
        """
        暂停追加队列运行

        Args:
            robot(int): 机器人号码
        
        Returns:
            None
        """
        self._send_command(self.CONSTANTS['DIRECTMOTION']['MODE_SUSPEND'], {"robot":robot})
        self.logger.info(f'暂停 追加队列运行')
        time.sleep(0.5)

    def directmotion_mode_start(self, robot: int = 1) -> None:
        """
        开始追加队列运行（暂停后, 发送使用）

        Args:
            robot(int): 机器人号码
        
        Returns:
            None
        """
        self._send_command(self.CONSTANTS['DIRECTMOTION']['MODE_START'], {"robot":robot})
        self.logger.info(f'开始 追加队列运行')
        time.sleep(0.5)

    def directmotion_mode_stop(self, robot: int = 1) -> None:
        """
        停止追加队列运行

        Args:
            robot(int): 机器人号码
        
        Returns:
            None
        """
        self._send_command(self.CONSTANTS['DIRECTMOTION']['MODE_STOP'], {"robot":robot})
        self.logger.info(f'停止 追加队列运行')
        time.sleep(0.5)

    def movj(self, vel: int, 
             coord: int, 
             pos: List[float], 
             robot: int=1) -> None:
        """
        机器人关节运动MOVJ
       
        Args:
            robot(int): 机器人号码
            vel(int): 速度百分⽐,1-100的整数
            coord(int): 坐标模式
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            pos(List[float]):[1.1,2.2,3.3,4.4,5.5,6.6,7.7]
                - 关节坐标分别代表1-7关节角度值
                - 直角坐标系分别x,y,z,a,b,c其中欧拉角的旋转顺序为'zyx',
                  工具、用户同直角直角坐标系下第七位参数默认为0即可
        
        Returns:
            None
        """
        if coord not in {-1, 0, 1, 2, 3}:
            self.logger.warning(f"坐标参数应在0(关节坐标)、1(直角坐标)、2(工具坐标)、3(用户坐标)之一, 当前为: {coord}")
            return
        
        if vel > 100 or vel < 1:
            self.logger.warning(f"速度百分比应在 1 ~ 100 之间, 当前为: {vel}")
            return

        if coord in {1, 2, 3}:  # 直角 / 工具 / 用户坐标
            self.logger.info(f"已发送移动的直角坐标(movj)")
            self.logger.info(f"XYZ(mm): {pos[:3]}, ABC(rad): {pos[3:]}\n")
        else:  # coord == 0（关节坐标）
            self.logger.info(f"已发送移动的关节角度(movj)")
            self.logger.info(f"移动的关节角度(deg): {pos}\n")

        cmd_data = {"robot": robot, "vel": vel, "coord": coord, "pos": pos}
        self._send_command(self.CONSTANTS['ROBOT_MOVEMENT']['JOINT'], cmd_data)

    def movl(self, vel: int, 
             coord: int, 
             pos: List[float], 
             robot: int=1) -> None:
        """
        机器人直线运动MOVL

        Args:
            robot(int): 机器人号码
            vel(int): 速度,单位mm/s,1以上的整数, 2~1000 整数
            coord(int): 坐标模式
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            pos(List[float]):[1.1,2.2,3.3,4.4,5.5,6.6,7.7]
                - 关节坐标分别代表1-7关节角度值
                - 直角坐标系分别x,y,z,a,b,c其中欧拉角的旋转顺序为'zyx',
                  工具、用户同直角直角坐标系下第七位参数默认为0即可
        
        Returns:
            None
        """
        if coord not in {-1, 0, 1, 2, 3}:
            self.logger.warning(f"坐标参数应在0(关节坐标)、1(直角坐标)、2(工具坐标)、3(用户坐标)之一, 当前为: {coord}")
            return
        
        if vel > 1000 or vel < 2:
            self.logger.warning(f"速度百分比应在 2 ~ 1000 之间, 当前为: {vel}")
            return
        
        if coord in {1, 2, 3}:
            self.logger.info(f"已发送移动的直角坐标(movl)")
            self.logger.info(f"XYZ(mm): {pos[:3]}, ABC(rad): {pos[3:]}\n")
        else: 
            self.logger.info(f"已发送移动的关节角度(movl)")
            self.logger.info(f"移动的关节角度(deg): {pos}\n")
        
        cmd_data = {"robot":robot, "vel":vel, "coord":coord, "pos":pos}
        self._send_command(self.CONSTANTS['ROBOT_MOVEMENT']['LINEAR'], cmd_data)
    
    def movc(self, vel: int, 
             coord: int, 
             isFull:bool, 
             posOne: List[float], 
             posTwo: List[float], 
             posThree: List[float], 
             robot: int=1) -> None:
        """
        机器人圆弧运动MOVC
        
        Args:
            robot(int): 机器人号码
            vel(int): 速度,单位 mm/s,1以上的整数, 2~1000 整数
            coord(int): 坐标模式
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            isFull(bool): 
                - False: MOVC
                - True: MOVCA(整圆)
            posOne(List[float]): 圆弧起始点 
            posTwo(List[float]): 圆弧经过的中间点
            posThree(List[float]): 圆弧的目标点
                - 关节坐标分别代表1-7关节角度值
                - 直角坐标系分别x,y,z,a,b,c其中欧拉角的旋转顺序为'zyx',
                  工具、用户同直角直角坐标系下第七位参数默认为0即可
        
        Returns:
            None
        """
        if coord not in {-1, 0, 1, 2, 3}:
            self.logger.warning(f"坐标参数应在0(关节坐标)、1(直角坐标)、2(工具坐标)、3(用户坐标)之一, 当前为: {coord}")
            return

        if vel > 1000 or vel < 2:
            self.logger.warning(f"速度百分比应在 2~1000 之间, 当前为: {vel}")
            return

        if coord in {1, 2, 3}: 
            self.logger.info(f"已发送直角圆弧运动(movc)")
            if isFull == True:
                self.logger.info(f"> 当前为整圆")
            elif isFull == False:
                self.logger.info(f"> 当前为圆弧")
            else:
                self.logger.warning(f"isFull为布尔类型, True为整圆, False为圆弧, 当前为: {isFull}")
                return
            self.logger.info(f"> 起点: XYZ(mm): {posOne[:3]}, ABC(rad): {posOne[3:]}")
            self.logger.info(f"> 中间点: XYZ(mm): {posTwo[:3]}, ABC(rad): {posTwo[3:]}")
            self.logger.info(f"> 终点: XYZ(mm): {posThree[:3]}, ABC(rad): {posThree[3:]}")

        else:  
            self.logger.info(f"已发送关节圆弧运动(movc)")
            if isFull == True:
                self.logger.info(f"> 当前为整圆")
            elif isFull == False:
                self.logger.info(f"> 当前为圆弧")
            else:
                self.logger.warning(f"isFull为布尔类型, True为整圆, False为圆弧, 当前为: {isFull}")
                return
            
            self.logger.info(f"> 起点(deg): {posOne}")
            self.logger.info(f"> 中间点(deg): {posTwo}")
            self.logger.info(f"> 终点(deg): {posThree}")

        cmd_data = {
            "robot": robot,
            "vel": vel,
            "coord": coord,
            "isFull": isFull,
            "posOne": posOne,
            "posTwo": posTwo,
            "posThree": posThree
        }
        self._send_command(self.CONSTANTS['ROBOT_MOVEMENT']['CIRULAR'], cmd_data)

    def movs(self, vel: int, 
             coord: int, 
             size: int, 
             pos: List[List[float]], 
             robot: int=1) -> None:
        """
        机器人样条曲线运动MOVS

        Args:
            robot(int): 机器人号码
            vel(int): 速度,单位mm/s,1以上的整数, 2~1000 整数
            coord(int): 坐标模式
                - 0:  关节坐标(Joint)
                - 1:  直角坐标(Cart)
                - 2:  工具坐标(Tool)
                - 3:  用户坐标(User)
            size(int): 样条曲线的点数目, 要求至少4个点位
            pos(List[float]): 样条曲线的轨迹点位
                            [[1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7],  
                            [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7], 
                            [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7], 
                            [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7]]
                - 关节坐标分别代表1-7关节角度值
                - 直角坐标系分别x,y,z,a,b,c其中欧拉角的旋转顺序为'zyx',
                  工具、用户同直角直角坐标系下第七位参数默认为0即可
        
        Returns:
            None
        """
        if coord not in {-1, 0, 1, 2, 3}:
            self.logger.warning(f"坐标参数应在0(关节坐标)、1(直角坐标)、2(工具坐标)、3(用户坐标)之一, 当前为: {coord}")
            return

        if size != len(pos):
            self.logger.warning(f"样条轨迹点数量 size={size} 与 pos 中实际点数 {len(pos)} 不一致")
            return

        if vel > 1000 or vel < 2:
            self.logger.warning(f"速度百分比应在 2~1000 之间, 当前为: {vel}")
            return

        if coord in {1, 2, 3}:  
            self.logger.info("已发送直角样条轨迹(movs)")
            self.logger.info(f'轨迹点位:')
            for idx, p in enumerate(pos):
                self.logger.info(f"[{idx+1}] XYZ(mm): {p[:3]}, ABC(rad): {p[3:]}")
        else:
            self.logger.info("已发送关节样条轨迹(movs)")
            self.logger.info(f'关节角度(deg)点位:')
            for idx, p in enumerate(pos):
                self.logger.info(f"[{idx+1}]: {p}")

        cmd_data = {
            "robot": robot,
            "vel": vel,
            "coord": coord,
            "size": size,
            "pos": pos
        }

        self._send_command(self.CONSTANTS['ROBOT_MOVEMENT']['SPLINE'], cmd_data)

    def jog_operation_move(self, axis: int, direction: int) -> None:
        """
        执行点动操作
        
        Args:
            axis(int): 代表所要操作的轴, 如 1 代表轴1, 外部轴从8开始
            direction(int): 关节移动方向
                - 1: 正向
                - -1: 反向
        
        Returns:
            None
        """
        if not (1 <= axis <= self.dof):
            self.logger.warning(f"轴号应为 1 ~ {self.dof} 的关节号, 当前为: {axis}")
            return
        
        if direction not in [1, -1]:
            self.logger.warning(f"关节移动方向应为1(正向)或者-1(反向), 当前为: {direction}")
            return
              
        cmd_data = {"axis":axis, "direction":direction}
        self._send_command(self.CONSTANTS['JOG_OPERATION']['MOVE'], cmd_data)
        self.logger.info(f"开始点动操作")

    def jog_operation_stop(self, axis: int) -> None:
        """
        停止执行点动操作

        Args:
            axis(int): 代表所要操作的轴, 如 1 代表轴1, 外部轴从8开始
        
        Returns:
            None
        """
        if not (1 <= axis <= self.dof):
            self.logger.warning(f"轴号应为 1 ~ {self.dof} 的关节号, 当前为: {axis}")
            return
        
        self._send_command(self.CONSTANTS['JOG_OPERATION']['STOP'], {"axis":axis})
        self.logger.info(f"停止点动操作")
    
    def jog_jointparameter_set(self, AxisNum: int, MaxSpeed: int, MaxAcc: int) -> None:
        """
        设置关节轴点动参数

        Args:
            AxisNum(int): 表示设置的关节轴
            MaxSpeed(int): 关节轴最大点动速度 单位: °/s
            MaxAcc(int): 关节轴点动加速度 单位: °/s²

        Returns:
            None
        """
        if not (1 <= AxisNum <= self.dof):
            self.logger.warning(f"轴号应为 1 ~ {self.dof} 的关节号, 当前为: {AxisNum}")
            return
        
        cmd_data = {"AxisNum":AxisNum,"MaxSpeed":MaxSpeed,"MaxAcc":MaxAcc}
        self._send_command(self.CONSTANTS['JOG_JOINTPARAMETER']['SET'], cmd_data)
        time.sleep(0.5)
        self.logger.info(f"设置关节轴 {AxisNum} 的最大点动速度为{MaxSpeed} °/s, 点动加速度{MaxAcc} °/s²")
    
    def jog_jointparameter_inquire(self, AxisNum: int) -> dict:
        """
        查询关节轴点动参数

        Args:
            AxisNum(int): 表示需要查询的关节轴
        
        Returns:
            dict: 关节轴点动参数, 包括以下字段: 
                - AxisNum: int, 当前查询关节点动参数的轴号
                - MaxSpeed: float, 关节轴最大点动速度, 单位 °/s
                - MaxAcc: float, 关节轴点动加速度, 单位 °/s²
        """
        if not (1 <= AxisNum <= self.dof):
            self.logger.warning(f"轴号应为 1 ~ {self.dof} 的关节号, 当前为: {AxisNum}")
            return

        self._send_command(self.CONSTANTS['JOG_JOINTPARAMETER']['INQUIRE'], {"AxisNum":AxisNum})
        self.logger.info(f'查询 关节轴点动参数')
        AxisNum = self._return_get('2606AxisNum')
        self.logger.info(f"> 关节轴编号为: {AxisNum}")
        MaxSpeed = self._return_get('2606MaxSpeed')
        self.logger.info(f"> 关节轴最大点动速度为: {MaxSpeed} °/s")
        MaxAcc = self._return_get('2606MaxAcc')
        self.logger.info(f"> 关节轴点动加速度为: {MaxAcc} °/s²")
        result = {
            "AxisNum": AxisNum,
            "MaxSpeed": MaxSpeed,
            "MaxAcc": MaxAcc
        }
        return result

    def jog_rectparameter_set(self, MaxSpeed: int, MaxAcc:int) -> None:
        """
        设置直角坐标点动参数

        Args:
            MaxSpeed(int): 直角坐标下的点动最大速度, 单位 mm/s
            MaxAcc(int): 直角坐标下的点动最大加速度, 单位 mm/s²

        Returns:
            None
        """
        cmd_data = {"MaxSpeed":MaxSpeed,"MaxAcc":MaxAcc}
        self._send_command(self.CONSTANTS['JOG_RECTPARAMETER']['SET'], cmd_data)
        self.logger.info(f'设置直角坐标点动参数')
        self.logger.info(f"> 设置直角坐标下的点动最大速度为: {MaxSpeed} mm/s")
        self.logger.info(f"> 设置直角坐标下的点动最大加速度为:{MaxAcc} mm/s²")

    def jog_rectparameter_inqure(self) -> dict:
        """
        查询直角坐标点动参数

        Args:
            None

        Returns:
            dict: 直角坐标点动参数, 包括以下字段: 
                - MaxSpeed(float): 直角坐标下的点动最大速度, 单位 mm/s
                - MaxAcc(float): 直角坐标下的点动最大加速度, 单位 mm/s²
        """
        self._send_command(self.CONSTANTS['JOG_RECTPARAMETER']['INQUIRE'])
        time.sleep(0.1)
        MaxAcc = self._return_get('2609MaxAcc')
        MaxSpeed = self._return_get('2609MaxSpeed')
        self.logger.info(f'查询直角坐标点动参数')
        self.logger.info(f"> 直角坐标下点动最大速度为: {MaxSpeed} mm/s")
        self.logger.info(f"> 直角坐标下点动最大加速度为: {MaxAcc} mm/s²")
        result = {"MaxAcc":MaxAcc, "MaxSpeed":MaxSpeed}
        return result

    def jog_sensitivity_set(self, Sensitivity: float) -> None:
        """
        设置点动灵敏度(默认 0.001)

        Args:
            Sensitivity(float): 点动灵敏度, 单位 度, 范围 0.001 - 1
        
        Returns:
            None
        """
        self._send_command(self.CONSTANTS['JOG_SENSITIVITY']['SET'], {"Sensitivity":Sensitivity})
        time.sleep(0.5)
        self.logger.info(f"设置点动灵敏度为: {Sensitivity} °")
    
    def jog_sensitivity_inquire(self) -> float:
        """
        查询点动灵敏度

        Args:
            None
        
        Returns:
            float: 当前点动灵敏度, 单位 度, 范围 0.001 - 1
        """
        self._send_command(self.CONSTANTS['JOG_SENSITIVITY']['INQUIRE'])
        Sensitivity = self.return_status.get('260CSensitivity', 0)
        self.logger.info(f"点动灵敏度为: {Sensitivity} 度")
        return Sensitivity

    def pos_trans_coord(self, pos: List[float], currentCoord: int, targetCoord: int, robot: int=1) -> List[float]:
        """
        位置点坐标系转换

        Args:
            robot(int): 机器人号码
            pos(List[float]): 
                - 关节坐标: 代表关节角度, pos 长度需要对应 self.dof, 单位为度
                - 直角坐标、工具坐标、用户坐标: pos长度为 6, 前三位为位置(xyz), 单位为毫米；后三位为姿态(ABC), 单位为度
            currentCoord(int): 当前 pos 对应的坐标系
                - 0: 关节坐标
                - 1: 直角坐标
                - 2: 工具坐标
                - 3: 用户坐标
            targetCoord(int): 目标坐标系
                - 0: 关节坐标
                - 1: 直角坐标
                - 2: 工具坐标
                - 3: 用户坐标
            (currentCoord为0, targetCoord为1, 表示正向运动学解算；
            currentCoord为1, targetCoord为0, 表示逆向运动学解算；
            其中对于逆向运动学解算的情况, 参考位姿为当前的实际位姿)
            
        Returns: 
            List[float]: 目标坐标系下的pos
                - 关节坐标: 代表关节角度, pos 长度为 self.dof, 单位为度
                - 直角坐标、工具坐标、用户坐标: pos长度为 6, 前三位为位置(xyz), 单位为毫米；后三位为姿态(ABC), 单位为度
        """
        pos_send = [0.0] * 14
        if currentCoord == 0:
            pos_send[0] = 0
            pos_send[1] = 0
        elif currentCoord == 1:
            pos_send[0] = 1
            pos_send[1] = 1
        elif currentCoord == 2:
            pos_send[0] = 2
            pos_send[1] = 1
        elif currentCoord == 3:
            pos_send[0] = 3
            pos_send[1] = 1
        else:
            raise ValueError("targetCoord 非法")
        
        if currentCoord == 0 and self.dof == 7:
            if len(pos) != 7:
                raise ValueError("pos 长度必须和当前的 7 自由度对应")
            pos_send[7:14] = pos
        else:
            if len(pos) < 6:
                raise ValueError("pos 长度不得小于 6")
            pos_send[7:13] = pos[:6]
            pos_send[13] = 0.0
        
        cmd_data = {
            "robot": robot,
            "pos": pos_send,
            "targetCoord": targetCoord
        }
        self._send_command(self.CONSTANTS['POS_TRANS_COORD'], cmd_data)
        pos_trans_raw = self.return_status.get('2A13pos')
        success_flag = self.return_status.get('2A13result')

        if success_flag == False:
            data = self.return_status.get('2B03data')
            self.logger.warning(data)
            return None
        if success_flag == True:
            if targetCoord == 0 and self.dof == 7:
                pos_trans = pos_trans_raw[7:14]
            else:
                pos_trans = pos_trans_raw[7:13]
            return pos_trans

    ########################################
    # API接口（7000端口）
    ########################################
    def motion_control(self, trajectory: List[List[float]], 
                       coord: str="ACS",
                       speed: int=100, 
                       acc: int=70,
                       pl: int=5,
                       moveMode: str='TeachSet',
                       robot: int=1) -> bool:
        """
        运动控制
        这个控制端口一定要记得在控制器上建立作业文件, 通过6001端口运行该文件之后才可使用
        建立作业文件步骤: 
            1、在示教器界面上点击左侧“工程”, 点击“新建”
            2、在“程序名称”输入框中输入作业文件名称, 例如“tlibot”, 点击“确定”
            3、在“工程预览/程序指令”界面点击左下角的“插入”
            4、在指令类型中选择“条件控制类”, 然后在右边“指令”中选择“循环”, 点击“确定”
            5、在“工程预览/程序指令/指令插入/参数设定”界面, 点击“条件未设定”后点击“确定”
            6、界面回到了“工程预览/程序指令/指令插入/参数设定”界面后再点击“确定”
            7、在“工程预览/程序指令”界面继续点击左下角的“插入”
            8、在指令类型中选择“运动控制类”, 然后在右边“指令”中选择“外部点”, 点击“确定”
            9、在“工程预览/程序指令/指令插入/参数设定”界面, 修改合适参数(可以直接默认, 不做修改), 点击“确定”

        Args:
            target_vecs(List[List[float]]): 
                - 轨迹运动: [[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
                            [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
                            ......]
                - 关节运动: [[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7]]
            coord(str):
                - ACS: 关节坐标
                - MCS: 直角坐标
                  需要注意的是, 如果是直角坐标, 发送的数据为6个元素
                  比如：需要在直角坐标下移动到的点位坐标为 X、Y、Z 为 279.852 mm、0 mm、409.237 mm, A、B、C(单位弧度)为 -3.141、0、0 
                  trajectory = [[279.852, 0, 409.237, -3.141, 0, 0]]
                  轨迹为 trajectory = [[279.852, 0, 409.237, -3.141, 0, 0],
                                    [379.852, 0, 409.237, -3.141, 0, 0],
                                    [179.852, 0, 409.237, -3.141, 0, 0], 
                                    [279.852, 0, 409.237, -3.141, 0, 0]]
            speed(int): 运动速度, 范围: 1-100
            acc(int): 运动加速度, 范围: 1-100
            pl(int): 平滑系数, 若不写, 则使用默认值5, 范围: 1-5
            moveMode(str): 运动模式
                - 可选"TeachSet","MOVJ","MOVL","MOVS", 默认值"TeachSet"
            robot(int): 机器人号码
        
        Returns:
            bool: 运动控制是否成功
                - True: 成功
                - False: 失败
        """
        cmd_data = { 
            "robot": robot, 
            "clearBuffer": 1, 
            "targetMode": 0, 
            "cfg": { 
                "coord": coord, 
                "speed": speed, 
                "acc": acc, 
                "pl": pl, 
                "moveMode": moveMode}, 
            "targetVec": [ {"pos": pos} for pos in trajectory ]  
            }
        
        self._send_command(self.CONSTANTS['MULTI_POINT'], cmd_data)
        success = self._return_get('9523success')
        if success:
            self.logger.info(f"已发送7000端口运动控制指令(motion_control):")
            if coord == "ACS":
                self.logger.info(f"关节坐标 移动的路径如下:")
            elif coord == "MCS":
                self.logger.info(f"直角坐标 移动的路径如下:")
            self.logger.info(f"{trajectory}")
            return True
        else:
            cause = self._return_get('9523cause')
            if cause == "busy":
                self.logger.error(f"当前有未传输完成的数据")
            elif cause == "timeout":
                self.logger.error(f"接收超时")
            elif cause == "dataErr":
                self.logger.error(f"数据错误")
            elif cause == "termination":
                self.logger.error(f"发送端终止了正在传输的数据")
            return False
        
    def set_servo_point_motion_control(self, switch: bool, robot: int=1):
        """
        开关伺服点位运动控制
        (该功能和"运动控制 0x9521"功能不可同时使用)

        Args:
            robot(int): 机器人号码
            switch(bool): 开关控制命令(注意这个是字符串的true和false)
                - True: 开启伺服点位运动控制
                - False: 关闭伺服点位运动控制
        
        Returns:
            bool: 开关伺服点位运动控制是否成功
                - True: 开启成功
                - False: 开启失败
        """
        if not isinstance(switch, bool):
            self.logger.warning(f"实际收到: {switch}, 类型: {type(switch)}, switch 必须是布尔类型")
            return
        
        cmd_data = {"robot":robot, "switch":switch}
        self._send_command(self.CONSTANTS['SERVOCONTROL']['OPEN'], cmd_data)
        cause = self._return_get('95A3cause')
        cause_map = {# 接收成功时为空
            "dataErr": "接收到的数据错误", 
            "startupErr": "启动失败",
            "busy": "当前通道被占用"
        }
        switch_map = {
            True: "开启",
            False: "关闭"
        }
        self.logger.info(f"开关伺服点位运动控制状态设置: {cause_map.get(cause, '接收成功')}; 状态: {switch_map.get(switch, '未知')}")
        time.sleep(0.5)
        return cause

    def servo_point_motion_control(self, end: int, 
                                   sum: int, 
                                   count: int, 
                                   PosVec: List[List[float]], 
                                   robot: int=1):
        """
        伺服运动控制

        Args:
            robot(int): 机器人号码
            end(int):
                - 1: 停止之前的持续传输,下面的数据可不发
                - 0: 可不发 end 参数值
            sum(int): 总共要发的帧数
            count(int): 当前为第几帧
            PosVec(List[List[float]]): 机器人的关节角度
                - [[1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
                    [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
                    ......]

        Returns:
            tuple[str, bool]:
                - str:  
                    - "": 接收成功
                    - "notStart": 未开启服点位运动控制模式
                    - "dataErr": 数据错误
                    - "termination": 发送端终止了正在传输的数据
                    - "cacheFull": 缓存区已满(最大缓存 6 条轨迹)
                - bool: 判断是否接收成功, 用于后续判断
                    - True: 成功
                    - False: 失败
        """
        cmd_data = {
            "robot":robot, 
            "end":end, 
            "sum":sum, 
            "count":count, 
            "PosVec":PosVec
            }
        
        self._send_command(self.CONSTANTS['SERVOCONTROL']['MOVE'], cmd_data)
        cause = self._return_get('95A6cause')
        if cause == "notStart":
            self.logger.info(f'未开启服点位运动控制模式')
            return cause, False
        elif cause == "dataErr":
            self.logger.info(f'数据错误')
            return cause, False
        elif cause == "termination":
            self.logger.info(f'发送端终止了正在传输的数据')
            return cause, False
        elif cause == "cacheFull":
            self.logger.info(f'缓存区已满(最大缓存 6 条轨迹)')
            return cause, False
        elif cause == None:
            return cause, True

    def open_servo_j(self, vmax: list[float], 
                     amax: list[float], 
                     jmax: list[float], 
                     robot: int=1):
        """
        打开关节跟踪模式

        Args:
            robot(int): 机器人号码
            vmax(list[float]): 速度约束,  单位: 度/秒
            amax(list[float]): 加速度约束,  单位: 度/秒^2
            jmax(list[float]): 加加速度约束,  单位: 度/秒^3

        Returns:
            None
        """
        self._send_command(self.CONSTANTS['SERVO_J_CONTROL']['OPEN'], 
                           {"robot":robot, "vmax":vmax, "amax":amax, "jmax":jmax})
        self.logger.info(f"打开关节跟踪模式")
        self.logger.info(f"> 速度约束: {vmax} °/s")
        self.logger.info(f"> 加速度约束: {amax} °/s²")
        self.logger.info(f"> 加加速度约束: {jmax} °/s³")
        return None
    
    def set_servo_j(self, q: List[float], robot: int=1) -> None:
        """
        发送跟踪关节位置

        Args:
            robot(int): 机器人号码
            PosVec(List[float]): 机器人的关节角度

        Returns:
            None
        """
        self._send_command(self.CONSTANTS['SERVO_J_CONTROL']['MOVE'], {"robot":robot, "q":q})
        self.logger.debug(f"关节跟踪模式")
        self.logger.debug(f"> 机器人的关节角度: {q}")
        return None
    
    def stop_servo_j(self, robot: int=1) -> None:
        """
        停止跟踪关节位置

        Args:
            robot(int): 机器人号码

        Returns:
            None
        """
        self._send_command(self.CONSTANTS['SERVO_J_CONTROL']['STOP'], {"robot":robot})
        self.logger.info(f"停止关节跟踪模式")
        return None
