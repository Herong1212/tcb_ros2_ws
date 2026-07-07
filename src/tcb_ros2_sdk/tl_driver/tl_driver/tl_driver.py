import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
import math
import time
import traceback
from typing import Dict, List, Optional

from scipy.spatial.transform import Rotation as R
import threading
import queue


def _load_sdk(version: int):
    if version == 2403:
        from sdk.TCB_SDK_2403.robotarm_sdk import RobotArmSDK
        from sdk.TCB_SDK_2403.utils.robotarm_function import RobotArmFunction
    elif version == 2207:
        from sdk.TCB_SDK_2207.robotarm_sdk import RobotArmSDK
        from sdk.TCB_SDK_2207.utils.robotarm_function import RobotArmFunction
    else:
        raise ValueError(f"Unsupported sdk version: {version}")
    return RobotArmSDK, RobotArmFunction


class ArmDriver:
    """单个机械臂的运行时控制对象（各自线程/缓冲区/到位判断）"""

    def __init__(
        self,
        node: Node,
        *,
        arm_key: str,  # 'armleft' / 'armright'
        joint_prefix: str,  # '' / 'left_' / 'right_'
        ip: str,
        port_6001: int,
        port_7000: int,
        sdk_version: int,
        dof: int,
        trajectory_timeout: float,
        stride: int,
        arm_control_mode: str,
        speed: float,
        position_tolerance: float,  # radians
        velocity_tolerance: float,  # radians/s
        settle_time: float,
        servo_j_frequency: float,
        log_level: int,
        is_dual: bool,
    ):
        self.node = node
        self.logger = node.get_logger()
        self.arm_key = arm_key
        self.joint_prefix = joint_prefix
        self.ip = ip
        self.port_6001 = port_6001
        self.port_7000 = port_7000
        self.sdk_version = int(sdk_version)
        self.dof = int(dof)
        self.trajectory_timeout = float(trajectory_timeout)
        self.stride = int(stride) if int(stride) > 0 else 1
        self.arm_control_mode = arm_control_mode
        self.speed = int(speed)
        self.position_tolerance = position_tolerance
        self.velocity_tolerance = velocity_tolerance
        self.settle_time = settle_time
        self.servo_j_frequency = servo_j_frequency

        RobotArmSDK, RobotArmFunction = _load_sdk(self.sdk_version)

        self.sdk_6001 = RobotArmSDK(
            self.ip, self.port_6001, log_level=log_level, color_log=False
        )
        self.sdk_7000 = RobotArmSDK(
            self.ip, self.port_7000, log_level=log_level, color_log=False
        )
        self.robotfuc = RobotArmFunction(color_log=False)

        self.sdk_6001.connect()
        self.sdk_7000.connect()

        self.joint_names: List[str] = [
            f"{self.joint_prefix}tl_robot_joint{i+1}" for i in range(self.dof)
        ]

        self.motion_complete_publisher: Optional[rclpy.publisher.Publisher] = None
        if is_dual:
            self.motion_complete_publisher = node.create_publisher(
                String, f"tl_driver/{self.arm_key}/motion_complete", 10
            )
            ready_msg = String()
            ready_msg.data = "ready"
            self.motion_complete_publisher.publish(ready_msg)

        end_pose_topic = (
            "/tl_driver/end_pose"
            if self.joint_prefix == ""
            else f"/tl_driver/{self.arm_key}/end_pose"
        )
        self.end_pose_publisher = node.create_publisher(PoseStamped, end_pose_topic, 10)

        if is_dual:
            speed_topic = f"tl_driver/{self.arm_key}/set_speed"
        else:
            speed_topic = "tl_driver/set_speed"

        self.speed_subscriber = node.create_subscription(
            Int32, speed_topic, self.speed_callback, 10
        )

        # 状态
        self.move_lock = threading.Lock()
        self.current_position: Optional[List[float]] = None
        self.current_velocity: Optional[List[float]] = None
        self.current_effort: Optional[List[float]] = None
        self.target_position: Optional[List[float]] = None

        self.trajectory_buffer: List[List[float]] = []
        self.buffer_lock = threading.Lock()
        self.last_trajectory_time = 0.0

        self.servo_j_queue: queue.Queue[List[float]] = queue.Queue()
        self.servo_j_last_send_time = 0.0
        self.servo_j_idle_timeout = 0.2
        self.servo_j_last_position: Optional[List[float]] = None
        self.servo_j_settled_since = 0.0
        self.servo_j_arrived = False
        self.servo_j_motion_active = False

        self.trajectory_condition = threading.Condition()
        self.is_moving = False
        self.last_motion_start_time = 0.0
        self.last_motion_end_time = 0.0
        self.settled_since = 0.0
        self.move_status = "ready"

        self.motion_start_time = 0.0  # 新增：最近一次运动开始时间戳
        self.motion_guard_time = 0.3  # 新增：保护时间，可参数化
        self.received_trajectory_length = 0

        self.running = True
        self.trajectory_thread = threading.Thread(
            target=self._trajectory_processor, daemon=True
        )
        self.trajectory_thread.start()

        self.servo_j_thread: Optional[threading.Thread] = None
        if self.arm_control_mode == "servo_j":
            self.servo_j_thread = threading.Thread(
                target=self._servo_j_processor, daemon=True
            )
            self.servo_j_thread.start()

        self.logger.info(
            f"{self.arm_key} initialized, ip={self.ip}, mode={self.arm_control_mode}"
        )

    def speed_callback(self, msg: Int32) -> None:
        try:
            new_speed = int(msg.data)

            # 验证速度值范围 (1-100)
            if new_speed < 1:
                self.logger.warn(
                    f"{self.arm_key} 设置速度失败：速度值 {new_speed} 必须大于0"
                )
                return
            if new_speed > 100:
                self.logger.warn(
                    f"{self.arm_key} 设置速度失败：速度值 {new_speed} 不能超过100"
                )
                return

            old_speed = self.speed
            self.speed = new_speed

            # 更新SDK中的速度设置（仅适用于queue和motion_control模式）
            if self.arm_control_mode in ["queue", "motion_control"]:
                try:
                    with self.move_lock:
                        self.sdk_6001.speed_set(self.speed)
                    self.logger.info(
                        f"{self.arm_key} 速度已从 {old_speed}% 更新为 {self.speed}%"
                    )
                except Exception as e:
                    self.logger.error(f"{self.arm_key} SDK速度设置失败: {e}")
                    self.speed = old_speed  # 恢复原速度
            elif self.arm_control_mode == "servo_j":
                # servo_j模式的速度参数在open_servo_j时设置，需要特殊处理
                self.logger.info(
                    f"{self.arm_key} servo_j模式速度已记录为 {self.speed}%，但需重新上电生效"
                )
            else:
                self.logger.warn(
                    f"{self.arm_key} 未知的控制模式 {self.arm_control_mode}，速度设置无效"
                )

        except ValueError as e:
            self.logger.error(f"{self.arm_key} 设置速度失败：无效的速度值 '{msg.data}'")
        except Exception as e:
            self.logger.error(f"{self.arm_key} 速度回调处理失败: {e}")
            traceback.print_exc()

    def _publish_motion(self, status: str) -> None:
        self.move_status = status
        if self.motion_complete_publisher is None:
            return
        msg = String()
        msg.data = status
        self.motion_complete_publisher.publish(msg)

    def enqueue_trajectory_points_deg(
        self, points_deg: List[List[float]], now: float
    ) -> None:
        with self.buffer_lock:
            self.last_trajectory_time = now
            self.trajectory_buffer.extend(points_deg)
        with self.trajectory_condition:
            self.trajectory_condition.notify_all()

    def enqueue_servo_point_deg(self, point_deg: List[float]) -> None:
        self.servo_j_queue.put(point_deg)

    def _trajectory_processor(self):
        while self.running:
            try:
                if self.arm_control_mode == "servo_j":
                    time.sleep(1.0 / self.servo_j_frequency)
                    continue

                with self.trajectory_condition:
                    self.trajectory_condition.wait(timeout=self.trajectory_timeout)

                current_time = time.time()
                points_to_send: List[List[float]] = []
                with self.buffer_lock:
                    time_since_last = current_time - self.last_trajectory_time
                    if (
                        self.trajectory_buffer
                        and time_since_last >= self.trajectory_timeout
                    ):
                        points_to_send = self.trajectory_buffer.copy()
                        self.trajectory_buffer.clear()

                if not points_to_send:
                    continue

                last_point = points_to_send[-1]
                self.target_position = [math.radians(p) for p in last_point]
                self.last_motion_start_time = current_time
                self.is_moving = True
                self.settled_since = 0.0

                self.received_trajectory_length = len(points_to_send)

                self.logger.info(
                    f"{self.arm_key} 轨迹结束, 发送 {self.received_trajectory_length} 个点, {points_to_send}"
                )
                ok = self._execute_points(points_to_send)
                if ok:
                    self.motion_start_time = current_time  # 记录运动开始时刻
                    self._publish_motion("moving")
                else:
                    # 下发失败则不进入 moving 状态
                    self.is_moving = False
                    self.target_position = None
                    self.settled_since = 0.0
                    self._publish_motion("ready")

            except Exception as e:
                self.logger.error(f"{self.arm_key} _trajectory_processor 错误: {e}")
                traceback.print_exc()

    def _servo_j_processor(self):
        send_interval = (
            1.0 / self.servo_j_frequency if self.servo_j_frequency > 0 else 0.01
        )
        while self.running:
            try:
                try:
                    next_point = self.servo_j_queue.get(timeout=send_interval)
                except queue.Empty:
                    continue

                self._execute_servo_j_point(next_point)
                self.servo_j_queue.task_done()

            except Exception as e:
                self.logger.error(f"{self.arm_key} _servo_j_processor 错误: {e}")
                traceback.print_exc()

    def _execute_servo_j_point(self, point_deg: List[float]) -> None:
        self._publish_motion("moving")
        try:
            with self.move_lock:
                self.sdk_7000.set_servo_j(point_deg)
            self.servo_j_last_send_time = time.time()
            self.servo_j_motion_active = True
            self.servo_j_arrived = False
            self.servo_j_settled_since = 0.0
        except Exception as e:
            self.logger.error(f"{self.arm_key} servo_j 发送失败: {e}")
            traceback.print_exc()

    def _execute_points(self, points_deg: List[List[float]]) -> bool:
        lock_acquired = False
        try:
            if not self.move_lock.acquire(blocking=True, timeout=0.5):
                self.logger.debug(
                    f"{self.arm_key} Previous motion running, skipping..."
                )
                return False
            lock_acquired = True

            stride = self.stride
            if stride > 1:
                sampled_points = points_deg[::stride]
                if points_deg[-1] not in sampled_points:
                    sampled_points.append(points_deg[-1])
            else:
                sampled_points = points_deg

            if self.arm_control_mode == "queue":
                self.sdk_6001.directmotion_insert_instrvec(
                    sampled_points, acc=100, dec=100, pl=5
                )
            elif self.arm_control_mode == "motion_control":
                self.sdk_7000.motion_control(sampled_points)
            else:
                self.logger.warn(
                    f"{self.arm_key} unsupported arm_control_mode={self.arm_control_mode}"
                )
                return False
            return True

        except Exception as e:
            self.logger.error(f"{self.arm_key} _execute_points 错误: {e}")
            traceback.print_exc()
            return False
        finally:
            if lock_acquired:
                time.sleep(0.05)
                self.move_lock.release()

    def power_on(self) -> None:
        try:
            queue_status = self.sdk_6001.directmotion_mode_inquire()
            self.robotfuc.robot_init_repeat(self.sdk_6001, self.speed)

            if self.arm_control_mode == "queue":
                if queue_status is False:
                    self.sdk_6001.directmotion_mode_set(True)
                self.sdk_6001.speed_set(self.speed)
            elif self.arm_control_mode == "motion_control":
                if queue_status is True:
                    self.sdk_6001.directmotion_mode_set(False)
                self.sdk_6001.stop_job_run()
                self.sdk_6001.jobsend_done("tlibot")
            elif self.arm_control_mode == "servo_j":
                vmax = [300] * self.dof
                amax = [3000] * self.dof
                jmax = [500000] * self.dof
                self.sdk_7000.open_servo_j(vmax=vmax, amax=amax, jmax=jmax)
            self.logger.info(f"{self.arm_key} 上电完成")
        except Exception as e:
            self.logger.error(f"{self.arm_key} 上电失败: {e}")
            traceback.print_exc()

    def power_off(self) -> None:
        try:
            if self.arm_control_mode == "servo_j":
                self.sdk_7000.stop_servo_j()
            self.robotfuc.robot_stop(self.sdk_6001)
            self.logger.info(f"{self.arm_key} 下电完成")
        except Exception as e:
            self.logger.error(f"{self.arm_key} 下电失败: {e}")
            traceback.print_exc()

    def matrix_to_xyz_and_quaternion(self, matrix):
        translation = matrix[:3, 3].tolist()
        rotation = matrix[:3, :3]
        r = R.from_matrix(rotation)
        quat = r.as_quat()  # 返回 [x, y, z, w] 格式的四元数
        # quat = np.roll(quat,1) # 返回 [w, x, y, z] 格式的四元数
        return translation, quat.tolist()

    def _publish_end_pose(self) -> None:
        if self.current_position_xyz is None or self.current_orientation_quat is None:
            return
        if (
            len(self.current_position_xyz) != 3
            or len(self.current_orientation_quat) != 4
        ):
            return
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.joint_prefix}tl_robot_link0"
        msg.pose.position.x = float(self.current_position_xyz[0])
        msg.pose.position.y = float(self.current_position_xyz[1])
        msg.pose.position.z = float(self.current_position_xyz[2])
        msg.pose.orientation.x = float(self.current_orientation_quat[0])
        msg.pose.orientation.y = float(self.current_orientation_quat[1])
        msg.pose.orientation.z = float(self.current_orientation_quat[2])
        msg.pose.orientation.w = float(self.current_orientation_quat[3])
        self.end_pose_publisher.publish(msg)

    def update_joint_state(self) -> None:
        try:
            if self.sdk_version == 2403:
                self.pos = self.sdk_6001.currentpos_inquiry(0)
                axisActualVel = self.sdk_6001.axis_actual_vel_inquire()["axisActualVel"]
                torq = self.sdk_6001.motor_torque_inquire()["torq"]
            elif self.sdk_version == 2207:
                self.pos = self.sdk_6001.currentpos_inquiry(0)
                axisActualVel = self.sdk_6001.axis_actual_vel_inquire()["axisActualVel"]
                torq = self.sdk_6001.currenttorq_inquire()["torq"]

            self.current_position = [math.radians(p) for p in self.pos[: self.dof]]
            self.current_velocity = [math.radians(p) for p in axisActualVel[: self.dof]]
            self.current_effort = [
                float(t) / 1000 if t is not None else 0.0 for t in torq[: self.dof]
            ]
            self.T_current = self.robotfuc.kinematics.fkine(self.pos)
            self.current_position_xyz, self.current_orientation_quat = (
                self.matrix_to_xyz_and_quaternion(self.T_current)
            )
            self._publish_end_pose()
        except Exception as e:
            self.logger.warn(f"{self.arm_key} update_joint_state 错误: {e}")

    def check_arrival_and_publish(self, current_time: float) -> None:
        # queue/motion_control 到位判断
        if self.arm_control_mode != "servo_j":
            if self.received_trajectory_length > 1:
                if current_time - self.motion_start_time < self.motion_guard_time:
                    self.settled_since = 0.0
                    return

            if not self.is_moving or self.target_position is None:
                return
            if self.current_position is None or self.current_velocity is None:
                return

            position_ok = True
            velocity_ok = True
            for i in range(min(len(self.current_position), len(self.target_position))):
                if (
                    abs(self.current_position[i] - self.target_position[i])
                    > self.position_tolerance
                ):
                    position_ok = False
                if abs(self.current_velocity[i]) > self.velocity_tolerance:
                    velocity_ok = False

            if position_ok and velocity_ok:
                if self.settled_since == 0.0:
                    self.settled_since = current_time
                if current_time - self.settled_since >= self.settle_time:
                    self.is_moving = False
                    self.last_motion_end_time = current_time
                    self._publish_motion("move complete")
            else:
                self.settled_since = 0.0
            return

        # servo_j 到位判断
        if (
            not self.servo_j_motion_active
            or self.current_position is None
            or self.current_velocity is None
        ):
            return
        idle_time = current_time - self.servo_j_last_send_time
        idle_ok = idle_time > max(
            self.servo_j_idle_timeout, 2.0 / max(self.servo_j_frequency, 1e-3)
        )
        if not idle_ok:
            return
        if self.servo_j_last_position is None:
            self.servo_j_last_position = self.current_position.copy()
            self.servo_j_settled_since = 0.0
            return
        pos_stable = all(
            abs(self.current_position[i] - self.servo_j_last_position[i])
            <= self.position_tolerance
            for i in range(len(self.current_position))
        )
        vel_stable = all(
            abs(v) <= self.velocity_tolerance for v in self.current_velocity
        )
        if pos_stable and vel_stable:
            if self.servo_j_settled_since == 0.0:
                self.servo_j_settled_since = current_time
            if current_time - self.servo_j_settled_since >= self.settle_time:
                if not self.servo_j_arrived:
                    self.servo_j_arrived = True
                    self.servo_j_motion_active = False
                    self._publish_motion("servo_j move complete")
        else:
            self.servo_j_settled_since = 0.0
        self.servo_j_last_position = self.current_position.copy()

    def shutdown(self) -> None:
        self.running = False
        with self.trajectory_condition:
            self.trajectory_condition.notify_all()
        try:
            self.robotfuc.robot_stop(self.sdk_6001)
        except Exception:
            pass
        try:
            self.sdk_6001.disconnect()
        except Exception:
            pass
        try:
            self.sdk_7000.disconnect()
        except Exception:
            pass


class TLDriver(Node):
    def __init__(self):
        super().__init__("tl_driver")
        # 以下参数修改 tl_driver_config.yaml 文件生效
        self.declare_parameter("arm_mode", "single")  # single: 单臂控制; dual: 双臂控制
        self.declare_parameter("frequency", 100)  # 发布关节数据的频率
        self.declare_parameter("debug_logging", False)  # 根据参数设置日志级别

        # 端口
        self.declare_parameter("port_6001", 6001)
        self.declare_parameter("port_7000", 7000)

        # 单臂控制下的手臂(双臂下表示左臂)
        self.declare_parameter("ip1", "192.168.2.13")  # 机械臂IP地址
        self.declare_parameter("sdk_version1", 2403)  # 2403 or 2207 版本通讯协议
        self.declare_parameter(
            "dof1", 7
        )  # 自由度, TCB605_05、TCB610_06的自由度为6, TCB705_05、TCB710_06的自由度为7
        self.declare_parameter("trajectory_timeout1", 0.5)  # 轨迹结束超时时间(秒)
        self.declare_parameter(
            "stride1", 1
        )  # "queue" or "motion_control" 模式下的采样步长
        self.declare_parameter(
            "arm_control_mode1", "queue"
        )  # "queue" or "motion_control" or "servo_j"
        # "queue"（队列运动） or "motion_control"（7000端口运动控制）为离线控制， "servo_j"为在线控制
        self.declare_parameter(
            "speed1", 60
        )  # "queue" or "motion_control" 模式下的运动全局速度
        self.declare_parameter("position_tolerance1", 2.0)  # 到位判断位置容差(度)
        self.declare_parameter("velocity_tolerance1", 1.0)  # 到位判断速度容差(度/秒)
        self.declare_parameter("settle_time1", 0.1)  # 到位判断稳定时间(秒)
        self.declare_parameter(
            "servo_j_frequency1", 100
        )  # servo_j 模式发送频率(Hz)（以及接收轨迹的频率）

        # 双臂控制下的另一条手臂(双臂下表示右臂)
        self.declare_parameter("ip2", "192.168.2.14")
        self.declare_parameter("sdk_version2", 2403)
        self.declare_parameter("dof2", 7)
        self.declare_parameter("trajectory_timeout2", 0.5)
        self.declare_parameter("stride2", 1)
        self.declare_parameter("arm_control_mode2", "queue")
        self.declare_parameter("speed2", 60)
        self.declare_parameter("position_tolerance2", 2.0)
        self.declare_parameter("velocity_tolerance2", 1.0)
        self.declare_parameter("settle_time2", 0.1)
        self.declare_parameter("servo_j_frequency2", 100)

        self.arm_mode = self.get_parameter("arm_mode").value
        self.frequency = self.get_parameter("frequency").value
        self.debug_logging = self.get_parameter("debug_logging").value

        self.port_6001 = self.get_parameter("port_6001").value
        self.port_7000 = self.get_parameter("port_7000").value

        self.ip1 = self.get_parameter("ip1").value
        self.sdk_version1 = self.get_parameter("sdk_version1").value
        self.dof1 = self.get_parameter("dof1").value
        self.trajectory_timeout1 = self.get_parameter("trajectory_timeout1").value
        self.stride1 = self.get_parameter("stride1").value
        self.arm_control_mode1 = self.get_parameter("arm_control_mode1").value
        self.speed1 = self.get_parameter("speed1").value
        self.position_tolerance1 = math.radians(
            self.get_parameter("position_tolerance1").value
        )
        self.velocity_tolerance1 = math.radians(
            self.get_parameter("velocity_tolerance1").value
        )
        self.settle_time1 = self.get_parameter("settle_time1").value
        self.servo_j_frequency1 = self.get_parameter("servo_j_frequency1").value

        self.ip2 = self.get_parameter("ip2").value
        self.sdk_version2 = self.get_parameter("sdk_version2").value
        self.dof2 = self.get_parameter("dof2").value
        self.trajectory_timeout2 = self.get_parameter("trajectory_timeout2").value
        self.stride2 = self.get_parameter("stride2").value
        self.arm_control_mode2 = self.get_parameter("arm_control_mode2").value
        self.speed2 = self.get_parameter("speed2").value
        self.position_tolerance2 = math.radians(
            self.get_parameter("position_tolerance2").value
        )
        self.velocity_tolerance2 = math.radians(
            self.get_parameter("velocity_tolerance2").value
        )
        self.settle_time2 = self.get_parameter("settle_time2").value
        self.servo_j_frequency2 = self.get_parameter("servo_j_frequency2").value

        if self.debug_logging:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            self.get_logger().info("调试日志已启用")
            log_level = 1
        else:
            log_level = 0

        self.joint_state_publisher = self.create_publisher(
            JointState, "tl_driver/current_joint_states", 10
        )

        # 单臂模式保持原 topic
        self.motion_complete_publisher = None
        if self.arm_mode != "dual":
            self.motion_complete_publisher = self.create_publisher(
                String, "tl_driver/motion_complete", 10
            )
            complete_msg = String()
            complete_msg.data = "ready"
            self.motion_complete_publisher.publish(complete_msg)
            # 单臂模式下避免重复刷屏：仅在状态变化时发布
            self._single_last_motion_status = complete_msg.data

        self.trajectory_subscriber = self.create_subscription(
            JointTrajectory, "tl_driver/joint_trajectory", self.trajectory_callback, 10
        )

        self.robot_cmd_subscriber = self.create_subscription(
            String, "tl_driver/cmd", self.robot_cmd_callback, 10
        )

        self.timer = self.create_timer(1 / self.frequency, self.publish_joint_status)

        # 创建左右臂 driver（single 时只创建左臂, 且 joint_prefix 为空以保持兼容）
        self.arms: Dict[str, ArmDriver] = {}

        left_prefix = "" if self.arm_mode != "dual" else "left_"
        self.arms["armleft"] = ArmDriver(
            self,
            arm_key="armleft",
            joint_prefix=left_prefix,
            ip=self.ip1,
            port_6001=self.port_6001,
            port_7000=self.port_7000,
            sdk_version=self.sdk_version1,
            dof=self.dof1,
            trajectory_timeout=self.trajectory_timeout1,
            stride=self.stride1,
            arm_control_mode=self.arm_control_mode1,
            speed=self.speed1,
            position_tolerance=self.position_tolerance1,
            velocity_tolerance=self.velocity_tolerance1,
            settle_time=self.settle_time1,
            servo_j_frequency=self.servo_j_frequency1,
            log_level=log_level,
            is_dual=(self.arm_mode == "dual"),
        )

        if self.arm_mode == "dual":
            self.arms["armright"] = ArmDriver(
                self,
                arm_key="armright",
                joint_prefix="right_",
                ip=self.ip2,
                port_6001=self.port_6001,
                port_7000=self.port_7000,
                sdk_version=self.sdk_version2,
                dof=self.dof2,
                trajectory_timeout=self.trajectory_timeout2,
                stride=self.stride2,
                arm_control_mode=self.arm_control_mode2,
                speed=self.speed2,
                position_tolerance=self.position_tolerance2,
                velocity_tolerance=self.velocity_tolerance2,
                settle_time=self.settle_time2,
                servo_j_frequency=self.servo_j_frequency2,
                log_level=log_level,
                is_dual=(self.arm_mode == "dual"),
            )

        self.get_logger().info(f"TLDriver initialized, arm_mode={self.arm_mode}")

    def trajectory_callback(self, msg):
        try:
            now = time.time()

            # single：严格依赖 joint_names 进行映射
            if self.arm_mode == "single":
                if not msg.joint_names:
                    self.get_logger().error(
                        "single 模式下 JointTrajectory 缺少 joint_names, 忽略该轨迹"
                    )
                    return

                left = self.arms["armleft"]
                idx_map = {name: i for i, name in enumerate(msg.joint_names)}

                # 允许两种命名：tl_robot_joint* 或 left_tl_robot_joint*
                index_list: List[int] = []
                missing: List[str] = []
                for j in left.joint_names:
                    i = idx_map.get(j)
                    if i is None and j.startswith("tl_robot_joint"):
                        alias = f"left_{j}"
                        i = idx_map.get(alias)
                    if i is None:
                        missing.append(j)
                    else:
                        index_list.append(i)

                if missing:
                    self.get_logger().error(
                        f"single 模式下 JointTrajectory 关节名称不完整, 缺少: {missing}, 忽略该轨迹"
                    )
                    return

                points_deg: List[List[float]] = []
                for pt in msg.points:
                    if not pt.positions:
                        continue
                    positions = pt.positions
                    if max(index_list) >= len(positions):
                        self.get_logger().error(
                            "single 模式下 JointTrajectory 点的 positions 数量不足, 忽略该轨迹"
                        )
                        return
                    degs = [round(math.degrees(positions[i]), 4) for i in index_list]

                    points_deg.append(degs)

                if not points_deg:
                    return

                if left.arm_control_mode == "servo_j":
                    for p in points_deg:
                        left.enqueue_servo_point_deg(p)
                else:
                    left.enqueue_trajectory_points_deg(points_deg, now)
                return

            # dual：按 joint_names 拆分左右臂
            if not msg.joint_names:
                self.get_logger().error(
                    "dual 模式下 JointTrajectory 缺少 joint_names, 忽略该轨迹"
                )
                return

            idx_map = {name: i for i, name in enumerate(msg.joint_names)}
            left = self.arms.get("armleft")
            right = self.arms.get("armright")
            if left is None or right is None:
                return

            # 先检查左右臂的所有关节是否都在 joint_names 中
            missing_left: List[str] = [j for j in left.joint_names if j not in idx_map]
            missing_right: List[str] = [
                j for j in right.joint_names if j not in idx_map
            ]
            if missing_left or missing_right:
                if missing_left:
                    self.get_logger().error(
                        f"dual 模式下 left 关节名称不完整, 缺少: {missing_left}, 忽略该轨迹"
                    )
                if missing_right:
                    self.get_logger().error(
                        f"dual 模式下 right 关节名称不完整, 缺少: {missing_right}, 忽略该轨迹"
                    )
                return

            left_indices = [idx_map[j] for j in left.joint_names]
            right_indices = [idx_map[j] for j in right.joint_names]

            left_points_deg: List[List[float]] = []
            right_points_deg: List[List[float]] = []

            for pt in msg.points:
                if not pt.positions:
                    continue
                positions = pt.positions
                if max(left_indices + right_indices) >= len(positions):
                    self.get_logger().error(
                        "dual 模式下 JointTrajectory 点的 positions 数量不足, 忽略该轨迹"
                    )
                    return

                # 依 joint_names 严格取值
                ldeg: List[float] = []
                rdeg: List[float] = []
                for i in left_indices:
                    ldeg.append(round(math.degrees(positions[i]), 4))
                for i in right_indices:
                    rdeg.append(round(math.degrees(positions[i]), 4))

                left_points_deg.append(ldeg)
                right_points_deg.append(rdeg)

            if left.arm_control_mode == "servo_j":
                for p in left_points_deg:
                    left.enqueue_servo_point_deg(p)
            else:
                if left_points_deg:
                    left.enqueue_trajectory_points_deg(left_points_deg, now)

            if right.arm_control_mode == "servo_j":
                for p in right_points_deg:
                    right.enqueue_servo_point_deg(p)
            else:
                if right_points_deg:
                    right.enqueue_trajectory_points_deg(right_points_deg, now)

        except Exception as e:
            self.get_logger().error(f"trajectory_callback 错误: {e}")
            traceback.print_exc()

    def robot_cmd_callback(self, msg):
        try:
            cmd = msg.data
            self.get_logger().info(f"接收到机械臂控制命令: {cmd}")

            # 单臂模式
            if self.arm_mode != "dual":
                if (
                    cmd == "arm_power_on"
                    or cmd == "armleft_power_on"
                    or cmd == "armright_power_on"
                ):
                    self.arms["armleft"].power_on()
                    self.get_logger().info("单臂上电完成")
                elif (
                    cmd == "arm_power_off"
                    or cmd == "armleft_power_off"
                    or cmd == "armright_power_off"
                ):
                    self.arms["armleft"].power_off()
                    self.get_logger().info("单臂下电完成")
                else:
                    self.get_logger().warn(f"单臂模式下不支持的命令: {cmd}")
                return

            # 双臂模式
            if cmd == "arm_power_on":
                # 双臂同时上电
                if "armleft" in self.arms and "armright" in self.arms:
                    t1 = threading.Thread(
                        target=self.arms["armleft"].power_on, daemon=True
                    )
                    t2 = threading.Thread(
                        target=self.arms["armright"].power_on, daemon=True
                    )
                    t1.start()
                    t2.start()
                    t1.join()
                    t2.join()
                    self.get_logger().info("双臂上电完成")
            elif cmd == "arm_power_off":
                # 双臂同时下电
                if "armleft" in self.arms and "armright" in self.arms:
                    t1 = threading.Thread(
                        target=self.arms["armleft"].power_off, daemon=True
                    )
                    t2 = threading.Thread(
                        target=self.arms["armright"].power_off, daemon=True
                    )
                    t1.start()
                    t2.start()
                    t1.join()
                    t2.join()
                    self.get_logger().info("双臂下电完成")
            elif cmd == "armleft_power_on":
                # 左臂单独上电
                if "armleft" in self.arms:
                    self.arms["armleft"].power_on()
                    self.get_logger().info("左臂上电完成")
            elif cmd == "armleft_power_off":
                # 左臂单独下电
                if "armleft" in self.arms:
                    self.arms["armleft"].power_off()
                    self.get_logger().info("左臂下电完成")
            elif cmd == "armright_power_on":
                # 右臂单独上电
                if "armright" in self.arms:
                    self.arms["armright"].power_on()
                    self.get_logger().info("右臂上电完成")
            elif cmd == "armright_power_off":
                # 右臂单独下电
                if "armright" in self.arms:
                    self.arms["armright"].power_off()
                    self.get_logger().info("右臂下电完成")
            else:
                self.get_logger().warn(f"未知命令: {cmd}")

        except Exception as e:
            self.get_logger().error(f"robot_cmd_callback 错误: {e}")
            traceback.print_exc()

    def publish_joint_status(self):
        try:
            current_time = time.time()
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = ""

            # 更新各臂关节状态并聚合发布
            names: List[str] = []
            pos: List[float] = []
            vel: List[float] = []
            eff: List[float] = []

            if self.arm_mode != "dual":
                arm = self.arms["armleft"]
                arm.update_joint_state()
                if arm.current_position is None:
                    return
                names = arm.joint_names
                pos = arm.current_position
                vel = arm.current_velocity or [0.0] * len(pos)
                eff = arm.current_effort or [0.0] * len(pos)

                # 单臂保持原 motion_complete topic
                arm.check_arrival_and_publish(current_time)
                if self.motion_complete_publisher is not None:
                    status = arm.move_status
                    # 兼容旧逻辑：ready/moving/complete 都可能出现, 但只在变化时发布
                    if not hasattr(self, "_single_last_motion_status"):
                        self._single_last_motion_status = ""
                    if status != self._single_last_motion_status:
                        msg = String()
                        msg.data = status
                        self.motion_complete_publisher.publish(msg)
                        self._single_last_motion_status = status

            else:
                left = self.arms["armleft"]
                right = self.arms["armright"]
                left.update_joint_state()
                right.update_joint_state()
                if left.current_position is None or right.current_position is None:
                    return

                names = left.joint_names + right.joint_names
                pos = left.current_position + right.current_position
                vel = (left.current_velocity or [0.0] * left.dof) + (
                    right.current_velocity or [0.0] * right.dof
                )
                eff = (left.current_effort or [0.0] * left.dof) + (
                    right.current_effort or [0.0] * right.dof
                )

                left.check_arrival_and_publish(current_time)
                right.check_arrival_and_publish(current_time)

            joint_msg.name = names
            joint_msg.position = pos
            joint_msg.velocity = vel
            joint_msg.effort = eff
            self.joint_state_publisher.publish(joint_msg)

        except Exception as e:
            self.get_logger().warn(f"publish_joint_status 错误: {e}")

    def destroy_node(self):
        try:
            for arm in self.arms.values():
                arm.shutdown()

        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TLDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        try:
            node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception as e:
                pass


if __name__ == "__main__":
    main()
