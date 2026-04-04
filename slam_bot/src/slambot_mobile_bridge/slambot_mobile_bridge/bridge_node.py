import asyncio
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional
from uuid import uuid4

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from pydantic import BaseModel
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from slambot_navigation.action import AStar


class BridgeError(RuntimeError):
    """Raised when a bridge operation fails."""


@dataclass
class LaunchProcess:
    mode: str
    process: subprocess.Popen


class ModeRequest(BaseModel):
    mode: str


class GoalRequest(BaseModel):
    x_m: float
    y_m: float


class SlambotBridgeNode(Node):
    def __init__(self, ros_args: Optional[list[str]] = None) -> None:
        rclpy.init(args=ros_args)
        super().__init__('slambot_mobile_bridge')

        self.state_lock = threading.RLock()
        self.telemetry_revision = 0
        self.map_revision = 0
        self.current_mode = 'idle'
        self.current_launch: Optional[LaunchProcess] = None
        self.current_map_msg: Optional[OccupancyGrid] = None
        self.current_pose_msg: Optional[PoseStamped] = None
        self.current_odom_msg: Optional[Odometry] = None
        self.current_path_msg: Optional[NavPath] = None
        self.last_navigation_goal: Optional[tuple[float, float]] = None
        self.last_planned_path_msg: Optional[NavPath] = None
        self.paused_path_msg: Optional[NavPath] = None
        self.is_navigating = False
        self.is_paused = False
        self.pause_requested = False
        self.last_error = ''
        self.teleop_owner_id: Optional[str] = None
        self.teleop_deadline_monotonic = 0.0

        self.sim = as_bool(self.declare_parameter('sim', False).value)
        self.http_host = str(self.declare_parameter('http_host', '0.0.0.0').value)
        self.http_port = int(self.declare_parameter('http_port', 8000).value)
        self.startup_mode = str(self.declare_parameter('startup_mode', 'mapping').value)
        self.bringup_package = str(self.declare_parameter('bringup_package', 'slambot_bringup').value)
        self.bringup_launch = str(
            self.declare_parameter('bringup_launch', 'slambot_bringup.launch.py').value
        )
        self.default_map_yaml = Path(
            str(self.declare_parameter('default_map_yaml', self.default_map_yaml_path()).value)
        )
        self.map_save_prefix = Path(
            str(self.declare_parameter('map_save_prefix', self.default_map_save_prefix()).value)
        )
        self.a_star_action_name = str(self.declare_parameter('a_star_action_name', '/a_star').value)
        self.stop_service_name = str(self.declare_parameter('stop_service_name', '/lqr/stop').value)
        self.teleop_timeout_ms = int(self.declare_parameter('teleop_timeout_ms', 250).value)
        self.launch_startup_delay_sec = float(
            self.declare_parameter('launch_startup_delay_sec', 1.0).value
        )
        self.launch_stop_timeout_sec = float(
            self.declare_parameter('launch_stop_timeout_sec', 10.0).value
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_resume_pub = self.create_publisher(
            NavPath,
            '/path',
            QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )
        self.stop_client = self.create_client(Trigger, self.stop_service_name)
        self.a_star_client = ActionClient(self, AStar, self.a_star_action_name)

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._handle_map,
            QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )
        self.create_subscription(PoseStamped, '/estimated_pose', self._handle_pose, 10)
        self.create_subscription(Odometry, '/odom/filtered', self._handle_odom, 10)
        self.create_subscription(
            NavPath,
            '/path',
            self._handle_path,
            QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )
        self.create_subscription(Bool, '/is_navigating', self._handle_navigating, 10)

        self.create_timer(0.05, self._check_teleop_timeout)
        self.create_timer(0.5, self._check_launch_process)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)

    def default_map_yaml_path(self) -> str:
        return '/home/slambot/ros2_ws/maps/my_map.yaml' if self.sim else '/home/slambot/ros2_ws_pi/maps/my_map_real.yaml'

    def default_map_save_prefix(self) -> str:
        return '/home/slambot/ros2_ws/maps/my_map' if self.sim else '/home/slambot/ros2_ws_pi/maps/my_map_real'

    def start(self) -> None:
        if not self._spin_thread.is_alive():
            self._spin_thread.start()
        if self.startup_mode in {'mapping', 'localization'}:
            try:
                self.set_mode(self.startup_mode)
            except Exception as exc:  # pragma: no cover - startup path
                self._set_error(f'Failed to start in {self.startup_mode}: {exc}')

    def shutdown(self) -> None:
        try:
            self._stop_launch()
        finally:
            self._executor.shutdown()
            if self._spin_thread.is_alive():
                self._spin_thread.join(timeout=2.0)
            self.destroy_node()
            rclpy.shutdown()

    def snapshot_state(self) -> Dict[str, Any]:
        with self.state_lock:
            display_path_msg = self._display_path_locked()
            can_pause = (
                self.current_mode == 'localization'
                and not self.is_paused
                and display_path_msg is not None
            )
            return {
                'mode': self.current_mode,
                'localization_mode': self.current_mode == 'localization',
                'launch_running': self._launch_running_locked(),
                'is_navigating': self.is_navigating,
                'is_paused': self.is_paused,
                'can_pause': can_pause,
                'can_resume': self.current_mode == 'localization' and self.is_paused and self.paused_path_msg is not None,
                'can_cancel': self.current_mode == 'localization' and (
                    self.current_path_msg is not None or self.paused_path_msg is not None or self.last_navigation_goal is not None
                ),
                'map_available': self.current_map_msg is not None,
                'map_revision': self.map_revision,
                'telemetry_revision': self.telemetry_revision,
                'map_yaml': str(self.default_map_yaml),
                'map_save_prefix': str(self.map_save_prefix),
                'pose': serialize_pose(self.current_pose_msg, self.current_odom_msg, self.current_mode),
                'path': serialize_path(display_path_msg),
                'last_error': self.last_error or None,
            }

    def snapshot_map(self) -> Dict[str, Any]:
        with self.state_lock:
            if self.current_map_msg is None:
                raise BridgeError('No map has been received yet.')
            map_msg = self.current_map_msg

        return {
            'frame_id': map_msg.header.frame_id or 'map',
            'resolution': map_msg.info.resolution,
            'width': map_msg.info.width,
            'height': map_msg.info.height,
            'origin': {
                'x': map_msg.info.origin.position.x,
                'y': map_msg.info.origin.position.y,
                'yaw': yaw_from_orientation(map_msg.info.origin.orientation),
            },
            'data': list(map_msg.data),
            'revision': self.map_revision,
        }

    def set_mode(self, mode: str) -> Dict[str, Any]:
        normalized_mode = mode.lower().strip()
        if normalized_mode not in {'mapping', 'localization'}:
            raise BridgeError(f'Unsupported mode: {mode}')

        with self.state_lock:
            if normalized_mode == self.current_mode and self._launch_running_locked():
                return self.snapshot_state()

        self._stop_launch()
        self._start_launch(normalized_mode)
        return self.snapshot_state()

    def save_map(self) -> Dict[str, Any]:
        with self.state_lock:
            if self.current_mode != 'mapping':
                raise BridgeError('Map saving is only available in mapping mode.')

        self.map_save_prefix.parent.mkdir(parents=True, exist_ok=True)
        command = [
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
            str(self.map_save_prefix),
        ]
        result = subprocess.run(command, capture_output=True, text=True, timeout=60, check=False)
        if result.returncode != 0:
            raise BridgeError(result.stderr.strip() or result.stdout.strip() or 'map_saver_cli failed')
        yaml_path = self.map_save_prefix.with_suffix('.yaml')
        if not yaml_path.exists():
            raise BridgeError(f'Map saver completed but {yaml_path} was not created.')
        self._mark_revision()
        return {'map_yaml': str(yaml_path)}

    def stop_motion(self) -> Dict[str, Any]:
        with self.state_lock:
            current_mode = self.current_mode
        self._clear_teleop()
        self.publish_zero_velocity()
        if current_mode == 'localization':
            self._call_lqr_stop()
            with self.state_lock:
                self.is_navigating = False
                self.is_paused = False
                self.pause_requested = False
                self.current_path_msg = None
                self.last_planned_path_msg = None
                self.paused_path_msg = None
                self.last_navigation_goal = None
        self._mark_revision()
        return self.snapshot_state()

    def send_navigation_goal(self, x_m: float, y_m: float) -> Dict[str, Any]:
        with self.state_lock:
            if self.current_mode != 'localization':
                raise BridgeError('Point goals are only available in localization mode.')

        if not self.a_star_client.wait_for_server(timeout_sec=2.0):
            raise BridgeError('A* action server is not available.')

        goal_msg = AStar.Goal()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = float(x_m)
        goal_msg.goal.pose.position.y = float(y_m)
        goal_msg.goal.pose.orientation.w = 1.0

        send_future = self.a_star_client.send_goal_async(goal_msg)
        goal_handle = self._await_future(send_future, 5.0)
        if goal_handle is None or not goal_handle.accepted:
            raise BridgeError('Path rejected.')

        result_future = goal_handle.get_result_async()
        result = self._await_future(result_future, 10.0)
        if result is None or not result.result.success:
            raise BridgeError('Whoops! I cannot navigate there.')

        with self.state_lock:
            self.last_navigation_goal = (x_m, y_m)
            self.last_planned_path_msg = result.result.path
            self.current_path_msg = result.result.path
            self.paused_path_msg = None
            self.is_paused = False
            self.is_navigating = True
        self._mark_revision()
        return {'success': True, 'path_points': len(result.result.path.poses)}

    def pause_navigation(self) -> Dict[str, Any]:
        with self.state_lock:
            if self.current_mode != 'localization':
                raise BridgeError('Pause is only available in localization mode.')
            path_msg = self.current_path_msg or self.last_planned_path_msg
            if path_msg is None:
                raise BridgeError('There is no active path to pause.')
            self.pause_requested = True

        try:
            self._call_lqr_stop()
        except Exception:
            with self.state_lock:
                self.pause_requested = False
            raise

        with self.state_lock:
            self.pause_requested = False
            self.paused_path_msg = path_msg
            self.is_paused = True
            self.is_navigating = False
        self._mark_revision()
        return self.snapshot_state()

    def resume_navigation(self) -> Dict[str, Any]:
        with self.state_lock:
            if self.current_mode != 'localization':
                raise BridgeError('Resume is only available in localization mode.')
            if self.paused_path_msg is None:
                raise BridgeError('There is no paused path to resume.')
            path_msg = self.paused_path_msg

        self.path_resume_pub.publish(path_msg)
        with self.state_lock:
            self.current_path_msg = path_msg
            self.last_planned_path_msg = path_msg
            self.is_paused = False
            self.pause_requested = False
            self.is_navigating = True
        self._mark_revision()
        return self.snapshot_state()

    def cancel_navigation(self) -> Dict[str, Any]:
        with self.state_lock:
            if self.current_mode != 'localization':
                raise BridgeError('Cancel is only available in localization mode.')

        self._call_lqr_stop()
        with self.state_lock:
            self.current_path_msg = None
            self.last_planned_path_msg = None
            self.paused_path_msg = None
            self.last_navigation_goal = None
            self.is_paused = False
            self.pause_requested = False
            self.is_navigating = False
        self._mark_revision()
        return self.snapshot_state()

    def handle_teleop(self, client_id: str, payload: Dict[str, Any]) -> None:
        with self.state_lock:
            current_mode = self.current_mode
            is_navigating = self.is_navigating

        if current_mode == 'mapping':
            pass
        elif current_mode == 'localization':
            if is_navigating:
                raise BridgeError('Teleop is disabled while LQR is actively navigating.')
        else:
            raise BridgeError('Teleop is only available in mapping or idle-localization control.')

        deadman = bool(payload.get('deadman', False))
        if not deadman:
            if self.teleop_owner_id == client_id:
                self._clear_teleop()
                self.publish_zero_velocity()
            return

        cmd = Twist()
        cmd.linear.x = float(payload.get('linear_x', 0.0))
        cmd.linear.y = float(payload.get('linear_y', 0.0))
        cmd.angular.z = float(payload.get('angular_z', 0.0))
        self.cmd_vel_pub.publish(cmd)

        with self.state_lock:
            self.teleop_owner_id = client_id
            self.teleop_deadline_monotonic = time.monotonic() + (self.teleop_timeout_ms / 1000.0)
        self._mark_revision()

    def handle_disconnect(self, client_id: str) -> None:
        with self.state_lock:
            should_clear = self.teleop_owner_id == client_id
        if should_clear:
            self._clear_teleop()
            self.publish_zero_velocity()

    def publish_zero_velocity(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _start_launch(self, mode: str) -> None:
        command = [
            'ros2',
            'launch',
            self.bringup_package,
            self.bringup_launch,
            f"sim:={'true' if self.sim else 'false'}",
            f"localization_mode:={'true' if mode == 'localization' else 'false'}",
            f'map_yaml:={self.default_map_yaml}',
        ]
        process = subprocess.Popen(command, preexec_fn=os.setsid)
        time.sleep(self.launch_startup_delay_sec)
        exit_code = process.poll()
        if exit_code is not None:
            raise BridgeError(f'{mode} launch exited immediately with code {exit_code}')

        with self.state_lock:
            self.current_launch = LaunchProcess(mode=mode, process=process)
            self.current_mode = mode
            self.is_paused = False
            self.pause_requested = False
            self.is_navigating = False
            self.last_error = ''
            if mode == 'mapping':
                self.current_path_msg = None
                self.last_planned_path_msg = None
                self.paused_path_msg = None
                self.last_navigation_goal = None
        self._mark_revision()
        self.get_logger().info(f'Started {mode} launch with PID {process.pid}')

    def _stop_launch(self) -> None:
        with self.state_lock:
            launch = self.current_launch

        self._clear_teleop()
        self.publish_zero_velocity()

        if launch is not None:
            self._terminate_process_group(launch.process)

        with self.state_lock:
            self.current_launch = None
            self.current_mode = 'idle'
            self.is_navigating = False
            self.is_paused = False
            self.pause_requested = False
            self.current_path_msg = None
            self.last_planned_path_msg = None
            self.paused_path_msg = None
            self.last_navigation_goal = None
        self._mark_revision()

    def _terminate_process_group(self, process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return

        os.killpg(process.pid, signal.SIGINT)
        deadline = time.monotonic() + self.launch_stop_timeout_sec
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return
            time.sleep(0.1)

        process.terminate()
        try:
            process.wait(timeout=3.0)
            return
        except subprocess.TimeoutExpired:
            pass

        process.kill()
        process.wait(timeout=3.0)

    def _call_lqr_stop(self) -> None:
        if not self.stop_client.wait_for_service(timeout_sec=2.0):
            raise BridgeError(f'LQR stop service {self.stop_service_name} is not available.')
        future = self.stop_client.call_async(Trigger.Request())
        response = self._await_future(future, 5.0)
        if response is None or not response.success:
            message = response.message if response is not None else 'timed out'
            raise BridgeError(f'LQR stop failed: {message}')

    def _await_future(self, future: Any, timeout_sec: float) -> Any:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.05)
        return None

    def _check_teleop_timeout(self) -> None:
        with self.state_lock:
            expired = (
                self.teleop_owner_id is not None
                and time.monotonic() >= self.teleop_deadline_monotonic
            )
        if expired:
            self.get_logger().warn('Teleop timeout exceeded; publishing zero velocity.')
            self._clear_teleop()
            self.publish_zero_velocity()

    def _check_launch_process(self) -> None:
        with self.state_lock:
            launch = self.current_launch
        if launch is None:
            return
        exit_code = launch.process.poll()
        if exit_code is None:
            return
        with self.state_lock:
            self.current_launch = None
            self.current_mode = 'idle'
            self.is_navigating = False
            self.is_paused = False
            self.last_error = f'{launch.mode} launch exited with code {exit_code}'
        self._mark_revision()

    def _handle_map(self, msg: OccupancyGrid) -> None:
        with self.state_lock:
            self.current_map_msg = msg
            self.map_revision += 1
        self._mark_revision()

    def _handle_pose(self, msg: PoseStamped) -> None:
        with self.state_lock:
            self.current_pose_msg = msg
        self._mark_revision()

    def _handle_odom(self, msg: Odometry) -> None:
        with self.state_lock:
            self.current_odom_msg = msg
        self._mark_revision()

    def _handle_path(self, msg: NavPath) -> None:
        with self.state_lock:
            self.current_path_msg = msg if msg.poses else None
            if msg.poses:
                self.last_planned_path_msg = msg
        self._mark_revision()

    def _handle_navigating(self, msg: Bool) -> None:
        with self.state_lock:
            self.is_navigating = bool(msg.data)
            if self.is_navigating:
                self.is_paused = False
                self.pause_requested = False
            elif not self.is_paused and not self.pause_requested:
                self.current_path_msg = None
                self.last_planned_path_msg = None
                self.last_navigation_goal = None
        self._mark_revision()

    def _clear_teleop(self) -> None:
        with self.state_lock:
            self.teleop_owner_id = None
            self.teleop_deadline_monotonic = 0.0

    def _display_path_locked(self) -> Optional[NavPath]:
        return self.paused_path_msg or self.current_path_msg or self.last_planned_path_msg

    def _mark_revision(self) -> None:
        with self.state_lock:
            self.telemetry_revision += 1

    def _launch_running_locked(self) -> bool:
        return self.current_launch is not None and self.current_launch.process.poll() is None

    def _set_error(self, message: str) -> None:
        self.get_logger().error(message)
        with self.state_lock:
            self.last_error = message
        self._mark_revision()


def serialize_pose(
    pose_msg: Optional[PoseStamped],
    odom_msg: Optional[Odometry],
    current_mode: str,
) -> Optional[Dict[str, float]]:
    if current_mode == 'mapping' and odom_msg is not None:
        return {
            'x': odom_msg.pose.pose.position.x,
            'y': odom_msg.pose.pose.position.y,
            'theta': yaw_from_orientation(odom_msg.pose.pose.orientation),
        }
    if pose_msg is not None:
        return {
            'x': pose_msg.pose.position.x,
            'y': pose_msg.pose.position.y,
            'theta': yaw_from_orientation(pose_msg.pose.orientation),
        }
    if odom_msg is not None:
        return {
            'x': odom_msg.pose.pose.position.x,
            'y': odom_msg.pose.pose.position.y,
            'theta': yaw_from_orientation(odom_msg.pose.pose.orientation),
        }
    return None


def serialize_path(path_msg: Optional[NavPath]) -> list[Dict[str, float]]:
    if path_msg is None:
        return []
    return [
        {
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
        }
        for pose in path_msg.poses
    ]


def yaw_from_orientation(orientation: Any) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {'1', 'true', 'yes', 'on'}
    return bool(value)


async def telemetry_stream(websocket: WebSocket, bridge: SlambotBridgeNode) -> None:
    last_telemetry_revision = -1
    last_map_revision = -1
    while True:
        state = bridge.snapshot_state()
        telemetry_revision = int(state['telemetry_revision'])
        map_revision = int(state['map_revision'])

        if telemetry_revision != last_telemetry_revision:
            await websocket.send_text(json.dumps({'type': 'state', 'state': state}))
            last_telemetry_revision = telemetry_revision

        if state['map_available'] and map_revision != last_map_revision:
            try:
                payload = bridge.snapshot_map()
            except BridgeError:
                payload = None
            if payload is not None:
                await websocket.send_text(json.dumps({'type': 'map', 'map': payload}))
                last_map_revision = map_revision

        await asyncio.sleep(0.1)


def create_app(bridge: SlambotBridgeNode) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_: FastAPI):
        await asyncio.to_thread(bridge.start)
        try:
            yield
        finally:
            await asyncio.to_thread(bridge.shutdown)

    app = FastAPI(title='Slambot Mobile Bridge', lifespan=lifespan)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_methods=['*'],
        allow_headers=['*'],
    )

    @app.get('/health')
    async def health() -> Dict[str, Any]:
        return {'status': 'ok', 'state': bridge.snapshot_state()}

    @app.get('/state')
    async def state() -> Dict[str, Any]:
        return bridge.snapshot_state()

    @app.get('/map/current')
    async def map_current() -> Dict[str, Any]:
        try:
            return bridge.snapshot_map()
        except BridgeError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.post('/mode')
    async def set_mode(request: ModeRequest) -> Dict[str, Any]:
        try:
            return bridge.set_mode(request.mode)
        except BridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post('/mapping/save')
    async def save_map() -> Dict[str, Any]:
        try:
            return bridge.save_map()
        except BridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post('/navigation/goal')
    async def navigation_goal(request: GoalRequest) -> Dict[str, Any]:
        try:
            return bridge.send_navigation_goal(request.x_m, request.y_m)
        except BridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post('/navigation/pause')
    async def navigation_pause() -> Dict[str, Any]:
        try:
            return bridge.pause_navigation()
        except BridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post('/navigation/resume')
    async def navigation_resume() -> Dict[str, Any]:
        try:
            return bridge.resume_navigation()
        except BridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post('/navigation/cancel')
    async def navigation_cancel() -> Dict[str, Any]:
        try:
            return bridge.cancel_navigation()
        except BridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post('/navigation/stop')
    async def navigation_stop() -> Dict[str, Any]:
        try:
            return bridge.stop_motion()
        except BridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.websocket('/ws')
    async def websocket_endpoint(websocket: WebSocket) -> None:
        client_id = str(uuid4())
        await websocket.accept()
        sender_task = asyncio.create_task(telemetry_stream(websocket, bridge))
        try:
            while True:
                message = await websocket.receive_text()
                payload = json.loads(message)
                message_type = payload.get('type')
                if message_type == 'teleop':
                    bridge.handle_teleop(client_id, payload)
                elif message_type == 'ping':
                    await websocket.send_json({'type': 'pong'})
        except WebSocketDisconnect:
            pass
        except (ValueError, BridgeError) as exc:
            await websocket.send_json({'type': 'error', 'message': str(exc)})
        finally:
            sender_task.cancel()
            bridge.handle_disconnect(client_id)

    return app


def main() -> None:
    bridge = SlambotBridgeNode(sys.argv)
    app = create_app(bridge)
    uvicorn.run(app, host=bridge.http_host, port=bridge.http_port, log_level='info')
