import asyncio
import json
import math
import sys
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, Optional
from uuid import uuid4

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from pydantic import BaseModel
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger

from slambot_interfaces.msg import OrchestrationState
from slambot_interfaces.srv import NavigateTo, SaveMap, SetMode, TeleopCommand


class BridgeError(RuntimeError):
    """Raised when a bridge operation fails."""


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
        self.current_map_msg: Optional[OccupancyGrid] = None
        self.current_pose_msg: Optional[PoseStamped] = None
        self.current_odom_msg: Optional[Odometry] = None
        self.current_path_msg: Optional[NavPath] = None
        self.last_planned_path_msg: Optional[NavPath] = None

        self.current_mode = 'idle'
        self.launch_running = False
        self.is_navigating = False
        self.is_paused = False
        self.pause_requested = False
        self.last_error = ''
        self.last_navigation_goal: Optional[tuple[float, float]] = None

        self.sim = as_bool(self.declare_parameter('sim', False).value)
        self.http_host = str(self.declare_parameter('http_host', '0.0.0.0').value)
        self.http_port = int(self.declare_parameter('http_port', 8000).value)
        self.default_map_yaml = Path(
            str(self.declare_parameter('default_map_yaml', self.default_map_yaml_path()).value)
        )
        self.map_save_prefix = Path(
            str(self.declare_parameter('map_save_prefix', self.default_map_save_prefix()).value)
        )
        self.set_mode_service_name = str(
            self.declare_parameter('set_mode_service_name', '/orchestration/set_mode').value
        )
        self.save_map_service_name = str(
            self.declare_parameter('save_map_service_name', '/orchestration/save_map').value
        )
        self.navigate_to_service_name = str(
            self.declare_parameter('navigate_to_service_name', '/orchestration/navigate_to').value
        )
        self.pause_navigation_service_name = str(
            self.declare_parameter('pause_navigation_service_name', '/orchestration/pause_navigation').value
        )
        self.resume_navigation_service_name = str(
            self.declare_parameter('resume_navigation_service_name', '/orchestration/resume_navigation').value
        )
        self.cancel_navigation_service_name = str(
            self.declare_parameter('cancel_navigation_service_name', '/orchestration/cancel_navigation').value
        )
        self.stop_motion_service_name = str(
            self.declare_parameter('stop_motion_service_name', '/orchestration/stop_motion').value
        )
        self.teleop_service_name = str(
            self.declare_parameter('teleop_service_name', '/orchestration/teleop').value
        )

        state_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.set_mode_client = self.create_client(SetMode, self.set_mode_service_name)
        self.save_map_client = self.create_client(SaveMap, self.save_map_service_name)
        self.navigate_to_client = self.create_client(NavigateTo, self.navigate_to_service_name)
        self.pause_navigation_client = self.create_client(Trigger, self.pause_navigation_service_name)
        self.resume_navigation_client = self.create_client(Trigger, self.resume_navigation_service_name)
        self.cancel_navigation_client = self.create_client(Trigger, self.cancel_navigation_service_name)
        self.stop_motion_client = self.create_client(Trigger, self.stop_motion_service_name)
        self.teleop_client = self.create_client(TeleopCommand, self.teleop_service_name)

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._handle_map,
            state_qos,
        )
        self.create_subscription(PoseStamped, '/estimated_pose', self._handle_pose, 10)
        self.create_subscription(Odometry, '/odom/filtered', self._handle_odom, 10)
        self.create_subscription(
            NavPath,
            '/path',
            self._handle_path,
            state_qos,
        )
        self.create_subscription(
            OrchestrationState,
            '/orchestration/state',
            self._handle_orchestration_state,
            state_qos,
        )

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

    def shutdown(self) -> None:
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
            can_resume = (
                self.current_mode == 'localization'
                and self.is_paused
                and self.last_planned_path_msg is not None
            )
            can_cancel = self.current_mode == 'localization' and (
                self.current_path_msg is not None
                or self.last_planned_path_msg is not None
                or self.last_navigation_goal is not None
            )
            return {
                'mode': self.current_mode,
                'localization_mode': self.current_mode == 'localization',
                'launch_running': self.launch_running,
                'is_navigating': self.is_navigating,
                'is_paused': self.is_paused,
                'can_pause': can_pause,
                'can_resume': can_resume,
                'can_cancel': can_cancel,
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
        request = SetMode.Request()
        request.mode = mode
        response = self._call_service(self.set_mode_client, request, 5.0)
        if response is None or not response.success:
            message = response.message if response is not None else 'timed out'
            raise BridgeError(message)
        return self.snapshot_state()

    def save_map(self) -> Dict[str, Any]:
        response = self._call_service(self.save_map_client, SaveMap.Request(), 65.0)
        if response is None or not response.success:
            message = response.message if response is not None else 'timed out'
            raise BridgeError(message)
        return {'map_yaml': response.map_yaml}

    def stop_motion(self) -> Dict[str, Any]:
        response = self._call_service(self.stop_motion_client, Trigger.Request(), 5.0)
        if response is None or not response.success:
            message = response.message if response is not None else 'timed out'
            raise BridgeError(message)
        return self.snapshot_state()

    def send_navigation_goal(self, x_m: float, y_m: float) -> Dict[str, Any]:
        request = NavigateTo.Request()
        request.x_m = float(x_m)
        request.y_m = float(y_m)
        response = self._call_service(self.navigate_to_client, request, 15.0)
        if response is None or not response.success:
            message = response.message if response is not None else 'timed out'
            raise BridgeError(message)
        return {'success': True, 'path_points': int(response.path_points)}

    def pause_navigation(self) -> Dict[str, Any]:
        response = self._call_service(self.pause_navigation_client, Trigger.Request(), 5.0)
        if response is None or not response.success:
            message = response.message if response is not None else 'timed out'
            raise BridgeError(message)
        return self.snapshot_state()

    def resume_navigation(self) -> Dict[str, Any]:
        response = self._call_service(self.resume_navigation_client, Trigger.Request(), 5.0)
        if response is None or not response.success:
            message = response.message if response is not None else 'timed out'
            raise BridgeError(message)
        return self.snapshot_state()

    def cancel_navigation(self) -> Dict[str, Any]:
        response = self._call_service(self.cancel_navigation_client, Trigger.Request(), 5.0)
        if response is None or not response.success:
            message = response.message if response is not None else 'timed out'
            raise BridgeError(message)
        return self.snapshot_state()

    def handle_teleop(self, client_id: str, payload: Dict[str, Any]) -> None:
        request = TeleopCommand.Request()
        request.client_id = client_id
        request.deadman = bool(payload.get('deadman', False))
        request.linear_x = float(payload.get('linear_x', 0.0))
        request.linear_y = float(payload.get('linear_y', 0.0))
        request.angular_z = float(payload.get('angular_z', 0.0))
        response = self._call_service(self.teleop_client, request, 2.0)
        if response is None or not response.success:
            message = response.message if response is not None else 'timed out'
            raise BridgeError(message)

    def handle_disconnect(self, client_id: str) -> None:
        request = TeleopCommand.Request()
        request.client_id = client_id
        request.deadman = False
        request.linear_x = 0.0
        request.linear_y = 0.0
        request.angular_z = 0.0
        self._call_service(self.teleop_client, request, 2.0)

    def _call_service(self, client: Any, request: Any, timeout_sec: float) -> Any:
        if not client.wait_for_service(timeout_sec=2.0):
            service_name = getattr(client, 'srv_name', 'unknown service')
            raise BridgeError(f'Service {service_name} is not available.')
        future = client.call_async(request)
        response = self._await_future(future, timeout_sec)
        return response

    def _await_future(self, future: Any, timeout_sec: float) -> Any:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.05)
        return None

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

    def _handle_orchestration_state(self, msg: OrchestrationState) -> None:
        with self.state_lock:
            self.current_mode = msg.current_mode or 'idle'
            self.launch_running = bool(msg.launch_running)
            self.is_navigating = bool(msg.is_navigating)
            self.is_paused = bool(msg.is_paused)
            self.pause_requested = bool(msg.pause_requested)
            self.last_error = msg.last_error
            if msg.map_yaml:
                self.default_map_yaml = Path(msg.map_yaml)
            if msg.map_save_prefix:
                self.map_save_prefix = Path(msg.map_save_prefix)
            if msg.has_navigation_goal:
                self.last_navigation_goal = (msg.navigation_goal_x, msg.navigation_goal_y)
            else:
                self.last_navigation_goal = None
            if self.current_mode != 'localization' or (
                not self.is_navigating and not self.is_paused and self.last_navigation_goal is None
            ):
                self.current_path_msg = None
                self.last_planned_path_msg = None
        self._mark_revision()

    def _display_path_locked(self) -> Optional[NavPath]:
        return self.current_path_msg or self.last_planned_path_msg

    def _mark_revision(self) -> None:
        with self.state_lock:
            self.telemetry_revision += 1


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
                try:
                    message = await websocket.receive_text()
                    payload = json.loads(message)
                    message_type = payload.get('type')
                    if message_type == 'teleop':
                        bridge.handle_teleop(client_id, payload)
                    elif message_type == 'ping':
                        await websocket.send_json({'type': 'pong'})
                except (ValueError, BridgeError) as exc:
                    await websocket.send_json({'type': 'error', 'message': str(exc)})
        except WebSocketDisconnect:
            pass
        finally:
            sender_task.cancel()
            bridge.handle_disconnect(client_id)

    return app


def main() -> None:
    bridge = SlambotBridgeNode(sys.argv)
    app = create_app(bridge)
    uvicorn.run(app, host=bridge.http_host, port=bridge.http_port, log_level='info')
