# paesano_vision

Subscribes to RealSense color, aligned depth, and color camera calibration. The first component
reports whether those streams are arriving; it does not yet project points or alter a map.

- Component: `paesano_vision::CameraIngest`
- Launch: `launch/camera_ingest.launch.py` (starts `realsense2_camera` too)
- Config: `config/camera_ingest.yaml`
- Inputs: `/camera/camera/color/image_raw`,
  `/camera/camera/aligned_depth_to_color/image_raw`,
  `/camera/camera/color/camera_info`
- Outputs: ROS logs reporting stream activity and image dimensions

```bash
ros2 launch paesano_vision camera_ingest.launch.py
```

On the Pi, the RealSense ROS wrapper must be installed before launching this package. The
standalone launch enables color, depth, frame synchronization, and depth alignment. Keep it
separate from the main robot launch while testing the camera. Check `ros2 topic list` and the
`camera_ingest` logs to confirm that the streams are arriving.
