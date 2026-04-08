# mecanum_drive_controller

Low-level mecanum base controller for Paesano hardware.

## Node

- executable: `mecanum_drive_controller_exec`
- config: `config/mecanum_drive_controller.yaml`

## Default interfaces

- velocity input: `/cmd_vel`
- encoder output: `/wheel_encoders`

## Launch

```bash
ros2 launch mecanum_drive_controller mecanum_drive_controller.launch.py
```
