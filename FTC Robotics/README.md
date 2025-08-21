This document contains the Autonomous OpMode from the 2025 season and the TeleOp OpMode from the 2024 season.

At this [Youtube Link](https://www.youtube.com/shorts/U5X9nA4jx2Q) you can find a solo match from the 2025 robot which advanced to the _FIRST_ World Championship

# 🤖 FTC Robot OpMode Documentation

---

## Auto 2025 – FTC Autonomous OpMode

This document details an **FTC autonomous mode** (`Autov3`) designed for advanced trajectory planning using [Road Runner](https://acmerobotics.github.io/road-runner/) and integrated mechanism control for scoring, sample pickup, and transfer operations.

### 🌐 Key Capabilities

- **Dynamic Path Planning:** Utilizes Road Runner's `ActionBuilder` for generating smooth spline paths and coordinating mechanism actions during robot movement.

- **Configurable via FTC Dashboard:** Allows real-time tuning of parameters like positions, speeds, and delays using `@Config` annotations for quick adjustments.

- **Runtime UI Setup:** Provides a built-in UI on the Driver Station for selecting crucial setup parameters:

  - **Alliance:** Red or Blue

  - **Side:** Left or Right

  - **Start Position:** Default or Custom (using distance sensors)

  - **Delays:** Options for start and scoring delays.

- **Integrated Scoring Support:**

  - **Basket Scoring:** Optimized for the Left side of the field.

  - **High Rung Scoring:** Optimized for the Right side of the field.

- **Automated Sample Handling:** Includes precise arm, extension, and gripper control for picking up and transferring multiple samples.

---

### 🔧 How It Works

The autonomous sequence follows a structured flow:

1.  **Hardware Initialization:** Encoders are reset, and all mechanisms (arm, gripper, extension) are set to their predefined initial positions.

2.  **Driver Station UI Interaction:** Drivers configure alliance, side, start position, and delays before the match begins.

3.  **Pose Initialization:** The robot's starting position is calculated based on the selected field side or input from distance sensors for precise placement.

4.  **Trajectory Execution:**

    - **Right Side:** The robot executes a sequence involving high rung scoring, followed by multiple sample pickups, and repeated scoring cycles.

    - **Left Side:** The robot performs basket scoring, picks up samples, and then executes transfer and additional scoring actions.

5.  **Mechanism Actions:** Specific helper functions (`scoreRung()`, `scoreBasket()`, `pickupSample()`, `releaseSample()`, `allDown()`) are triggered as `InstantAction`, `SequentialAction`, or `ParallelAction` within the Road Runner trajectories to synchronize robot motion with mechanism operations.

---

### 📊 Configurable Parameters (via FTC Dashboard)

| **Parameter**        | **Description**                       |
| :------------------- | :------------------------------------ |
| `spikePickupX/Y`     | Coordinates for spike mark pickup     |
| `samplePickupPosX/Y` | Coordinates for generic sample pickup |
| `sample1PosX`        | X-coordinate for the first sample     |
| `sample2PosX`        | X-coordinate for the second sample    |
| `sample3PosX`        | X-coordinate for the third sample     |
| `pickupSpeed1`       | Drive speed during the first pickup   |
| `pickupSpeed2`       | Drive speed during the second pickup  |
| `scoreRungDelay`     | Delay before scoring on the rung      |
| `releaseSampleDelay` | Delay before releasing a sample       |
| `pickupSampleDelay`  | Delay before picking up a sample      |

---

### 📚 Technical Details & Libraries

- **Road Runner:** The primary library for generating and following complex trajectories.

- **FTC Dashboard:** Used extensively for live parameter tuning, enabling rapid iteration and optimization of autonomous paths and mechanism timings.

- **OpenCV (Planned):** Future integration for AprilTag detection to enhance vision-based positioning accuracy.

- **Core Helper Functions:**

  - `normalizedPose(x, y, heading)`: Normalizes position coordinates and heading for Road Runner.

  - `facing(currentPos, targetPos)`: Calculates the required heading to face a specific target.

  - `samplePos(sampleNum)`: Retrieves the position for a given sample number.

  - **Action Sequences:** `pickupSample()`, `releaseSample()`, `extendHorizontal()`, `retractHorizontal()`.

---

### Autonomous Flow Overview

#### **Pre-Start**

- Driver UI selections are processed to set the robot's initial pose.

#### **On Start**

- The configured start delay is applied.

- The autonomous action sequence begins.

#### **Execution**

- The robot follows predefined Road Runner trajectories.

- Mechanism actions are executed precisely:

  - `InstantAction` for immediate, single commands.

  - `SequentialAction` for a series of ordered tasks.

  - `ParallelAction` for simultaneous operations.

#### **End**

- The robot moves to a final parking or designated position.

---

---

## 🎮 TeleOp – FTC Driver-Controlled OpMode

This file defines the **driver-controlled (TeleOp) mode** for an FTC robot, integrating **Road Runner** for precise motion control with a comprehensive suite of mechanisms for intake, scoring, hanging, and plane launching. It uses **sensor feedback** and **driver inputs** for responsive and robust control during the TeleOp period.

### Key Capabilities

- **Field-Centric (Headless) Driving:** Offers both normal (robot-centric) and headless (field-centric) control modes. The headless offset can be reset on the fly.

- **Configurable via FTC Dashboard:** All key operational parameters (drive speeds, intake speeds, extension limits, mechanism positions) are tunable in real time using `@Config`.

- **Dynamic Speed Control:** Drivers can instantly switch between **slow**, **normal**, and **fast** drive speeds using gamepad bumpers, adapting to different game scenarios.

- **Automated Mechanism Behaviors:**

  - **Pixel Jam Detection & Auto-Reverse:** Automatically detects if the intake velocity drops below a threshold (indicating a jam). If a jam is detected, the system temporarily reverses the intake and bottom roller to clear the obstruction.

  ```
  if (gamepadA.isHeld(Buttons.X)) {
      val reverseDuration = 350
      val reverseThreshold = 100.0

      val intakeVelocity = intake.velocity
      if (intakeVelocity < reverseThreshold && (System.currentTimeMillis() - intakeStart) > 250) {
          val reverseStart = System.currentTimeMillis()
          if (System.currentTimeMillis() - reverseStart < reverseDuration) {
              intake.power = -intakeSpeed
              bottomRoller.power = -1.0
          }
      } else {
          intake.power = intakeSpeed
          bottomRoller.power = 1.0
      }
  } else if (gamepadA.isHeld(Buttons.Y)) {
      intake.power = -intakeSpeed
      bottomRoller.power = -1.0
  } else {
      intake.power = 0.0
      bottomRoller.power = 0.0
  }
  ```

  - **Extension Position Adjustments:** Fine control of the extension system using stick buttons and triggers.

  - **Hanging System:** Supports both **manual** and **assisted modes** for robust hanging operations.

  - **✈️ Plane Launcher & Drone Deployment:** Pre-programmed sequences are triggered via Road Runner Actions for reliable launch.

- **Distance Sensor Safety:** Automatically limits backward movement when the robot is too close to the backdrop (within 15 cm) to prevent collisions and disturbing previously placed pixels.

  ```
  if (backDist1.getDistance(DistanceUnit.CM) in 0.0..15.0) {
      if (gamepadA.isHeld(Buttons.LEFT_BUMPER)) {
          drive.setDrivePowers(
              PoseVelocity2d(
                  if (isHeadless) Vector2d(0.0, gamepadA.leftX)
                      .rotated(headlessOffset - drive.pose.heading.toDouble()) * speed
                  else Vector2d(0.0, gamepadA.leftX) * speed,
                  -gamepadA.rightX * speed
              )
          )
      }
  }
  ```

- **🎨Color Detection Feedback:** Provides visual feedback through gamepad LED indicators, reflecting the color of detected pixels using onboard color sensors.

### 🔧 How It Works

1.  **Hardware Initialization:** All motors, servos, and sensors are initialized to their default states (e.g., hoppers closed, hanging mechanism down, plane launcher in default position).

2.  **Driver Control System:** Input is managed across two gamepads:

    - **Gamepad A:** Dedicated to main robot movement, intake, and extension control.

    - **Gamepad B:** Manages hanging, hopper, and plane launcher operations.

3.  **Drive System:** Leverages **Road Runner's `PoseVelocity2d`** for smooth, omnidirectional movement. An optional **headless mode** simplifies field-centric driving.

4.  **Mechanism Control:**

    - **Intake & Bottom Roller:** Pressing **X** activates intake. If a jam is detected (low intake velocity), the system briefly reverses the rollers to clear it.

    - **Hopper Positions:** Supports multiple predefined positions (IN, MID, OUT, UP) for efficient pixel handling.

    - **Vertical Extension (Lift):** Allows incremental up/down movements via D-Pad and quick presets.

    - **Hanging Mechanism:** Features both manual and automated power modes with integrated safeguards.

    - **Plane Launch & Drone Drop:** Triggered by the `launchPlane()` action sequence.

    ```
    fun launchPlane(): Action {
        return SequentialAction(
            InstantAction { planeLauncher.position = HangPosRight.PLANE },
            SleepAction(0.5),
            InstantAction { drone.position = 0.5 },
            SleepAction(0.5),
            InstantAction {
                planeLauncher.position = HangPosRight.DOWN
                drone.position = 0.0
                hangingServo.position = HangPos.UP
            }
        )
    }
    ```

5.  **Sensor Integration:**

    - **Distance Sensor (`backDist1`):** Prevents the robot from moving too far backward when near walls or the backdrop.

    - **Pixel Detection Sensors (`pixel1`, `pixel2`):** Provide real-time pixel color and HSV values to telemetry and drive gamepad LED feedback.

### Configurable Parameters (via FTC Dashboard)

| **Parameter**     | **Description**                                |
| :---------------- | :--------------------------------------------- |
| `moveSpeed`       | Default drive power for normal movement        |
| `moveSpeedFast`   | Drive power when the right bumper is held      |
| `moveSpeedSlow`   | Drive power when the left bumper is held       |
| `intakeSpeed`     | Power level for the intake rollers             |
| `pixelHeight`     | Increment value for adjusting intake extension |
| `extensionHeight` | Maximum allowed extension height               |
| `headlessOffset`  | Reference heading for field-centric control    |

### 📚 Technical Details & Libraries

- **Road Runner:** Essential for smooth drive control and defining action sequences within TeleOp.

- **FTC Dashboard:** The primary tool for real-time parameter tuning and comprehensive telemetry monitoring.

- **Sensors:**

  - **Distance Sensors:** Utilized for obstacle avoidance and positional assistance.

  - **Color Sensors:** Enable pixel detection and visual feedback via gamepad LEDs.

- **Core Helper Functions:**

  - `launchPlane()`: Executes the sequence for deploying the plane and drone using sequential actions.

  - `hopperAfterIn()`: Moves the hopper from the `IN` position to `MID`, then `OUT`.

  - `extUpAndOut()`: Raises the extension and adjusts the hopper for outtake.

  - `motorToPos()`: Drives a motor to a specified encoder position.

### 🎛️ Driver Control Overview

#### **Gamepad A (Driver)**

- **Left Stick:** Controls drive translation (forward/backward, strafing).

- **Right Stick X:** Controls robot rotation.

- **Left Bumper:** Activates slow drive mode.

- **Right Bumper:** Activates fast drive mode.

- **X Button:** Engages intake (with jam detection and auto-reverse).

- **Y Button:** Activates reverse intake.

- **A / B Buttons:** Adjust intake extension incrementally.

- **Right Stick Button:** Resets intake extension to default.

- **Triggers:** Provide fine-tuning control for intake extension.

- **BACK Button:** Triggers the plane launch sequence.

#### **Gamepad B (Operator)**

- **D-Pad Up/Down:** Incremental vertical extension movement.

- **D-Pad Left/Right:** Selects preset extension positions.

- **X / A / B Buttons:** Control hopper positions (IN / MID / OUT).

- **Left Bumper:** Moves hopper to the UP position.

- **Triggers:** Control the hopper door.

- **Left Stick X / Y:** Controls the plane launcher and hanging servo.

- **Right Bumper:** Activates manual hanging control.

- **Stick Buttons:** Selects preset hanging linear positions.

### Telemetry Data

The FTC Dashboard displays critical real-time data:

- **Headless Mode Status**

- **Alliance & Side Information**

- **Detected Pixel Colors & HSV Values**

- **Distance Sensor Readings**

- **Battery Voltage & Motor Diagnostics**

- **Extension & Hanging Motor States**

### Key Autonomous Behaviors in TeleOp

- **Jam Detection:** If the intake motor stalls or its velocity drops, the rollers automatically reverse for a brief period to clear any lodged pixels.

- **Distance Guard:** Automatically limits backward robot movement when obstacles or the backdrop are detected within a safe range, preventing collisions.

- **Pixel Detection Feedback:** Gamepad LEDs visually indicate the color of detected pixels, providing immediate feedback to the driver.

- **Safe Extensions:** All extension movements are constrained within predefined limits to prevent mechanical damage or over-extension.
