package org.firstinspires.ftc.teamcode

import android.os.Build
import androidx.annotation.RequiresApi
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.*
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utils.Buttons
import java.lang.Math.abs


@Config
@TeleOp
class TeleOp : Rachel() {
    companion object {
        @Volatile @JvmField var moveSpeed = 0.7
        @Volatile @JvmField var moveSpeedFast = 1.0
        @Volatile @JvmField var moveSpeedSlow = 0.25
        @Volatile @JvmField var intakeSpeed = 0.65
        @Volatile @JvmField var hangLinearPos = -0.5
        @Volatile @JvmField var hang_myself = 4000
        @Volatile @JvmField var pixelHeight = 0.063
        @Volatile @JvmField var intakePosition = 0.29
        @Volatile @JvmField var extensionHeight = 500
        @Volatile @JvmField var isHeadless = false
        @Volatile @JvmField var headlessOffset = alliance.value * -TAU / 4
        @Volatile @JvmField var extensionUpCount = 80
        @Volatile @JvmField var extensionDownCount = 40
    }
    
    @RequiresApi(Build.VERSION_CODES.N)
    override fun runOpMode() {
        
        hwInit()
        
        var intakeStart = 0L
        var canHang = false
        var isFirstTimeInRange = true
        
        // drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        drive.pose = currentPose
        planeLauncher.position = HangPosRight.DOWN
        hoppers.forEach { it.position = HopperPos.IN }
        hangingLinear.power = HangLinearPose.DOWN
        hangingServo.position = HangPos.DOWN
        
        waitForStart()
        val startTime = System.currentTimeMillis()
        
        if (opModeIsActive()) while (opModeIsActive()) {
            if (abs(System.currentTimeMillis() - startTime - 60 * 1000) < 100)
                hangLinearPos = HangLinearPose.UP
            hangingLinear.power = hangLinearPos
            
            
            // Speed of the chassis
            val speed = if (gamepadA.isHeld(Buttons.LEFT_BUMPER)) moveSpeedSlow
            else if (gamepadA.isHeld(Buttons.RIGHT_BUMPER)) moveSpeedFast
            else moveSpeed
            
            // Shake buttons
            if (gamepadA.isHeld(Buttons.DPAD_UP)) {
                if ((0..3).random() == 0)
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, listOf(-speed, speed).random()), 0.0))
            } else if (gamepadA.isHeld(Buttons.DPAD_LEFT)) {
                if ((0..3).random() == 0)
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), listOf(-speed, speed).random()))
            } else if (gamepadA.isHeld(Buttons.DPAD_RIGHT)) {
                if ((0..3).random() == 0)
                    drive.setDrivePowers(PoseVelocity2d(Vector2d(listOf(-speed, speed).random(), 0.0), 0.0))
            } else
                drive.setDrivePowers(
                    PoseVelocity2d(
                        if (isHeadless) Vector2d(-gamepadA.leftY, gamepadA.leftX)
                            .rotated(headlessOffset - drive.pose.heading.toDouble()) * speed
                        else Vector2d(-gamepadA.leftY, gamepadA.leftX) * speed,
                        -gamepadA.rightX * speed
                    )
                )
            // if (gamepadA.justPressed(Buttons.BACK)) isHeadless = !isHeadless
            
            
            if (gamepadA.justPressed(Buttons.GUIDE))
                headlessOffset = drive.pose.heading.toDouble()
            if (gamepadB.justPressed(Buttons.GUIDE))
                for (vExt in vExts) vExt.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            
            
            // Intake
            if (gamepadA.justPressed(Buttons.X)) intakeStart = System.currentTimeMillis()
            
            
            // Intake with auto outtake if pixel is stuck
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
            
            // Extending intake
            if (gamepadA.justPressed(Buttons.RIGHT_STICK_BUTTON)) extensionIntake.position = intakePosition
            if (gamepadA.justPressed(Buttons.A)) extensionIntake.position += pixelHeight
            if (gamepadA.justPressed(Buttons.B)) extensionIntake.position -= pixelHeight
            extensionIntake.position += 0.06 * (gamepadA.rightTrigger - gamepadA.leftTrigger)
            // extensionIntake.position = extensionIntake.position.coerceIn(0.05..0.9)
            
            // Hanging & Plane
            if (gamepadB.justReleased(Buttons.RIGHT_BUMPER) && canHang) motorToPos(
                hanging,
                hanging.currentPosition,
                1.0
            )
            if (gamepadB.isPressed(Buttons.RIGHT_BUMPER) && canHang) {
                hanging.mode = DcMotor.RunMode.RUN_USING_ENCODER
                hanging.power = if (gamepadB.isPressed(Buttons.GUIDE)) -1.0 else 1.0
            }
            
            if (gamepadB.justPressed(Buttons.LEFT_STICK_BUTTON)) hangLinearPos = HangLinearPose.DOWN
            if (gamepadB.justPressed(Buttons.RIGHT_STICK_BUTTON)) hangLinearPos = HangLinearPose.UP
            
            if (gamepadB.leftStickX < -0.5) planeLauncher.position = HangPosRight.DOWN
            else if (gamepadB.leftStickX > 0.5) planeLauncher.position = HangPosRight.UP
            
            if (gamepadB.leftStickY < -0.5) hangingServo.position = HangPos.DOWN
            else if (gamepadB.leftStickY > 0.5) hangingServo.position = HangPos.UP
            
            if (gamepadA.justPressed(Buttons.BACK) || gamepadB.justPressed(Buttons.BACK)) {
                actions.clear()
                actions.add(launchPlane())
                canHang = true
            }
            // if (pixelStackDistance.getDistance(DistanceUnit.INCH) in 5.0..8.0) {
            //     if (gamepadA.isHeld(Buttons.LEFT_BUMPER)) {
            //         drive.setDrivePowers(
            //             PoseVelocity2d(
            //                 if (isHeadless) Vector2d(0.0, gamepadA.leftX)
            //                     .rotated(headlessOffset - drive.pose.heading.toDouble()) * speed
            //                 else Vector2d(0.0, gamepadA.leftX) * speed,
            //                 -gamepadA.rightX * speed
            //             )
            //         )
            //         if (isFirstTimeInRange) {
            //             extensionIntake.position = intakePosition + pixelHeight
            //             isFirstTimeInRange = false
            //         }
            //     }
            // } else isFirstTimeInRange = true
            
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
            
            
            // Hopper
            if (gamepadB.isPressed(Buttons.X) || vExts.all { it.currentPosition < extensionHeight })
                for (hopper in hoppers) hopper.position = HopperPos.IN
            else if (gamepadB.isPressed(Buttons.A))
                for (hopper in hoppers) hopper.position = HopperPos.MID
            else if (gamepadB.isPressed(Buttons.B))
                for (hopper in hoppers) hopper.position = HopperPos.OUT
            else if (gamepadB.justPressed(Buttons.LEFT_BUMPER)) for (hopper in hoppers) hopper.position = HopperPos.UP
            
            
            // Hopper Door
            hopperDoor.position = if (gamepadB.isPressed(Buttons.LEFT_TRIGGER)) HopperDoorPos.IN
            else if (gamepadB.isPressed(Buttons.RIGHT_TRIGGER)) HopperDoorPos.OUT
            else HopperDoorPos.CENTER
            
            
            if (gamepadB.justReleased(Buttons.LEFT_TRIGGER)) {
                if (!gamepadB.isHeld(Buttons.Y)) {
                    actions.clear()
                    actions.add(hopperAfterIn())
                }
            }
            
            // Vertical Extension
            if (gamepadB.isPressed(Buttons.DPAD_UP)) {
                var newPos = vExt1.currentPosition + extensionUpCount
                for (vExt in vExts) {
                    if (!gamepadB.isPressed(Buttons.START)) newPos =
                        newPos.coerceIn(extensionHeights.first()..extensionHeights.last())
                    motorToPos(vExt, newPos, vExtUp)
                    if (vExt.currentPosition == extensionHeights.last()) gamepadB.rumble(100)
                }
            } else if (gamepadB.isPressed(Buttons.DPAD_DOWN)) {
                var newPos = vExt1.currentPosition - extensionDownCount
                for (vExt in vExts) {
                    if (!gamepadB.isPressed(Buttons.START)) newPos =
                        newPos.coerceIn(extensionHeights.first()..extensionHeights.last())
                    motorToPos(vExt, newPos, vExtUp)
                    if (vExt.currentPosition == extensionHeights.first()) gamepadB.rumble(100)
                }
            }
            // if (gamepadB.justReleased(Buttons.DPAD_UP) || gamepadB.justReleased(Buttons.DPAD_DOWN))
            //     for (vExt in vExts) motorToPos(vExt, vExt.currentPosition, vExtUp)
            
            
            // Auto up for each pixel position
            if (gamepadB.justPressed(Buttons.DPAD_RIGHT)) for (vExt in vExts)
                extensionHeights.firstOrNull {
                    it > if (vExt.mode == DcMotor.RunMode.RUN_TO_POSITION) vExt.targetPosition else vExt.currentPosition + 10
                }.let { motorToPos(vExt, it ?: extensionHeights.last(), vExtUp) }
            else if (gamepadB.justPressed(Buttons.DPAD_LEFT)) {
                for (vExt in vExts) motorToPos(vExt, extensionHeights.first(), vExtDown)
                for (hopper in hoppers) hopper.position = HopperPos.IN
            }
            
            // lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE)
            with(pixel1.detectedPixelColor?.displayColor) {
                if (this != null) gamepadA.setLedColor(this, Gamepad.LED_DURATION_CONTINUOUS)
                if (this != null) gamepadB.setLedColor(this, Gamepad.LED_DURATION_CONTINUOUS)
            }
            
            // Telemetry
            telemetry.addLine("Headless: $isHeadless")
            telemetry.addLine("Alliance: ${alliance.name}, ${alliance.ordinal}, ${alliance.value}")
            telemetry.addLine("Side: ${side.name}, ${side.ordinal}, ${side.value}")
            telemetry.addLine("Pixel 1: ${pixel1.detectedPixelColor?.name ?: "No Pixel"}")
            telemetry.addLine("hsv: ${pixel1.hsv.joinToString(", ") { String.format("%.3f", it) }}")
            telemetry.addLine("Pixel 2: ${pixel2.detectedPixelColor?.name ?: "No Pixel"}")
            telemetry.addLine("hsv: ${pixel2.hsv.joinToString(", ") { String.format("%.3f", it) }}")
            // telemetry.addLine("Pixel 1 Dist: ${pixel1Dist.getDistance(DistanceUnit.MM)}")
            // telemetry.addLine("Pixel 2 Dist: ${pixel2Dist.getDistance(DistanceUnit.MM)}")
            telemetry.addData("Voltage", voltage.voltage)
            telemetry.addData("Test", stringFromJNI())
            telemetry.addData("Dist side", distUltrasonic.getDistance(DistanceUnit.CM))
            telemetry.addData("Hanging Velocity", hanging.velocity)
            telemetry.addData("Hanging Mode", hanging.mode)
            telemetry.addData("Hanging Current Position", hanging.currentPosition)
            telemetry.addData("Hanging Target Position", hanging.targetPosition)
            
            telemetry.addData("vExt1 Velocity", vExt1.velocity)
            telemetry.addData("vExt1 Mode", vExt1.mode)
            telemetry.addData("vExt1 Current Position", vExt1.currentPosition)
            telemetry.addData("vExt1 Target Position", vExt1.targetPosition)
            
            telemetry.addData("vExt2 Velocity", vExt2.velocity)
            telemetry.addData("vExt2 Mode", vExt2.mode)
            telemetry.addData("vExt2 Current Position", vExt2.currentPosition)
            telemetry.addData("vExt2 Target Position", vExt2.targetPosition)
            
            telemetry.addData("Back Distance 1", backDist1.getDistance(DistanceUnit.CM))
            // telemetry.addData("Back Distance 2", backDist2.getDistance(DistanceUnit.CM))
            telemetry.addData("Pixel Stack Distance", pixelStackDistance.getDistance(DistanceUnit.INCH))
            
            for (motor in hardwareMap.dcMotor) if ((motor as DcMotorEx).isOverCurrent)
                telemetry.addData("MOTOR OVER CURRENT", motor.deviceName)
            
            update()
        }
        currentPose = drive.pose
    }
    
    fun launchPlane(): Action {
        return SequentialAction(
            InstantAction { planeLauncher.position = HangPosRight.PLANE },
            SleepAction(.5),
            InstantAction { drone.position = 0.5 },
            SleepAction(.5),
            InstantAction {
                planeLauncher.position = HangPosRight.DOWN
                drone.position = 0.0
                hangingServo.position = HangPos.UP
            }
        )
    }
    
    fun hopperAfterIn(): Action {
        return SequentialAction(
            InstantAction { for (hopper in hoppers) hopper.position = HopperPos.MID },
            SleepAction(0.2),
            InstantAction { for (hopper in hoppers) hopper.position = HopperPos.OUT }
        )
    }
    
    
    fun extUpAndOut(): Action {
        return SequentialAction(
            InstantAction { for (vExt in vExts) motorToPos(vExt, extensionHeights[3], vExtUp) },
            SleepAction(0.7),
            InstantAction { for (hopper in hoppers) hopper.position = HopperPos.MID },
        )
    }
}