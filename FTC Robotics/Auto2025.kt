
package org.firstinspires.ftc.teamcode

import android.os.Build
import androidx.annotation.RequiresApi
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import org.checkerframework.framework.qual.JavaExpression
import org.firstinspires.ftc.teamcode.UI.BooleanField
import org.firstinspires.ftc.teamcode.UI.NumberField
import org.firstinspires.ftc.teamcode.UI.SelectField
import org.firstinspires.ftc.teamcode.UI.UI
import org.openftc.apriltag.AprilTagDetection
import kotlin.math.pow

@Config
@Autonomous
open class Autov3 : Rachel() {
    private var signalTag: AprilTagDetection? = null
    
    companion object {
        @Volatile @JvmField var spikePickupX = 59.5
        @Volatile @JvmField var spikePickupY = 41.25
        @Volatile @JvmField var samplePickupPosX = 38.0
        @Volatile @JvmField var samplePickupPosY = 58.5
        @Volatile @JvmField var samplePickupPosIncrement = 0.5
        @Volatile @JvmField var sample1PosX = 46.0
        @Volatile @JvmField var sample2PosX = 66.0
        @Volatile @JvmField var sample3PosX = 93.0
        @Volatile @JvmField var samplePosY = 20.0
        @Volatile @JvmField var scoreY = 36.0
        @Volatile @JvmField var basketPos = 60.0
        @Volatile @JvmField var pickupSpeed1 = 25.0
        @Volatile @JvmField var pickupSpeed2 = 15.0
        @Volatile @JvmField var scoreRungDelay = 0.3
        @Volatile @JvmField var releaseSampleDelay = 0.25
        @Volatile @JvmField var scoreBasketDelay = 0.2
        @Volatile @JvmField var pickupSampleDelay = 0.25
    }
    
    val basketScorePose = normalizedPose(basketPos, basketPos, TAU * 1 / 8)
    
    
    @RequiresApi(Build.VERSION_CODES.N)
    override fun runOpMode() {
        hwInit()
        hZAxis.position = GripperPos.Down
        hRotation.position = GripperPos.Mid
        hPincher.position = GripperPos.Open
        vArm.position = VerticalArmPos.Down
        
        vZAxis.position = VerticalZAxisPos.Initial
        vPincher.position = VerticalGripper.Close
        
        for (vExt in vExts) vExt.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        
        val (
            isRed,
            isLeft,
            isDefaultStartPos,
            startDelay,
            scoreDelay,
        ) = UI(
            this, gamepadA, arrayOf(
                SelectField("Alliance", Alliance.entries.map { it.name }.toTypedArray(), alliance.ordinal),
                SelectField("Side", Side.entries.map { it.name }.toTypedArray(), side.ordinal),
                BooleanField("Default Start Pos", true),
                NumberField("Start Delay", 0, 20),
                NumberField("Score Delay", 0, 20),
            )
        ).config.map { it.value }
        
        alliance = Alliance.entries[isRed]
        side = Side.entries[isLeft]
        
        // Look for custom element until start is pressed
        while (!isStarted && !isStopRequested) {
            // customElementDetectionPipeline.lastDetection?.let { spikeMark = it }
            // telemetry.addLine("Detected element position=${spikeMark.name}")
            telemetry.addData("Dist Left Side", leftDist.distance)
            telemetry.addData("Dist Right Side", rightDist.distance)
            telemetry.addData("Selected Side", side.name)
            telemetry.update()
            runBlocking(SleepAction(0.02))
        }
        if (isDefaultStartPos == 1)
            currentPose =
                if (side == Side.LEFT) normalizedPose(24.0 + 8.0, 62.0, TAU / 4)
                else normalizedPose(24.0 - 8.0, 62.0, -TAU / 4)
        else {
            val distToSide = if (side == Side.LEFT) leftDist else rightDist
            currentPose =
                normalizedPose(72.0 + 9 - distToSide.distance, 62.0, if (side == Side.LEFT) TAU / 4 else -TAU / 4)
        }
        drive.localizer.pose = currentPose
        
        waitForStart()
        if (opModeIsActive()) {
            val startTime = System.currentTimeMillis()
            runBlocking(SleepAction(startDelay.toDouble()))
            
            if (side == Side.RIGHT) {
                runBlocking(
                    drive.actionBuilder(drive.localizer.pose)
                        .setTangent(normalizeAngle(TAU / 4))
                        .afterDisp(0.0, highRung())
                        .splineToConstantHeading(
                            normalizedVector(4.0, scoreY),
                            normalizeAngle(TAU / 4),
                            { _, _, _ -> 25.0 }
                        )
                        .afterDisp(0.0, scoreRung())
                        .splineToConstantHeading(
                            normalizedVector(3.5, scoreY),
                            normalizeAngle(-TAU / 4),
                            { _, _, _ -> 5.0 }
                        )
                        .afterDisp(1.0, allDown())
                        .splineToSplineHeading(
                            normalizedPose(24.0, 42.0, normalizeAngle(TAU / 2)),
                            normalizeAngle(TAU / 2),
                            { pose, _, _ ->
                                pose.position.value().minus(normalizedVector(1.0, scoreY)).norm().div(4).pow(2)
                                    .plus(20)
                                    .coerceIn(20.0, 40.0)
                            }
                        )
                        .afterDisp(1.0, extendHorizontal())
                        .splineToSplineHeading(
                            normalizedPose(
                                spikePickupX,
                                spikePickupY,
                                normalizeAngle(facing(normalizedVector(spikePickupX, spikePickupY), samplePos(0)))
                            ), normalizeAngle(TAU / 2)
                        )
                        .stopAndAdd(pickupSample())
                        .afterTime(0.4, releaseSample())
                        .turnTo(TAU / 4)
                        .let {
                            var builder = it
                            for (i in 1 until 3) builder = builder
                                .afterTime(0.0,
                                    InstantAction {
                                        hExt.position =
                                            if (i == 1) HorizontalExtPos.Sample2 else HorizontalExtPos.Sample3
                                        hRotation.position =
                                            if (i == 1) GripperPos.Sample2Rot else GripperPos.Sample3Rot
                                    })
                                .turn(-(TAU / 4 - facing(normalizedVector(spikePickupX, spikePickupY), samplePos(i))))
                                .stopAndAdd(pickupSample())
                                .afterTime(if (i == 1) 0.6 else 0.8, releaseSample())
                                .turn(TAU / 4 - facing(normalizedVector(spikePickupX, spikePickupY), samplePos(i)))
                            
                            builder
                        }
                        .afterDisp(0.0, retractHorizontal())
                        .let {
                            var builder = it
                            for (i in 1 until 5) builder = builder
                                .splineToConstantHeading(
                                    normalizedVector(samplePickupPosX, 55.0), TAU / 4,
                                    { _, _, _ -> pickupSpeed1 }
                                )
                                .afterDisp(3.5, InstantAction{ vPincher.position = VerticalGripper.Close })
                                .splineToConstantHeading(
                                    normalizedVector(samplePickupPosX, samplePickupPosY + samplePickupPosIncrement * i),
                                    TAU / 4,
                                    { _, _, _ -> pickupSpeed2 }
                                )
                                .splineToConstantHeading(
                                    normalizedVector(
                                        samplePickupPosX,
                                        samplePickupPosY + 0.5 + samplePickupPosIncrement * i
                                    ),
                                    TAU / 4,
                                    { _, _, _ -> pickupSpeed2 }
                                )
                                .setTangent(normalizeAngle(TAU / 8))
                                .afterDisp(0.5, highRung())
                                .splineToConstantHeading(
                                    normalizedVector(4.0 + i * 2, scoreY),
                                    normalizeAngle(TAU / 4),
                                    { pose, _, _ ->
                                        pose.position.value().minus(normalizedVector(4.0 + i * 2, scoreY)).norm().times(2)
                                            .plus(20)
                                            .coerceIn(20.0, 30.0)
                                    }
                                )
                                .afterDisp(0.0, scoreRung())
                                .splineToConstantHeading(
                                    normalizedVector(3.5 + i * 2, scoreY - 0.5),
                                    normalizeAngle(-TAU / 4),
                                    { _, _, _ -> 5.0 }
                                )
                                .afterDisp(1.0, allDown())
                            builder
                        }
                        .afterDisp(0.0, extendHorizontal())
                        .splineToConstantHeading(
                            normalizedVector(7.5, 42.0),
                            normalizeAngle(-TAU / 4),
                            { _, _, _ -> 10.0 }
                        )
                        .build(),
                )
            } else if (side == Side.LEFT) runBlocking(
                drive.actionBuilder(drive.localizer.pose)
                    .setTangent(normalizeAngle(TAU / 4))
                    .afterDisp(
                        0.0, SequentialAction(
                            basket(ExtPos.upperLimitVertical),
                            InstantAction {
                                vArm.position = VerticalArmPos.Basket
                                vZAxis.position = VerticalZAxisPos.Rung
                            },
                        )
                    )
                    .splineToSplineHeading(basketScorePose, normalizeAngle(TAU * 5 / 8), { _, _, _ -> 15.0 })
                    .stopAndAdd(scoreBasket())
                    .let {
                        var builder = it
                        for (i in 0 until 3) builder = builder
                            .afterDisp(1.0, ParallelAction(allDown(), extendHorizontal()))
                            .splineToSplineHeading(
                                normalizedPose(
                                    spikePickupX, spikePickupY,
                                    normalizeAngle(facing(normalizedVector(spikePickupX, spikePickupY), samplePos(i)))
                                ),
                                normalizeAngle(0.0),
                                { _, _, _ -> 15.0 }
                            )
                            .stopAndAdd(pickupSample())
                            .afterDisp(0.0, transferSample())
                            .setTangent(normalizeAngle(-TAU / 4))
                            .splineToSplineHeading(basketScorePose, normalizeAngle(TAU * 5 / 8), { _, _, _ -> 15.0 })
                            .stopAndAdd(scoreBasket())
                        builder
                    }
                    .splineToSplineHeading(normalizedPose(48.0, 12.0, normalizeAngle(0.0)), normalizeAngle(0.0))
                    .splineToSplineHeading(normalizedPose(24.0, 12.0, normalizeAngle(0.0)), normalizeAngle(0.0))
                    .build()
            )
        }
    }
    
    fun scoreRung() = SequentialAction(
        basket(ExtPos.UpperRungScore),
        SleepAction(scoreRungDelay),
        InstantAction { vPincher.position = VerticalGripper.Open }
    )
    
    fun allDown() = SequentialAction(
        basket(0),
        InstantAction {
            vArm.position = VerticalArmPos.DownAuto
            vZAxis.position = VerticalZAxisPos.DownAuto
            vPincher.position = VerticalGripper.Open
        }
    )
    
    fun extendHorizontal() = SequentialAction(
        InstantAction {
            hExt.position = HorizontalExtPos.Sample1
            hRotation.position = GripperPos.Sample1Rot
            hPincher.position = GripperPos.Open
        },
    )
    
    fun retractHorizontal() = SequentialAction(
        InstantAction {
            hExt.position = HorizontalExtPos.In
            hRotation.position = GripperPos.Front
        },
    )
    
    fun samplePos(sampleNum: Int) = normalizedVector(
        if (sampleNum == 0) sample1PosX else if (sampleNum == 1) sample2PosX else sample3PosX,
        samplePosY
    )
    
    fun facing(currentPos: Vector2d, targetPos: Vector2d) = (targetPos - currentPos).angleCast().toDouble()
    
    fun pickupSample() = SequentialAction(
        InstantAction { hPincher.position = GripperPos.Close },
        SleepAction(pickupSampleDelay),
        InstantAction {
            if (side == Side.RIGHT)
                hExt.position = HorizontalExtPos.Out
            else {
                hZAxis.position = GripperPos.Up
            }
        }
    )
    
    fun releaseSample() = SequentialAction(
        InstantAction { hPincher.position = GripperPos.Open },
        SleepAction(releaseSampleDelay)
    )
    
    fun scoreBasket() = SequentialAction(
        InstantAction { vZAxis.position = VerticalZAxisPos.Basket },
        SleepAction(scoreBasketDelay),
        InstantAction { vPincher.position = VerticalGripper.Open }
    )
    
}
