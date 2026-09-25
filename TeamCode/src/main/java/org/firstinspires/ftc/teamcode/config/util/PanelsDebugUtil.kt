package org.firstinspires.ftc.teamcode.config.util

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.subSystem.FlowerSS

class PanelsDebugUtil(
    private val panels: TelemetryManager?
) {
    fun showAllDebugAuto(
        follower: Follower,
        alliance: Robot.Alliance,
        runtime: Double
    ) {
        panels?.apply {
            debug("=== PedroPathing ===")
            debug("Follower Pose X", follower.pose().x())
            debug("Follower Pose Y", follower.pose().y())
            debug("Follower Heading", Math.toDegrees(follower.pose().heading()))
            debug("Follower Distance Remaining", follower.remainingDistance())
            debug("Follower IsBusy", follower.isBusy)
            debug("")
            debug("=== OpMode ===")
            debug("runtime", runtime)
            debug("Alliance", alliance.name)
        }
    }

    fun showAllDebugTeleop(
        follower: Follower,
        alliance: Robot.Alliance,
        runtime: Double,
        gamepad: Gamepad,
        flowerSS: FlowerSS,
        robot: Robot,
    ) {
        panels?.apply {
            debug("=== PedroPathing ===")
            debug("Follower Pose X", follower.pose().x())
            debug("Follower Pose Y", follower.pose().y())
            debug("Follower Heading", Math.toDegrees(follower.pose().heading()))
            debug("Dist From Ref Pose", follower.pose().distance(robot.refPose))
            debug("")
            debug("=== HardWare ===")
            debug("Flower position", robot.flowerS.rawPosition)
            debug("Intake Speed", robot.intakeM.velocity)
            debug("")
            debug("=== Gamepad ===")
            debug("Left Stick X", gamepad.left_stick_x)
            debug("Left Stick Y", gamepad.left_stick_y)
            debug("Right Stick X", gamepad.right_stick_x)
            debug("Left Bumper", gamepad.left_bumper)
            debug("Right Bumper", gamepad.right_bumper)
            debug("")
            debug("=== OpMode ===")
            debug("runtime", runtime)
            debug("Alliance", alliance.name)
        }
    }

    fun showInit() {
        panels?.debug("Init Started")
    }
    fun update(telemetry: org.firstinspires.ftc.robotcore.external.Telemetry) {
        panels?.update(telemetry)
    }
}