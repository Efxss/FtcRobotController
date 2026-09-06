package org.firstinspires.ftc.teamcode.config.util

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.subSystem.LLSS

class PanelsDebugUtil(
    private val panels: TelemetryManager?
) {
    fun showAllDebugAuto(
        follower: Follower,
        hubUtil: HubUtil,
        alliance: Robot.Alliance,
        runtime: Double
    ) {
        panels?.apply {
            debug("=== PedroPathing ===")
            debug("Follower Pose X", follower.pose.x)
            debug("Follower Pose Y", follower.pose.y)
            debug("Follower Distance Traveled On Path", follower.distanceTraveledOnPath)
            debug("Follower Distance Remaining", follower.distanceRemaining)
            debug("Follower Heading", Math.toDegrees(follower.pose.heading))
            debug("Follower Total Heading", Math.toDegrees(follower.totalHeading))
            debug("Follower IsBusy", follower.isBusy)
            debug("Follower IsStuck", follower.isRobotStuck)
            debug("follower IsTurning", follower.isTurning)
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
        gp: GamepadEx,
        limelight: LLSS,
        llDeadZone: Double,
        robot: Robot
    ) {
        panels?.apply {
            debug("=== PedroPathing ===")
            debug("Follower Pose X", follower.pose.x)
            debug("Follower Pose Y", follower.pose.y)
            debug("Follower Heading", Math.toDegrees(follower.pose.heading))
            debug("Total Heading", Math.toDegrees(follower.totalHeading))
            debug("")
            debug("=== Vision ===")
            debug("Target X Deg", limelight.currentTagXDeg(alliance, llDeadZone, robot))
            debug("")
            debug("=== Gamepad ===")
            debug("Left Stick X", gp.gamepad.left_stick_x)
            debug("Left Stick Y", gp.gamepad.left_stick_y)
            debug("Right Stick X", gp.gamepad.right_stick_x)
            debug("Right Bumper", gp.gamepad.right_bumper)
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