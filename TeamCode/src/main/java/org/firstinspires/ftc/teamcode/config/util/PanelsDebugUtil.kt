package org.firstinspires.ftc.teamcode.config.util

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.Gamepad

class PanelsDebugUtil(
    private val panels : TelemetryManager?
) {
    fun showAllDebug(
        gamepad1 : Gamepad,
        follower: Follower,
        hubUtil: HubUtil,
        runtime: Double
    ) {
        panels?.apply {
            debug("=== PedroPathing ===")
            debug("Follower Pose X", follower.pose.x)
            debug("Follower Pose Y", follower.pose.y)
            debug("Follower Heading", Math.toDegrees(follower.pose.heading))
            debug("Velocity Mag", follower.velocity.magnitude)
            debug("Total Heading", Math.toDegrees(follower.totalHeading))
            debug("Follower CurrentPath", follower.currentPath)
            debug("Follower CurrentPathChain", follower.currentPathChain)
            debug("Follower IsBusy", follower.isBusy)
            debug("Follower IsStuck", follower.isRobotStuck)
            debug("follower IsTurning", follower.isTurning)
            debug("")
            debug("=== GamePad ===")
            debug("Left Stick Y", gamepad1.left_stick_y)
            debug("Left Stick X", gamepad1.left_stick_x)
            debug("Right Stick Y", gamepad1.right_stick_y)
            debug("Right Stick X", gamepad1.right_stick_x)
            debug("")
            debug("=== Hub ===")
            debug("Temp F", hubUtil.getTempFahrenheit())
            debug("Temp C", hubUtil.getTempCelsius())
            debug("Temp K", hubUtil.getTempKelvin())
            debug("")
            debug("=== OpMode ===")
            debug("runtime", runtime)
            debug("")
        }
    }
    fun showInit() {
        panels?.debug("Init Started")
    }
    fun update(telemetry: org.firstinspires.ftc.robotcore.external.Telemetry) {
        panels?.update(telemetry)
    }
}