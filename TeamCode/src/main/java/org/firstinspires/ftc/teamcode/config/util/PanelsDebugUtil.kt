package org.firstinspires.ftc.teamcode.config.util

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower

class PanelsDebugUtil(
    private val panels : TelemetryManager?
) {
    fun showAllDebug(
        follower: Follower,
        hubUtil: HubUtil
    ) {
        panels?.apply {
            debug("=== PedroPathing ===")
            debug("Follow Pose X", follower.pose.x)
            debug("Follow Pose X", follower.pose.y)
            debug("Follow Heading", follower.pose.heading)
            debug("Follow IsBusy", follower.isBusy)
            debug("")
            debug("=== Hub ===")
            debug("Temp F", hubUtil.getTempFahrenheit())
            debug("Temp C", hubUtil.getTempCelsius())
            debug("Temp K", hubUtil.getTempKelvin())
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