package org.firstinspires.ftc.teamcode.util

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.subSystems.CamSS
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS

class PanelsDebugUtil(
    private val panels : TelemetryManager?
) {

    fun showAllDebug(
        gamepad1 : Gamepad,
        centerUtil : CenterUtil,
        firing : FiringUtil,
        spinDexer : SpinDexerSS,
        cam : CamSS
    ) {
        panels?.apply {
            debug("=== GAMEPAD ===")
            debug("RB pressed:", gamepad1.right_bumper)
            debug("Square:", gamepad1.square)
            debug("Triangle:", gamepad1.triangle)
            debug("Circle:", gamepad1.circle)
            debug("Cross released:", gamepad1.crossWasReleased())
            debug("")

            debug("=== STATE ===")
            debug("Is Centering:", centerUtil.isCentering())
            debug("Is Firing:", firing.isFiring())
            debug("")

            debug("=== POWER ===")
            debug("Center Power:", centerUtil.getPower())
            debug("Fly Wheel Power:", firing.flyWheelPower())
            debug("")

            debug("=== SPINDEXER ===")
            debug("SpinDexer Position:", spinDexer.position())
            debug("SpinDexer Raw Position:", spinDexer.rawPosition())
            debug("")

            debug("=== CAM ===")
            debug("Cam Position", cam.position())
            debug("Cam Raw Position", cam.rawPosition())
        }
    }

    fun showInit() {
        panels?.debug("Init Started")
    }

    fun update(telemetry: org.firstinspires.ftc.robotcore.external.Telemetry) {
        panels?.update(telemetry)
    }
}